# Autonomous filament buffer feeding and loading (runs on the MCU, see
# src/buffer_feed.c)
#
# Copyright (C) 2026  CR-3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

REASON_NAMES = {0: "none", 1: "done", 2: "stop_sensor", 3: "aborted",
                4: "fault", 5: "busy", 6: "load_timeout", 7: "loaded"}
STATE_NAMES = {0: "idle", 1: "feeding", 2: "fault"}
REASON_ABORTED = 3
REASON_FAULT = 4
REASON_BUSY = 5
REASON_LOAD_TIMEOUT = 6
MIN_INTERVAL_S = 40e-6        # maximum feed rate: 25 kHz
MAX_INTERVAL_TICKS = 1 << 23  # must match MAX_INTERVAL in buffer_feed.c
FP_SHIFT = 8

class BufferFeed:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.name = config.get_name().split()[-1]
        # Stepper that receives the extra steps
        self.mcu_stepper = self._lookup_stepper(config, config.get('stepper'))
        self.mcu = self.mcu_stepper.get_mcu()
        # Sensor pins (may be shared with buttons / gcode_button)
        ppins = self.printer.lookup_object('pins')
        self.trigger = self._lookup_pin(ppins, config.get('trigger_pin'))
        self.stop = self._lookup_pin(ppins, config.get('stop_pin', None))
        self.gate = self._lookup_pin(ppins, config.get('gate_pin', None))
        self.entry = self._lookup_pin(ppins, config.get('entry_pin', None))
        self.exit = self._lookup_pin(ppins, config.get('exit_pin', None))
        if (self.entry is None) != (self.exit is None):
            raise config.error("buffer_feed: entry_pin and exit_pin must"
                               " be configured together")
        for pp in (self.trigger, self.stop, self.gate, self.entry, self.exit):
            if pp is not None and pp['chip'] is not self.mcu:
                raise config.error("buffer_feed: sensors and stepper must be"
                                   " on the same mcu")
        # Feed parameters
        self.distance = config.getfloat('distance', above=0.)
        self.velocity = config.getfloat('velocity', above=0.)
        self.accel = config.getfloat('accel', 0., minval=0.)
        self.decel = config.getfloat('decel', self.accel, minval=0.)
        self.start_velocity = config.getfloat('start_velocity', 2.,
                                              above=0.)
        # Load parameters
        if self.entry is not None:
            self.load_velocity = config.getfloat('load_velocity',
                                                 self.velocity, above=0.)
            self.load_accel = config.getfloat('load_accel', self.accel,
                                              minval=0.)
            self.load_decel = config.getfloat('load_decel', self.load_accel,
                                              minval=0.)
            self.load_start_velocity = config.getfloat(
                'load_start_velocity', self.start_velocity, above=0.)
            self.load_timeout = config.getfloat('load_timeout', 10.,
                                                above=0., maxval=60.)
            self.load_clear_distance = config.getfloat(
                'load_clear_distance', 0.)
            self.entry_debounce = config.getfloat('entry_debounce', .002,
                                                  minval=0., maxval=.100)
            self.exit_samples = config.getint('exit_samples', 2,
                                              minval=1, maxval=255)
        # Sensor handling
        self.poll_interval = config.getfloat('poll_interval', .0005,
                                             minval=.0001, maxval=.010)
        self.trigger_debounce = config.getfloat('trigger_debounce', .002,
                                                minval=0., maxval=.100)
        self.stop_samples = config.getint('stop_samples', 2,
                                          minval=1, maxval=255)
        self.max_runs = config.getint('max_runs', 3, minval=0, maxval=255)
        self.fill_runs = config.getint('fill_runs', 30, minval=0, maxval=255)
        self.start_enabled = config.getboolean('enable', True)
        self.enable_stepper = config.getboolean('enable_stepper', True)
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.fault_gcode = gcode_macro.load_template(config, 'fault_gcode', '')
        self.load_fault_gcode = gcode_macro.load_template(
            config, 'load_fault_gcode', '')
        # State
        self.oid = self.mcu.create_oid()
        self.ready = False
        self.enabled = False
        self.last_reason = 0
        self.last_steps = 0
        self.fault_count = 0
        self.load_fault_count = 0
        self.move_pending = False   # a move started by the host is running
        self.move_tag = 0           # tag of that move (events carry it)
        self.next_tag = 1
        self.move_result = 0
        self.set_profile_cmd = self.set_load_profile_cmd = None
        self.start_cmd = self.load_cmd = self.enable_cmd = None
        self.abort_cmd = self.query_cmd = None
        self.mcu.register_config_callback(self._build_config)
        self.event_resp = self.mcu.register_serial_response(
            self._handle_event_thread,
            "buffer_feed_event oid=%c reason=%c done=%u tag=%c", self.oid)
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler("klippy:shutdown",
                                            self._handle_shutdown)
        self.printer.register_event_handler("stepper:set_dir_inverted",
                                            self._handle_dir_inverted)
        self.printer.register_event_handler("stepper_enable:motor_off",
                                            self._handle_motor_off)
        # Commands
        for cmd, func, desc in [
                ("SET_BUFFER_FEED", self.cmd_SET_BUFFER_FEED,
                 "Enable/disable a buffer feed and change its parameters"),
                ("BUFFER_FEED_MOVE", self.cmd_BUFFER_FEED_MOVE,
                 "Start a single feed move (optionally stopped by sensor)"),
                ("BUFFER_FEED_LOAD", self.cmd_BUFFER_FEED_LOAD,
                 "Start the load sequence (entry to exit sensor)"),
                ("BUFFER_FEED_WAIT", self.cmd_BUFFER_FEED_WAIT,
                 "Wait until a running buffer feed move has finished"),
                ("BUFFER_FEED_ABORT", self.cmd_BUFFER_FEED_ABORT,
                 "Abort a running buffer feed or load move"),
                ("QUERY_BUFFER_FEED", self.cmd_QUERY_BUFFER_FEED,
                 "Report the state of a buffer feed")]:
            self.gcode.register_mux_command(cmd, "BUFFER", self.name, func,
                                            desc=desc)
    @staticmethod
    def _bare_pin(desc):
        # Remove the leading "^", "~" and "!" modifiers ("^!station:PA5")
        desc = desc.strip()
        while desc and desc[0] in '^~!':
            desc = desc[1:].lstrip()
        return desc
    def _lookup_pin(self, ppins, desc):
        if desc is None:
            return None
        ppins.allow_multi_use_pin(self._bare_pin(desc))
        return ppins.lookup_pin(desc, can_invert=True, can_pullup=True)
    def _lookup_stepper(self, config, section):
        obj = self.printer.load_object(config, section)
        # manual_stepper
        steppers = getattr(obj, 'steppers', None)
        if steppers:
            return steppers[0]
        # extruder_stepper (and extruder)
        es = getattr(obj, 'extruder_stepper', None)
        if es is not None:
            if hasattr(es, 'extruder_stepper'):
                es = es.extruder_stepper
            if hasattr(es, 'stepper'):
                return es.stepper
        raise config.error("buffer_feed: unable to find a stepper in '%s'"
                           % (section,))
    def _build_config(self):
        pulse, both_edge = self.mcu_stepper.get_pulse_duration()
        if not both_edge:
            raise self.printer.config_error(
                "buffer_feed '%s': stepper '%s' must use step on both edges"
                " (TMC drivers do by default; do not set a large"
                " step_pulse_duration)" % (
                    self.name, self.mcu_stepper.get_name()))
        sec = self.mcu.seconds_to_clock
        if self.entry is not None and (
                self.load_timeout * sec(1.) >= (1 << 32)):
            raise self.printer.config_error(
                "buffer_feed '%s': load_timeout too long for this mcu"
                % (self.name,))
        tp = self.trigger
        self.mcu.add_config_cmd(
            "config_buffer_feed oid=%d stepper_oid=%d trigger_pin=%s"
            " trigger_pull_up=%d trigger_active=%d poll_ticks=%d"
            " trigger_debounce=%d max_runs=%d fill_runs=%d" % (
                self.oid, self.mcu_stepper.get_oid(), tp['pin'], tp['pullup'],
                0 if tp['invert'] else 1, sec(self.poll_interval),
                self._debounce_count(self.trigger_debounce), self.max_runs,
                self.fill_runs))
        sp = self.stop
        if sp is not None:
            self.mcu.add_config_cmd(
                "config_buffer_feed_stop oid=%d stop_pin=%s stop_pull_up=%d"
                " stop_active=%d stop_samples=%d" % (
                    self.oid, sp['pin'], sp['pullup'],
                    0 if sp['invert'] else 1, self.stop_samples))
        gp = self.gate
        if gp is not None:
            self.mcu.add_config_cmd(
                "config_buffer_feed_gate oid=%d gate_pin=%s gate_pull_up=%d"
                " gate_active=%d" % (
                    self.oid, gp['pin'], gp['pullup'],
                    0 if gp['invert'] else 1))
        if self.entry is not None:
            ep, xp = self.entry, self.exit
            self.mcu.add_config_cmd(
                "config_buffer_feed_load oid=%d entry_pin=%s entry_pull_up=%d"
                " entry_active=%d entry_debounce=%d exit_pin=%s"
                " exit_pull_up=%d exit_active=%d exit_samples=%d" % (
                    self.oid, ep['pin'], ep['pullup'],
                    0 if ep['invert'] else 1,
                    self._debounce_count(self.entry_debounce),
                    xp['pin'], xp['pullup'], 0 if xp['invert'] else 1,
                    self.exit_samples))
        lookup = self.mcu.lookup_command
        self.set_profile_cmd = lookup(
            "buffer_feed_set_profile oid=%c steps=%u start_interval=%u"
            " cruise_interval=%u accel_add=%u decel_add=%u dir=%c")
        self.start_cmd = lookup(
            "buffer_feed_start oid=%c steps=%u start_interval=%u"
            " cruise_interval=%u accel_add=%u decel_add=%u stop_sel=%c"
            " stop_level=%c reverse=%c tag=%c")
        if self.entry is not None:
            self.set_load_profile_cmd = lookup(
                "buffer_feed_set_load_profile oid=%c start_interval=%u"
                " cruise_interval=%u accel_add=%u decel_add=%u dir=%c"
                " clear_steps=%u clear_back=%c timeout_ticks=%u")
            self.load_cmd = lookup("buffer_feed_load oid=%c tag=%c")
        self.enable_cmd = lookup("buffer_feed_enable oid=%c enable=%c")
        self.abort_cmd = lookup("buffer_feed_abort oid=%c")
        self.query_cmd = self.mcu.lookup_query_command(
            "buffer_feed_query oid=%c",
            "buffer_feed_state oid=%c state=%c reason=%c enabled=%c runs=%c"
            " trigger=%c stop=%c gate=%c entry=%c exit=%c done=%u"
            " remaining=%u total=%u", oid=self.oid)
    def _debounce_count(self, debounce):
        # Number of consecutive polls that must see the sensor active
        return max(1, int(round(debounce / self.poll_interval)))
    def _ramp_add(self, start, cruise, velocity, v0, accel):
        # Per step interval change (1/256 ticks) of a linear interval ramp
        # that takes (velocity - v0) / accel seconds
        if accel <= 0. or v0 >= velocity:
            return 0
        freq = self.mcu.seconds_to_clock(1.)
        ramp_time = (velocity - v0) / accel
        ramp = 2. * ramp_time * freq / (start + cruise)
        if ramp < 2.:
            return 0
        return max(1, int((start - cruise) * (1 << FP_SHIFT) / ramp))
    def _calc_profile(self, distance, velocity, accel, decel, start_velocity):
        step_dist = self.mcu_stepper.get_step_dist()
        freq = self.mcu.seconds_to_clock(1.)
        steps = max(1, int(round(distance / step_dist)))
        cruise = int(freq / (velocity / step_dist))
        if cruise < self.mcu.seconds_to_clock(MIN_INTERVAL_S):
            raise self.printer.command_error(
                "buffer_feed '%s': velocity %.1f too high (max step rate"
                " %d/s)" % (self.name, velocity, int(1. / MIN_INTERVAL_S)))
        start = cruise
        add = dadd = 0
        v0 = min(start_velocity, velocity)
        if v0 < velocity and (accel > 0. or decel > 0.):
            start = int(freq / (v0 / step_dist))
            add = self._ramp_add(start, cruise, velocity, v0, accel)
            dadd = self._ramp_add(start, cruise, velocity, v0, decel)
            if not add and not dadd:
                start = cruise
        if start >= MAX_INTERVAL_TICKS:
            raise self.printer.command_error(
                "buffer_feed '%s': start_velocity too low" % (self.name,))
        # Level of the dir pin for "forward" (positive) motion
        feed_dir = 0 if self.mcu_stepper.get_dir_inverted()[0] else 1
        return steps, start, cruise, add, dadd, feed_dir
    def _calc_load(self, p):
        # Returns the arguments for buffer_feed_set_load_profile
        _, start, cruise, add, dadd, feed_dir = self._calc_profile(
            1., p['load_velocity'], p['load_accel'], p['load_decel'],
            p['load_start_velocity'])
        step_dist = self.mcu_stepper.get_step_dist()
        clear = p['load_clear_distance']
        clear_steps = int(round(abs(clear) / step_dist))
        ticks = int(p['load_timeout'] * self.mcu.seconds_to_clock(1.))
        if ticks >= (1 << 32):
            raise self.printer.command_error(
                "buffer_feed '%s': load_timeout too long for this mcu"
                % (self.name,))
        return [start, cruise, add, dadd, feed_dir, clear_steps,
                1 if clear < 0. else 0, ticks]
    def _params(self, **override):
        p = {'distance': self.distance, 'velocity': self.velocity,
             'accel': self.accel, 'decel': self.decel}
        if self.entry is not None:
            for name in ('load_velocity', 'load_accel', 'load_decel',
                         'load_start_velocity', 'load_timeout',
                         'load_clear_distance'):
                p[name] = getattr(self, name)
        p.update(override)
        return p
    def _send_profiles(self, p=None):
        if p is None:
            p = self._params()
        steps, start, cruise, add, dadd, feed_dir = self._calc_profile(
            p['distance'], p['velocity'], p['accel'], p['decel'],
            self.start_velocity)
        load_args = None
        if self.entry is not None:
            load_args = self._calc_load(p)
        self.set_profile_cmd.send([self.oid, steps, start, cruise, add, dadd,
                                   feed_dir])
        if load_args is not None:
            self.set_load_profile_cmd.send([self.oid] + load_args)
    def _enable_motor(self):
        if self.enable_stepper:
            se = self.printer.lookup_object('stepper_enable')
            se.set_motors_enable([self.mcu_stepper.get_name()], True)
    def _handle_ready(self):
        self.ready = True
        if self.mcu.is_fileoutput():
            return
        self._send_profiles()
        if self.start_enabled:
            self.reactor.register_callback(self._startup_enable)
    def _startup_enable(self, eventtime):
        try:
            self.gcode.run_script("SET_BUFFER_FEED BUFFER=%s ENABLE=1"
                                  % (self.name,))
        except Exception:
            logging.exception("buffer_feed %s: startup enable failed",
                              self.name)
    def _handle_shutdown(self):
        self.enabled = False
        self.move_pending = False
    def _handle_dir_inverted(self, mcu_stepper):
        if self.ready and mcu_stepper is self.mcu_stepper:
            self._send_profiles()
    def _handle_motor_off(self, *args):
        # Feeding with a disabled driver would only report false faults
        if self.enabled and not self.mcu.is_fileoutput():
            self.enabled = False
            self.enable_cmd.send([self.oid, 0])
            logging.info("buffer_feed %s disabled (motors off)", self.name)
    def _handle_event_thread(self, params):
        # Called from the serial thread - continue in the main thread
        self.reactor.register_async_callback(
            (lambda e, p=params: self._handle_event(p)))
    def _handle_event(self, params):
        reason = params['reason']
        self.last_reason = reason
        self.last_steps = params['done']
        mine = self.move_pending and params['tag'] == self.move_tag
        if mine:
            # Events of other runs (auto feed, an aborted older move) do not
            # end the move the host is waiting for
            self.move_pending = False
            self.move_result = reason
        logging.info("buffer_feed %s: %s after %d steps", self.name,
                     REASON_NAMES.get(reason, reason), params['done'])
        if reason == REASON_BUSY and mine:
            self.gcode.respond_raw(
                "!! buffer_feed %s: command refused, the MCU is busy with"
                " another move (use WAIT=1 or BUFFER_FEED_WAIT)" % (self.name,))
        if reason == REASON_FAULT:
            self.fault_count += 1
            self.enabled = False
            self.reactor.register_callback(self._fault_handler)
        elif reason == REASON_LOAD_TIMEOUT:
            self.load_fault_count += 1
            self.reactor.register_callback(self._load_fault_handler)
    def _run_template(self, template, what):
        script = template.render()
        if script:
            try:
                self.gcode.run_script(script + "\nM400")
            except Exception:
                logging.exception("buffer_feed %s failed", what)
    def _fault_handler(self, eventtime):
        self.gcode.respond_raw(
            "!! buffer_feed %s: trigger still active after %d feed runs"
            % (self.name, self.max_runs))
        self._run_template(self.fault_gcode, "fault_gcode")
    def _load_fault_handler(self, eventtime):
        self.gcode.respond_raw(
            "!! buffer_feed %s: loading failed - sensor 2 (exit_pin) did not"
            " trigger within %.1f s" % (self.name, self.load_timeout))
        self._run_template(self.load_fault_gcode, "load_fault_gcode")
    # Commands
    def cmd_SET_BUFFER_FEED(self, gcmd):
        p = self._params()
        p['distance'] = gcmd.get_float('DISTANCE', p['distance'], above=0.)
        p['velocity'] = gcmd.get_float('VELOCITY', p['velocity'], above=0.)
        p['accel'] = gcmd.get_float('ACCEL', p['accel'], minval=0.)
        p['decel'] = gcmd.get_float('DECEL', p['decel'], minval=0.)
        if self.entry is not None:
            p['load_velocity'] = gcmd.get_float(
                'LOAD_VELOCITY', p['load_velocity'], above=0.)
            p['load_accel'] = gcmd.get_float(
                'LOAD_ACCEL', p['load_accel'], minval=0.)
            p['load_decel'] = gcmd.get_float(
                'LOAD_DECEL', p['load_decel'], minval=0.)
            p['load_timeout'] = gcmd.get_float(
                'LOAD_TIMEOUT', p['load_timeout'], above=0., maxval=60.)
            p['load_clear_distance'] = gcmd.get_float(
                'LOAD_CLEAR_DISTANCE', p['load_clear_distance'])
        enable = gcmd.get_int('ENABLE', None, minval=0, maxval=1)
        # Validate before changing any state
        self._calc_profile(p['distance'], p['velocity'], p['accel'],
                           p['decel'], self.start_velocity)
        if self.entry is not None:
            self._calc_load(p)
        for name, value in p.items():
            setattr(self, name, value)
        if self.mcu.is_fileoutput():
            return
        self._send_profiles(p)
        if enable is not None:
            if enable:
                self._enable_motor()
            self.enable_cmd.send([self.oid, enable])
            self.enabled = bool(enable)
            if not enable:
                self.move_pending = False   # the MCU aborts a running move
    def _error(self, gcmd, msg):
        # Errors of macros started by a gcode_button only reach klippy.log -
        # show them in the console as well
        self.gcode.respond_raw("!! " + msg)
        return gcmd.error(msg)
    def _start_move(self, gcmd, cmd, args):
        if self.move_pending:
            raise self._error(gcmd,
                "buffer_feed '%s': the previous move is still running (use"
                " WAIT=1 on it or BUFFER_FEED_WAIT)" % (self.name,))
        self._enable_motor()
        tag = self.next_tag
        self.next_tag = tag % 255 + 1
        self.move_tag = tag
        self.move_result = 0
        self.move_pending = True
        cmd.send(args + [tag])
    def _wait_done(self, gcmd, timeout):
        # Wait until the MCU reports the end of the move started by the host
        if self.mcu.is_fileoutput():
            return
        waited = self.move_pending
        end = self.reactor.monotonic() + timeout
        while self.move_pending:
            now = self.reactor.monotonic()
            if now > end:
                self.abort_cmd.send([self.oid])
                self.move_pending = False
                raise self._error(gcmd,
                    "buffer_feed '%s': move did not finish within %.1f s,"
                    " aborted" % (self.name, timeout))
            self.reactor.pause(now + .02)
        reason = self.move_result
        if waited and reason in (REASON_ABORTED, REASON_BUSY,
                                 REASON_LOAD_TIMEOUT):
            raise self._error(gcmd, "buffer_feed '%s': move ended with '%s'"
                             % (self.name, REASON_NAMES[reason]))
    def _lookup_stop_sensor(self, gcmd, name):
        # Returns (mcu selector, name) for a sensor name used by STOP=
        names = {'NONE': 0, '0': 0, 'FULL': 1, '1': 1, 'BUFFER_FULL': 1,
                 'LOW': 2, 'BUFFER_LOW': 2, 'ENTRY': 3, 'SENSOR1': 3,
                 'EXIT': 4, 'SENSOR2': 4, 'GATE': 5, 'SENSOR3': 5}
        sel = names.get(name.strip().upper())
        if sel is None:
            raise gcmd.error(
                "Unknown STOP sensor '%s' (use none, full, low, entry,"
                " exit or gate)" % (name,))
        present = {0: True, 1: self.stop is not None, 2: True,
                   3: self.entry is not None, 4: self.exit is not None,
                   5: self.gate is not None}
        if not present[sel]:
            raise gcmd.error("buffer_feed '%s' has no pin configured for"
                             " STOP=%s" % (self.name, name))
        return sel
    def cmd_BUFFER_FEED_MOVE(self, gcmd):
        # DISTANCE < 0 moves backwards (unloading)
        distance = gcmd.get_float('DISTANCE', self.distance)
        if not distance:
            raise gcmd.error("DISTANCE must not be zero")
        reverse = 1 if distance < 0. else 0
        velocity = gcmd.get_float('VELOCITY', self.velocity, above=0.)
        accel = gcmd.get_float('ACCEL', self.accel, minval=0.)
        decel = gcmd.get_float('DECEL', accel if gcmd.get('ACCEL', None)
                               is not None else self.decel, minval=0.)
        # Sensor that ends the move.  Default: "buffer full" when moving
        # forward (if there is such a sensor), otherwise none.
        default = 'full' if (self.stop is not None and not reverse) else 'none'
        sel = self._lookup_stop_sensor(gcmd, gcmd.get('STOP', default))
        stop_on = gcmd.get('STOP_ON', 'TRIGGER').strip().upper()
        if stop_on not in ('TRIGGER', 'RELEASE'):
            raise gcmd.error("STOP_ON must be TRIGGER or RELEASE")
        level = 1 if stop_on == 'TRIGGER' else 0
        wait = gcmd.get_int('WAIT', 0, minval=0, maxval=1)
        timeout = gcmd.get_float('WAIT_TIMEOUT',
                                 2. * abs(distance) / velocity + 3., above=0.)
        steps, start, cruise, add, dadd, feed_dir = self._calc_profile(
            abs(distance), velocity, accel, decel, self.start_velocity)
        if self.mcu.is_fileoutput():
            return
        self._start_move(gcmd, self.start_cmd,
                         [self.oid, steps, start, cruise, add, dadd, sel,
                          level, reverse])
        if wait:
            self._wait_done(gcmd, timeout)
    def cmd_BUFFER_FEED_LOAD(self, gcmd):
        if self.entry is None:
            raise gcmd.error("buffer_feed '%s' has no entry_pin/exit_pin"
                             % (self.name,))
        wait = gcmd.get_int('WAIT', 0, minval=0, maxval=1)
        timeout = gcmd.get_float(
            'WAIT_TIMEOUT', self.load_timeout + 2. * abs(
                self.load_clear_distance) / self.load_velocity + 3., above=0.)
        if self.mcu.is_fileoutput():
            return
        self._start_move(gcmd, self.load_cmd, [self.oid])
        if wait:
            self._wait_done(gcmd, timeout)
    def cmd_BUFFER_FEED_WAIT(self, gcmd):
        timeout = gcmd.get_float('TIMEOUT', 60., above=0.)
        self._wait_done(gcmd, timeout)
    def cmd_BUFFER_FEED_ABORT(self, gcmd):
        if not self.mcu.is_fileoutput():
            self.abort_cmd.send([self.oid])
            self.move_pending = False
    def cmd_QUERY_BUFFER_FEED(self, gcmd):
        if self.mcu.is_fileoutput():
            return
        p = self.query_cmd.send([self.oid])
        self.enabled = bool(p['enabled'])
        def sens(v, present=True):
            if not present:
                return "n/a"
            return "ACTIVE" if v else "open"
        gcmd.respond_info(
            "buffer_feed %s: state=%s enabled=%d last_result=%s\n"
            "sensors: entry(1)=%s exit(2)=%s gate(3)=%s buffer_low=%s"
            " buffer_full=%s\nruns=%d fed_steps=%d (last run %d steps)" % (
                self.name, STATE_NAMES.get(p['state'], p['state']),
                p['enabled'], REASON_NAMES.get(p['reason'], p['reason']),
                sens(p['entry'], self.entry is not None),
                sens(p['exit'], self.exit is not None),
                sens(p['gate'], self.gate is not None),
                sens(p['trigger']), sens(p['stop'], self.stop is not None),
                p['runs'], p['total'], self.last_steps))
    def get_status(self, eventtime):
        return {'enabled': self.enabled,
                'last_result': REASON_NAMES.get(self.last_reason, 'unknown'),
                'last_steps': self.last_steps,
                'fault_count': self.fault_count,
                'load_fault_count': self.load_fault_count,
                'distance': self.distance, 'velocity': self.velocity,
                'accel': self.accel, 'decel': self.decel}

def load_config_prefix(config):
    return BufferFeed(config)
