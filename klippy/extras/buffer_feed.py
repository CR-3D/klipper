# Autonomous filament buffer feeding and loading (runs on the MCU, see
# src/buffer_feed.c)
#
# A [buffer_feed] section describes one filament buffer with one or two
# channels (pre-feeders) t0 and t1.  Each channel gets its own buffer_feed
# object on the MCU, the buffer sensors are shared.
#
# Copyright (C) 2026  CR-3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

CHANNELS = ('t0', 't1')
REASON_NAMES = {0: "none", 1: "done", 2: "stop_sensor", 3: "aborted",
                4: "fault", 5: "busy", 6: "load_timeout", 7: "loaded",
                8: "runout", 9: "retract_fault"}
STATE_NAMES = {0: "idle", 1: "running", 2: "fault"}
REASON_ABORTED = 3
REASON_FAULT = 4
REASON_BUSY = 5
REASON_LOAD_TIMEOUT = 6
REASON_RUNOUT = 8
REASON_RETRACT_FAULT = 9
BE_FEED = 1                   # enable mask bits (see buffer_feed.c)
BE_LOAD = 2
BE_RUNOUT = 4                 # reported by buffer_feed_query only
MIN_INTERVAL_S = 40e-6        # maximum feed rate: 25 kHz
MAX_INTERVAL_TICKS = 1 << 23  # must match MAX_INTERVAL in buffer_feed.c
FP_SHIFT = 8
STARTUP_DELAY = 1.5           # time for the sensor states to arrive

# Sensor selectors for BUFFER_FEED_MOVE STOP= (must match buffer_feed.c)
STOP_SENSORS = {'NONE': 0, 'HIGH': 1, 'LOW': 2, 'ENTRY': 3, 'EXIT': 4,
                'GATE': 5}

class BufferChannel:
    def __init__(self, bf, config, name, stepper_section, entry, exit):
        self.bf = bf
        self.name = name
        self.mcu_stepper = bf.lookup_stepper(config, stepper_section)
        self.mcu = self.mcu_stepper.get_mcu()
        self.entry = entry
        self.exit = exit
        self.oid = self.mcu.create_oid()
        # Requested state (kept while the motors are off)
        self.feed = self.load = False
        self.runout = False
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
        self.mcu.register_serial_response(
            self._handle_event_thread,
            "buffer_feed_event oid=%c reason=%c done=%u tag=%c", self.oid)
    def get_name(self):
        return self.name
    def _build_config(self):
        bf = self.bf
        pulse, both_edge = self.mcu_stepper.get_pulse_duration()
        if not both_edge:
            raise bf.printer.config_error(
                "buffer_feed '%s': stepper '%s' must use step on both edges"
                " (TMC drivers do by default; do not set a large"
                " step_pulse_duration)" % (
                    bf.name, self.mcu_stepper.get_name()))
        sec = self.mcu.seconds_to_clock
        if self.entry is not None and (
                bf.load_timeout * sec(1.) >= (1 << 32)):
            raise bf.printer.config_error(
                "buffer_feed '%s': load_timeout too long for this mcu"
                % (bf.name,))
        lp = bf.low
        self.mcu.add_config_cmd(
            "config_buffer_feed oid=%d stepper_oid=%d trigger_pin=%s"
            " trigger_pull_up=%d trigger_active=%d poll_ticks=%d"
            " trigger_debounce=%d max_runs=%d" % (
                self.oid, self.mcu_stepper.get_oid(), lp['pin'], lp['pullup'],
                0 if lp['invert'] else 1, sec(bf.poll_interval),
                bf.debounce_count(bf.low_debounce), bf.max_runs))
        hp = bf.high
        if hp is not None:
            self.mcu.add_config_cmd(
                "config_buffer_feed_stop oid=%d stop_pin=%s stop_pull_up=%d"
                " stop_active=%d stop_samples=%d" % (
                    self.oid, hp['pin'], hp['pullup'],
                    0 if hp['invert'] else 1, bf.high_samples))
        gp = bf.gate
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
                    bf.debounce_count(bf.entry_debounce),
                    xp['pin'], xp['pullup'], 0 if xp['invert'] else 1,
                    bf.exit_samples))
        lookup = self.mcu.lookup_command
        self.set_profile_cmd = lookup(
            "buffer_feed_set_profile oid=%c steps=%u start_interval=%u"
            " cruise_interval=%u accel_add=%u decel_add=%u dir=%c"
            " retract_steps=%u")
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
    # Profiles
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
    def calc_profile(self, distance, velocity, accel, decel, start_velocity):
        bf = self.bf
        step_dist = self.mcu_stepper.get_step_dist()
        freq = self.mcu.seconds_to_clock(1.)
        steps = max(1, int(round(distance / step_dist)))
        cruise = int(freq / (velocity / step_dist))
        if cruise < self.mcu.seconds_to_clock(MIN_INTERVAL_S):
            raise bf.printer.command_error(
                "buffer_feed '%s': velocity %.1f too high (max step rate"
                " %d/s)" % (bf.name, velocity, int(1. / MIN_INTERVAL_S)))
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
            raise bf.printer.command_error(
                "buffer_feed '%s': start_velocity too low" % (bf.name,))
        # Level of the dir pin for "forward" (positive) motion
        feed_dir = 0 if self.mcu_stepper.get_dir_inverted()[0] else 1
        return steps, start, cruise, add, dadd, feed_dir
    def calc_load(self, p):
        # Returns the arguments for buffer_feed_set_load_profile
        bf = self.bf
        _, start, cruise, add, dadd, feed_dir = self.calc_profile(
            1., p['load_velocity'], p['load_accel'], p['load_decel'],
            p['load_start_velocity'])
        step_dist = self.mcu_stepper.get_step_dist()
        clear = p['load_clear_distance']
        clear_steps = int(round(abs(clear) / step_dist))
        ticks = int(p['load_timeout'] * self.mcu.seconds_to_clock(1.))
        if ticks >= (1 << 32):
            raise bf.printer.command_error(
                "buffer_feed '%s': load_timeout too long for this mcu"
                % (bf.name,))
        return [start, cruise, add, dadd, feed_dir, clear_steps,
                1 if clear < 0. else 0, ticks]
    def send_profiles(self, p):
        steps, start, cruise, add, dadd, feed_dir = self.calc_profile(
            p['distance'], p['velocity'], p['accel'], p['decel'],
            self.bf.start_velocity)
        step_dist = self.mcu_stepper.get_step_dist()
        retract_steps = int(round(p['retract_distance'] / step_dist))
        load_args = None
        if self.entry is not None:
            load_args = self.calc_load(p)
        self.set_profile_cmd.send([self.oid, steps, start, cruise, add, dadd,
                                   feed_dir, retract_steps])
        if load_args is not None:
            self.set_load_profile_cmd.send([self.oid] + load_args)
    # Enable state
    def get_mask(self):
        return ((BE_FEED if self.feed else 0)
                | (BE_LOAD if self.load and self.entry is not None else 0))
    def send_mask(self, mask=None):
        if mask is None:
            mask = self.get_mask()
        if mask:
            self.runout = False
        else:
            self.move_pending = False   # the MCU aborts a running move
        self.enable_cmd.send([self.oid, mask])
    def enable_motor(self):
        self.bf.enable_motor(self.mcu_stepper.get_name())
    # Events from the MCU
    def _handle_event_thread(self, params):
        # Called from the serial thread - continue in the main thread
        self.bf.reactor.register_async_callback(
            (lambda e, p=params: self._handle_event(p)))
    def _handle_event(self, params):
        bf = self.bf
        reason = params['reason']
        self.last_reason = reason
        self.last_steps = params['done']
        bf.last_reason = reason
        mine = self.move_pending and params['tag'] == self.move_tag
        if mine:
            # Events of other runs (auto feed, an aborted older move) do not
            # end the move the host is waiting for
            self.move_pending = False
            self.move_result = reason
        logging.info("buffer_feed %s %s: %s after %d steps", bf.name,
                     self.name, REASON_NAMES.get(reason, reason),
                     params['done'])
        if reason == REASON_BUSY and mine:
            bf.gcode.respond_raw(
                "!! buffer_feed %s %s: command refused, the MCU is busy with"
                " another move (use WAIT=1 or BUFFER_FEED_WAIT)"
                % (bf.name, self.name))
        if reason in (REASON_FAULT, REASON_RETRACT_FAULT):
            self.fault_count += 1
            # The MCU stopped - keep automatic loading, feeding stays off
            self.feed = False
            if not bf.sleeping:
                self.send_mask()
            if reason == REASON_FAULT:
                msg = ("buffer_low still active after %d feed runs"
                       % (bf.max_runs,))
            else:
                msg = ("buffer_high still active after %d retract runs"
                       % (bf.max_runs,))
            bf.run_event_template(bf.fault_gcode, self, msg,
                                  REASON_NAMES[reason])
        elif reason == REASON_LOAD_TIMEOUT:
            self.load_fault_count += 1
            bf.run_event_template(
                bf.load_fault_gcode, self,
                "loading failed - the exit sensor did not trigger within"
                " %.1f s" % (bf.load_timeout,), "load_timeout")
        elif reason == REASON_RUNOUT:
            self.runout = True
            bf.run_event_template(bf.runout_gcode, self, None, "runout")
    # Host moves
    def start_move(self, gcmd, cmd, args):
        bf = self.bf
        if self.move_pending:
            raise bf.error(gcmd,
                "buffer_feed '%s' %s: the previous move is still running"
                " (use WAIT=1 on it or BUFFER_FEED_WAIT)"
                % (bf.name, self.name))
        bf.wake()
        self.enable_motor()
        tag = self.next_tag
        self.next_tag = tag % 255 + 1
        self.move_tag = tag
        self.move_result = 0
        self.move_pending = True
        cmd.send(args + [tag])
    def wait_done(self, gcmd, timeout):
        # Wait until the MCU reports the end of the move started by the host
        bf = self.bf
        if self.mcu.is_fileoutput():
            return
        waited = self.move_pending
        end = bf.reactor.monotonic() + timeout
        while self.move_pending:
            now = bf.reactor.monotonic()
            if now > end:
                self.abort_cmd.send([self.oid])
                self.move_pending = False
                raise bf.error(gcmd,
                    "buffer_feed '%s' %s: move did not finish within %.1f s,"
                    " aborted" % (bf.name, self.name, timeout))
            bf.reactor.pause(now + .02)
        reason = self.move_result
        if waited and reason in (REASON_ABORTED, REASON_BUSY,
                                 REASON_LOAD_TIMEOUT):
            raise bf.error(gcmd, "buffer_feed '%s' %s: move ended with '%s'"
                           % (bf.name, self.name, REASON_NAMES[reason]))
    def abort(self):
        self.abort_cmd.send([self.oid])
        self.move_pending = False
    def get_status(self):
        bf = self.bf
        return {'entry': bf.sensors.get('entry_' + self.name, False),
                'exit': bf.sensors.get('exit_' + self.name, False),
                'feed': self.feed, 'auto_load': self.load,
                'runout': self.runout,
                'last_result': REASON_NAMES.get(self.last_reason, 'unknown'),
                'last_steps': self.last_steps,
                'fault_count': self.fault_count,
                'load_fault_count': self.load_fault_count}

class BufferFeed:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.name = config.get_name().split()[-1]
        # Buffer sensors (may also be used by other buffer_feed channels,
        # buttons, ...)
        self.ppins = self.printer.lookup_object('pins')
        self.low = self._lookup_pin(config.get('buffer_low_pin'))
        self.high = self._lookup_pin(config.get('buffer_high_pin', None))
        self.gate = self._lookup_pin(config.get('gate_pin', None))
        # Feed parameters
        self.distance = config.getfloat('distance', above=0.)
        self.velocity = config.getfloat('velocity', above=0.)
        self.accel = config.getfloat('accel', 0., minval=0.)
        self.decel = config.getfloat('decel', self.accel, minval=0.)
        self.start_velocity = config.getfloat('start_velocity', 2.,
                                              above=0.)
        self.retract_distance = config.getfloat('retract_distance', 0.,
                                                minval=0.)
        if self.retract_distance and self.high is None:
            raise config.error("buffer_feed '%s': retract_distance needs a"
                               " buffer_high_pin" % (self.name,))
        # Load parameters
        self.load_velocity = config.getfloat('load_velocity', self.velocity,
                                             above=0.)
        self.load_accel = config.getfloat('load_accel', self.accel, minval=0.)
        self.load_decel = config.getfloat('load_decel', self.load_accel,
                                          minval=0.)
        self.load_start_velocity = config.getfloat(
            'load_start_velocity', self.start_velocity, above=0.)
        self.load_timeout = config.getfloat('load_timeout', 10., above=0.,
                                            maxval=60.)
        self.load_clear_distance = config.getfloat('load_clear_distance', 0.)
        # Sensor handling
        self.poll_interval = config.getfloat('poll_interval', .005,
                                             minval=.0001, maxval=.010)
        self.low_debounce = config.getfloat('buffer_low_debounce', .010,
                                            minval=0., maxval=.100)
        self.high_samples = config.getint('buffer_high_samples', 2,
                                          minval=1, maxval=255)
        self.entry_debounce = config.getfloat('entry_debounce', .010,
                                              minval=0., maxval=.100)
        self.exit_samples = config.getint('exit_samples', 2,
                                          minval=1, maxval=255)
        self.max_runs = config.getint('max_runs', 3, minval=0, maxval=255)
        self.start_enabled = config.getboolean('enable', True)
        self.enable_stepper = config.getboolean('enable_stepper', True)
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.fault_gcode = gcode_macro.load_template(config, 'fault_gcode', '')
        self.load_fault_gcode = gcode_macro.load_template(
            config, 'load_fault_gcode', '')
        self.runout_gcode = gcode_macro.load_template(
            config, 'runout_gcode', '')
        self.gate_release_gcode = gcode_macro.load_template(
            config, 'gate_release_gcode', '')
        # Channels
        self.channels = []
        for cname in CHANNELS:
            section = config.get('stepper_' + cname, None)
            entry = self._lookup_pin(config.get('entry_pin_' + cname, None))
            exit = self._lookup_pin(config.get('exit_pin_' + cname, None))
            if section is None:
                if entry is not None or exit is not None:
                    raise config.error(
                        "buffer_feed '%s': entry_pin_%s/exit_pin_%s need"
                        " stepper_%s" % (self.name, cname, cname, cname))
                continue
            if (entry is None) != (exit is None):
                raise config.error(
                    "buffer_feed '%s': entry_pin_%s and exit_pin_%s must be"
                    " configured together" % (self.name, cname, cname))
            self.channels.append(
                BufferChannel(self, config, cname, section, entry, exit))
        if not self.channels:
            raise config.error("buffer_feed '%s': no channel configured (set"
                               " stepper_t0 and/or stepper_t1)" % (self.name,))
        mcu = self.channels[0].mcu
        pins = [self.low, self.high, self.gate]
        for ch in self.channels:
            if ch.mcu is not mcu:
                raise config.error("buffer_feed '%s': all steppers and"
                                   " sensors must be on the same mcu"
                                   % (self.name,))
            pins += [ch.entry, ch.exit]
        for pp in pins:
            if pp is not None and pp['chip'] is not mcu:
                raise config.error("buffer_feed '%s': all steppers and"
                                   " sensors must be on the same mcu"
                                   % (self.name,))
        self.mcu = mcu
        # Sensor states on the host (for the status, wake up and gate events)
        self.sensors = {}
        buttons = self.printer.load_object(config, 'buttons')
        watch = [('low', config.get('buffer_low_pin')),
                 ('high', config.get('buffer_high_pin', None)),
                 ('gate', config.get('gate_pin', None))]
        for ch in self.channels:
            watch += [('entry_' + ch.name,
                       config.get('entry_pin_' + ch.name, None)),
                      ('exit_' + ch.name,
                       config.get('exit_pin_' + ch.name, None))]
        for sname, desc in watch:
            if desc is None:
                continue
            self.sensors[sname] = False
            buttons.register_buttons(
                [desc], (lambda et, st, n=sname: self._sensor_event(n, st)))
        # State
        self.ready = False
        self.sleeping = False   # motors were switched off (M84, idle)
        self.last_reason = 0
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
                 "Enable/disable feeding and loading, change parameters"),
                ("BUFFER_FEED_MOVE", self.cmd_BUFFER_FEED_MOVE,
                 "Start a single move of a channel (optionally stopped by a"
                 " sensor)"),
                ("BUFFER_FEED_LOAD", self.cmd_BUFFER_FEED_LOAD,
                 "Start the load sequence of a channel (entry to exit"
                 " sensor)"),
                ("BUFFER_FEED_WAIT", self.cmd_BUFFER_FEED_WAIT,
                 "Wait until running buffer feed moves have finished"),
                ("BUFFER_FEED_ABORT", self.cmd_BUFFER_FEED_ABORT,
                 "Abort running buffer feed moves"),
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
    def _lookup_pin(self, desc):
        if desc is None:
            return None
        self.ppins.allow_multi_use_pin(self._bare_pin(desc))
        return self.ppins.lookup_pin(desc, can_invert=True, can_pullup=True)
    def lookup_stepper(self, config, section):
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
    def debounce_count(self, debounce):
        # Number of consecutive polls that must see the sensor active
        return max(1, int(round(debounce / self.poll_interval)))
    def _params(self):
        return {name: getattr(self, name) for name in (
            'distance', 'velocity', 'accel', 'decel', 'retract_distance',
            'load_velocity', 'load_accel', 'load_decel',
            'load_start_velocity', 'load_timeout', 'load_clear_distance')}
    def _lookup_channel(self, name):
        for ch in self.channels:
            if ch.name == name:
                return ch
        return None
    def _other(self, ch):
        for other in self.channels:
            if other is not ch:
                return other
        return None
    def get_active(self):
        for ch in self.channels:
            if ch.feed:
                return ch
        return None
    def enable_motor(self, stepper_name):
        if not self.enable_stepper:
            return
        se = self.printer.lookup_object('stepper_enable')
        # Only when needed - set_motors_enable() flushes the lookahead queue
        if not se.lookup_enable(stepper_name).is_motor_enabled():
            se.set_motors_enable([stepper_name], True)
    def error(self, gcmd, msg):
        # Errors of macros started by a gcode_button only reach klippy.log -
        # show them in the console as well
        self.gcode.respond_raw("!! " + msg)
        return gcmd.error(msg)
    def run_event_template(self, template, ch, msg, reason):
        if msg is not None:
            self.gcode.respond_raw("!! buffer_feed %s %s: %s"
                                   % (self.name, ch.name, msg))
        self.reactor.register_callback(
            (lambda e: self._run_template(template, ch, reason)))
    def _run_template(self, template, ch, reason):
        context = template.create_template_context()
        context['params'] = {'BUFFER': self.name, 'REASON': reason,
                             'CHANNEL': ch.name if ch is not None else ''}
        try:
            script = template.render(context)
            if script:
                self.gcode.run_script(script)
        except Exception:
            logging.exception("buffer_feed %s: %s template failed",
                              self.name, reason)
    # Startup and events
    def _handle_ready(self):
        self.ready = True
        if self.mcu.is_fileoutput():
            return
        p = self._params()
        for ch in self.channels:
            ch.send_profiles(p)
        if self.start_enabled:
            # Wait for the sensor states before selecting the channel
            self.reactor.register_callback(
                self._startup_enable, self.reactor.monotonic() + STARTUP_DELAY)
    def _startup_enable(self, eventtime):
        try:
            self.gcode.run_script("SET_BUFFER_FEED BUFFER=%s ENABLE=1"
                                  % (self.name,))
        except Exception:
            logging.exception("buffer_feed %s: startup enable failed",
                              self.name)
    def _handle_shutdown(self):
        self.ready = False
        for ch in self.channels:
            ch.move_pending = False
    def _handle_dir_inverted(self, mcu_stepper):
        if not self.ready:
            return
        for ch in self.channels:
            if mcu_stepper is ch.mcu_stepper:
                ch.send_profiles(self._params())
    def _handle_motor_off(self, *args):
        # Pause everything while the drivers are off, sensors wake it up
        if not self.ready or self.mcu.is_fileoutput() or self.sleeping:
            return
        if not any(ch.get_mask() for ch in self.channels):
            return
        self.sleeping = True
        for ch in self.channels:
            ch.send_mask(0)
        logging.info("buffer_feed %s: paused (motors off)", self.name)
    def wake(self, load_ch=None):
        # Resume after the motors were switched off
        if not self.sleeping:
            return
        self.sleeping = False
        for ch in self.channels:
            if ch.get_mask():
                ch.enable_motor()
        if load_ch is not None:
            # Filament inserted while paused - the MCU does not start a load
            # for filament that is already at the entry sensor
            load_ch.load_cmd.send([load_ch.oid, 0])
        for ch in self.channels:
            ch.send_mask()
        logging.info("buffer_feed %s: resumed", self.name)
    def _sensor_event(self, name, state):
        state = bool(state)
        self.sensors[name] = state
        if not self.ready or self.mcu.is_fileoutput():
            return
        if name.startswith('entry_') and state:
            ch = self._lookup_channel(name[6:])
            ch.runout = False
            if self.sleeping and ch.load:
                with self.gcode.get_mutex():
                    self.wake(load_ch=ch)
        elif name == 'low' and state:
            if self.sleeping and self.get_active() is not None:
                with self.gcode.get_mutex():
                    self.wake()
        elif name == 'gate' and not state:
            self._run_template(self.gate_release_gcode, self.get_active(),
                               "gate_release")
    def _auto_select(self):
        # Channel that feeds after ENABLE=1: the only one, or the one that
        # is loaded through to the buffer (exit sensor active)
        if len(self.channels) == 1:
            return self.channels[0]
        loaded = [ch for ch in self.channels
                  if ch.exit is not None and self.sensors['exit_' + ch.name]]
        if len(loaded) == 1:
            return loaded[0]
        if loaded:
            self.gcode.respond_raw(
                "!! buffer_feed %s: both channels are loaded through to the"
                " buffer (exit sensors), feeding stays off" % (self.name,))
        return None
    # Commands
    def _get_channel(self, gcmd, default_active=False):
        name = gcmd.get('CHANNEL', None)
        if name is None:
            if len(self.channels) == 1:
                return self.channels[0]
            if default_active and self.get_active() is not None:
                return self.get_active()
            raise gcmd.error("buffer_feed '%s' has two channels, CHANNEL="
                             " is required" % (self.name,))
        name = name.strip().lower()
        if name in ('0', '1'):
            name = 't' + name
        ch = self._lookup_channel(name)
        if ch is None:
            raise gcmd.error("buffer_feed '%s' has no channel '%s'"
                             % (self.name, name))
        return ch
    def cmd_SET_BUFFER_FEED(self, gcmd):
        p = self._params()
        p['distance'] = gcmd.get_float('DISTANCE', p['distance'], above=0.)
        p['velocity'] = gcmd.get_float('VELOCITY', p['velocity'], above=0.)
        p['accel'] = gcmd.get_float('ACCEL', p['accel'], minval=0.)
        p['decel'] = gcmd.get_float('DECEL', p['decel'], minval=0.)
        p['retract_distance'] = gcmd.get_float(
            'RETRACT_DISTANCE', p['retract_distance'], minval=0.)
        p['load_velocity'] = gcmd.get_float(
            'LOAD_VELOCITY', p['load_velocity'], above=0.)
        p['load_accel'] = gcmd.get_float('LOAD_ACCEL', p['load_accel'],
                                         minval=0.)
        p['load_decel'] = gcmd.get_float('LOAD_DECEL', p['load_decel'],
                                         minval=0.)
        p['load_timeout'] = gcmd.get_float(
            'LOAD_TIMEOUT', p['load_timeout'], above=0., maxval=60.)
        p['load_clear_distance'] = gcmd.get_float(
            'LOAD_CLEAR_DISTANCE', p['load_clear_distance'])
        if p['retract_distance'] and self.high is None:
            raise gcmd.error("buffer_feed '%s' has no buffer_high_pin"
                             % (self.name,))
        enable = gcmd.get_int('ENABLE', None, minval=0, maxval=1)
        feed = gcmd.get_int('FEED', None, minval=0, maxval=1)
        auto_load = gcmd.get_int('AUTO_LOAD', None, minval=0, maxval=1)
        ch = None
        if gcmd.get('CHANNEL', None) is not None or (
                (feed is not None or auto_load is not None)):
            ch = self._get_channel(gcmd)
        # Validate before changing any state
        for c in self.channels:
            c.calc_profile(p['distance'], p['velocity'], p['accel'],
                           p['decel'], self.start_velocity)
            if c.entry is not None:
                c.calc_load(p)
        if feed and ch is not None and len(self.channels) > 1:
            other = self._other(ch)
            if (other.exit is not None and self.sensors['exit_' + other.name]
                    and other.name != ch.name):
                raise self.error(gcmd,
                    "buffer_feed '%s': channel %s is loaded through to the"
                    " buffer (exit sensor), unload it first"
                    % (self.name, other.name))
        for name, value in p.items():
            setattr(self, name, value)
        # New requested state
        old = {c: c.get_mask() for c in self.channels}
        if enable is not None:
            if ch is not None:
                ch.feed = ch.load = bool(enable)
            elif not enable:
                for c in self.channels:
                    c.feed = c.load = False
            else:
                active = self._auto_select()
                for c in self.channels:
                    c.load = True
                    c.feed = c is active
        if ch is not None:
            if auto_load is not None:
                ch.load = bool(auto_load)
            if feed is not None:
                ch.feed = bool(feed)
        if ch is not None and ch.feed:
            # Only one channel feeds into the buffer
            for c in self.channels:
                if c is not ch:
                    c.feed = False
        if self.mcu.is_fileoutput():
            return
        for c in self.channels:
            c.send_profiles(p)
        if enable is None and feed is None and auto_load is None:
            return
        if self.sleeping:
            self.wake()
            return
        for c in self.channels:
            mask = c.get_mask()
            if mask and not old[c]:
                c.enable_motor()
            if mask != old[c]:
                c.send_mask(mask)
    def _lookup_stop_sensor(self, gcmd, ch, name):
        # Returns the mcu selector for a sensor name used by STOP=
        sel = STOP_SENSORS.get(name.strip().upper())
        if sel is None:
            raise gcmd.error(
                "Unknown STOP sensor '%s' (use none, high, low, entry, exit"
                " or gate)" % (name,))
        present = {0: True, 1: self.high is not None, 2: True,
                   3: ch.entry is not None, 4: ch.exit is not None,
                   5: self.gate is not None}
        if not present[sel]:
            raise gcmd.error("buffer_feed '%s' has no pin configured for"
                             " STOP=%s" % (self.name, name))
        return sel
    def cmd_BUFFER_FEED_MOVE(self, gcmd):
        ch = self._get_channel(gcmd, default_active=True)
        # DISTANCE < 0 moves backwards (unloading)
        distance = gcmd.get_float('DISTANCE', self.distance)
        if not distance:
            raise gcmd.error("DISTANCE must not be zero")
        reverse = 1 if distance < 0. else 0
        velocity = gcmd.get_float('VELOCITY', self.velocity, above=0.)
        accel = gcmd.get_float('ACCEL', self.accel, minval=0.)
        decel = gcmd.get_float('DECEL', accel if gcmd.get('ACCEL', None)
                               is not None else self.decel, minval=0.)
        # Sensor that ends the move.  Default: buffer_high when moving
        # forward (if there is such a sensor), otherwise none.
        default = 'high' if (self.high is not None and not reverse) else 'none'
        sel = self._lookup_stop_sensor(gcmd, ch, gcmd.get('STOP', default))
        stop_on = gcmd.get('STOP_ON', 'TRIGGER').strip().upper()
        if stop_on not in ('TRIGGER', 'RELEASE'):
            raise gcmd.error("STOP_ON must be TRIGGER or RELEASE")
        level = 1 if stop_on == 'TRIGGER' else 0
        wait = gcmd.get_int('WAIT', 0, minval=0, maxval=1)
        timeout = gcmd.get_float('WAIT_TIMEOUT',
                                 2. * abs(distance) / velocity + 3., above=0.)
        steps, start, cruise, add, dadd, feed_dir = ch.calc_profile(
            abs(distance), velocity, accel, decel, self.start_velocity)
        if self.mcu.is_fileoutput():
            return
        ch.start_move(gcmd, ch.start_cmd,
                      [ch.oid, steps, start, cruise, add, dadd, sel, level,
                       reverse])
        if wait:
            ch.wait_done(gcmd, timeout)
    def cmd_BUFFER_FEED_LOAD(self, gcmd):
        ch = self._get_channel(gcmd)
        if ch.entry is None:
            raise gcmd.error("buffer_feed '%s' channel %s has no"
                             " entry/exit pins" % (self.name, ch.name))
        wait = gcmd.get_int('WAIT', 0, minval=0, maxval=1)
        timeout = gcmd.get_float(
            'WAIT_TIMEOUT', self.load_timeout + 2. * abs(
                self.load_clear_distance) / self.load_velocity + 3., above=0.)
        if self.mcu.is_fileoutput():
            return
        ch.start_move(gcmd, ch.load_cmd, [ch.oid])
        if wait:
            ch.wait_done(gcmd, timeout)
    def cmd_BUFFER_FEED_WAIT(self, gcmd):
        timeout = gcmd.get_float('TIMEOUT', 60., above=0.)
        if gcmd.get('CHANNEL', None) is not None:
            channels = [self._get_channel(gcmd)]
        else:
            channels = self.channels
        for ch in channels:
            ch.wait_done(gcmd, timeout)
    def cmd_BUFFER_FEED_ABORT(self, gcmd):
        if gcmd.get('CHANNEL', None) is not None:
            channels = [self._get_channel(gcmd)]
        else:
            channels = self.channels
        if self.mcu.is_fileoutput():
            return
        for ch in channels:
            ch.abort()
    def cmd_QUERY_BUFFER_FEED(self, gcmd):
        if self.mcu.is_fileoutput():
            return
        def sens(v, present=True):
            if not present:
                return "n/a"
            return "ACTIVE" if v else "open"
        lines = []
        for ch in self.channels:
            p = ch.query_cmd.send([ch.oid])
            if not lines:
                lines.append(
                    "buffer_feed %s: active_channel=%s%s\nbuffer_low=%s"
                    " buffer_high=%s gate=%s" % (
                        self.name,
                        self.get_active().name if self.get_active() else "-",
                        " (paused, motors off)" if self.sleeping else "",
                        sens(p['trigger']),
                        sens(p['stop'], self.high is not None),
                        sens(p['gate'], self.gate is not None)))
            en = p['enabled']
            lines.append(
                "%s: state=%s feed=%d auto_load=%d runout=%d last_result=%s"
                " entry=%s exit=%s runs=%d fed_steps=%d (last run %d steps)"
                % (ch.name, STATE_NAMES.get(p['state'], p['state']),
                   bool(en & BE_FEED), bool(en & BE_LOAD),
                   bool(en & BE_RUNOUT),
                   REASON_NAMES.get(p['reason'], p['reason']),
                   sens(p['entry'], ch.entry is not None),
                   sens(p['exit'], ch.exit is not None),
                   p['runs'], p['total'], ch.last_steps))
        gcmd.respond_info("\n".join(lines))
    def get_status(self, eventtime):
        active = self.get_active()
        return {'active_channel': active.name if active is not None else '',
                'sleeping': self.sleeping,
                'buffer_low': self.sensors.get('low', False),
                'buffer_high': self.sensors.get('high', False),
                'gate': self.sensors.get('gate', False),
                'channels': {ch.name: ch.get_status()
                             for ch in self.channels},
                'last_result': REASON_NAMES.get(self.last_reason, 'unknown'),
                'distance': self.distance, 'velocity': self.velocity,
                'accel': self.accel, 'decel': self.decel,
                'retract_distance': self.retract_distance}

def load_config_prefix(config):
    return BufferFeed(config)
