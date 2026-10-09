# Non-fatal heater faults and disabling of heaters / temperature sensors
#
# With a [heater_faults] section a heater fault (sensor out of range,
# "not heating at expected rate") no longer shuts down the printer.
# Instead the heater is switched off and locked, fault_gcode is run
# (pauses a running print by default) and the fault can be cleared with
# RESET_HEATER_FAULT once the cause has been fixed.
#
# The range of temperature_sensor sections is checked on the host as well
# (an error is reported instead of a shutdown).
#
# Heaters and temperature sensors can also be disabled until the next
# restart (for example a defective tool head on an IDEX printer).
#
# Copyright (C) 2026  CR-3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

KELVIN_TO_CELSIUS = -273.15
NO_MAX_TEMP = 99999999.9
RANGE_CHECK_COUNT = 4

DEFAULT_FAULT_GCODE = """
{% if 'print_stats' in printer and printer.print_stats.state == 'printing' %}
  PAUSE
{% endif %}
"""

class HeaterFaults:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.reactor = printer.get_reactor()
        # Without a [heater_faults] section all heater errors shut down
        # the printer (stock Klipper behaviour)
        self.enabled = config.has_section('heater_faults')
        self.heaters = {}
        self.sensors = {}
        self.disabled = set()
        self.fault_template = None
        self.hard_max_margin = None
        self.is_ready = False
        self.pending_faults = []
        if not self.enabled:
            return
        gcode_macro = printer.load_object(config, 'gcode_macro')
        self.fault_template = gcode_macro.load_template(
            config, 'fault_gcode', DEFAULT_FAULT_GCODE)
        # A margin of 0 (or no margin) disables the hard mcu limit
        self.hard_max_margin = config.getfloat('hard_max_temp_margin', 0.,
                                               minval=0.)
        # Register commands
        gcode = printer.lookup_object('gcode')
        gcode.register_command("RESET_HEATER_FAULT",
                               self.cmd_RESET_HEATER_FAULT,
                               desc=self.cmd_RESET_HEATER_FAULT_help)
        gcode.register_command("DISABLE_HEATER", self.cmd_DISABLE_HEATER,
                               desc=self.cmd_DISABLE_HEATER_help)
        gcode.register_command("ENABLE_HEATER", self.cmd_ENABLE_HEATER,
                               desc=self.cmd_ENABLE_HEATER_help)
        gcode.register_command("QUERY_HEATER_FAULTS",
                               self.cmd_QUERY_HEATER_FAULTS,
                               desc=self.cmd_QUERY_HEATER_FAULTS_help)
        printer.register_event_handler("klippy:ready", self._handle_ready)
    def _handle_ready(self):
        self.is_ready = True
        pconfig = self.printer.lookup_object('configfile')
        # Faults found during startup (e.g. a disconnected thermistor)
        pending, self.pending_faults = self.pending_faults, []
        for heater, reason in pending:
            pconfig.runtime_warning("Heater %s fault: %s"
                                    % (heater.short_name, reason))
            self._run_fault_gcode(heater, reason)
    # Interface for heaters and temperature sensors
    def is_enabled(self):
        return self.enabled
    def is_disabled(self, name):
        return name in self.disabled
    def setup_heater(self, heater, sensor, min_temp, max_temp):
        # Returns True if the host must check the temperature range
        name = heater.get_name()
        self.heaters[name] = heater
        if not self.enabled:
            sensor.setup_minmax(min_temp, max_temp)
            return False
        # The range is checked on the host (see Heater.check_range()),
        # the mcu only checks the optional hard limit
        hard_max = NO_MAX_TEMP
        if self.hard_max_margin:
            hard_max = max_temp + self.hard_max_margin
        sensor.setup_minmax(KELVIN_TO_CELSIUS, hard_max)
        return True
    def setup_sensor(self, name, sensor, min_temp, max_temp):
        # Returns a SensorCheck if the host must check the range
        if not self.enabled:
            sensor.setup_minmax(min_temp, max_temp)
            return None
        sensor.setup_minmax(KELVIN_TO_CELSIUS, NO_MAX_TEMP)
        self.sensors[name] = sc = SensorCheck(name, min_temp, max_temp)
        return sc
    def check_sensor(self, sc, temp):
        # Host side range check of a temperature sensor (any thread)
        if sc.name in self.disabled or (
                temp >= sc.min_temp and temp <= sc.max_temp):
            sc.range_errors = 0
            if sc.fault is not None:
                sc.fault = None
                if sc.name not in self.disabled:
                    self._respond_async("%s temperature back in range"
                                        % (sc.name,))
            return
        sc.range_errors += 1
        if sc.range_errors < RANGE_CHECK_COUNT or sc.fault is not None:
            return
        sc.fault = "temperature %.1f not in range %.1f:%.1f" % (
            temp, sc.min_temp, sc.max_temp)
        logging.error("%s %s", sc.name, sc.fault)
        self._respond_async("!! %s %s" % (sc.name, sc.fault))
    def _respond_async(self, msg):
        gcode = self.printer.lookup_object('gcode')
        self.reactor.register_async_callback(
            (lambda e: gcode.respond_raw(msg)))
    def heater_fault(self, heater, reason):
        if heater.set_fault(reason):
            self.notify_fault(heater, reason)
    def notify_fault(self, heater, reason):
        # Report a fault already set on the heater (may be called from
        # any thread)
        self.reactor.register_async_callback(
            (lambda e: self._handle_fault(heater, reason)))
    def _handle_fault(self, heater, reason):
        name = heater.short_name
        msg = "Heater %s fault: %s" % (name, reason)
        logging.error(msg)
        gcode = self.printer.lookup_object('gcode')
        gcode.respond_raw("!! %s\n!! Heater is off. Fix the problem and run"
                          " RESET_HEATER_FAULT HEATER=%s" % (msg, name))
        self.printer.send_event("heater_faults:fault", heater, reason)
        if not self.is_ready:
            self.pending_faults.append((heater, reason))
            return
        self._run_fault_gcode(heater, reason)
    def _run_fault_gcode(self, heater, reason):
        if self.printer.is_shutdown():
            return
        gcode = self.printer.lookup_object('gcode')
        try:
            context = self.fault_template.create_template_context()
            context['params'] = {'HEATER': heater.short_name,
                                 'REASON': reason}
            # Waits until the current command (e.g. M109) has finished
            gcode.run_script(self.fault_template.render(context))
        except Exception:
            logging.exception("heater_faults: error in fault_gcode")
    # Command helpers
    def _lookup(self, gcmd):
        heater_name = gcmd.get('HEATER', None)
        sensor_name = gcmd.get('SENSOR', None)
        if (heater_name is None) == (sensor_name is None):
            raise gcmd.error("Specify either HEATER or SENSOR")
        if heater_name is not None:
            pheaters = self.printer.lookup_object('heaters')
            try:
                heater = pheaters.lookup_heater(heater_name)
            except self.printer.config_error as e:
                raise gcmd.error(str(e))
            return heater.get_name(), heater
        if sensor_name not in self.sensors:
            raise gcmd.error("Unknown temperature sensor '%s'"
                             % (sensor_name,))
        return sensor_name, None
    cmd_RESET_HEATER_FAULT_help = "Clear a heater fault"
    def cmd_RESET_HEATER_FAULT(self, gcmd):
        name = gcmd.get('HEATER', None)
        if name is None:
            heaters = [h for h in self.heaters.values()
                       if h.get_fault() is not None]
        else:
            name, heater = self._lookup(gcmd)
            heaters = [heater]
        for heater in heaters:
            if heater.get_fault() is None:
                gcmd.respond_info("Heater %s has no fault" % (
                    heater.short_name,))
                continue
            heater.clear_fault()
            gcmd.respond_info("Heater %s fault cleared" % (heater.short_name,))
    cmd_DISABLE_HEATER_help = "Disable a heater or temperature sensor"
    def cmd_DISABLE_HEATER(self, gcmd):
        name, heater = self._lookup(gcmd)
        if heater is not None:
            heater.set_disabled(True)
        self.disabled.add(name)
        gcmd.respond_info("%s disabled until ENABLE_HEATER or the next"
                          " restart" % (name,))
    cmd_ENABLE_HEATER_help = "Enable a disabled heater or temperature sensor"
    def cmd_ENABLE_HEATER(self, gcmd):
        name, heater = self._lookup(gcmd)
        if name not in self.disabled:
            raise gcmd.error("%s is not disabled" % (name,))
        self.disabled.discard(name)
        if heater is not None:
            heater.set_disabled(False)
        gcmd.respond_info("%s enabled" % (name,))
    cmd_QUERY_HEATER_FAULTS_help = "Report heater faults and disabled heaters"
    def cmd_QUERY_HEATER_FAULTS(self, gcmd):
        lines = []
        for name, heater in sorted(self.heaters.items()):
            fault = heater.get_fault()
            if fault is not None:
                lines.append("%s: fault: %s" % (name, fault))
        for name, sc in sorted(self.sensors.items()):
            if sc.fault is not None and name not in self.disabled:
                lines.append("%s: %s" % (name, sc.fault))
        for name in sorted(self.disabled):
            lines.append("%s: disabled" % (name,))
        gcmd.respond_info("\n".join(lines) or "No heater faults")
    def get_status(self, eventtime):
        faults = {}
        for name, heater in self.heaters.items():
            fault = heater.get_fault()
            if fault is not None:
                faults[name] = fault
        for name, sc in self.sensors.items():
            if sc.fault is not None and name not in self.disabled:
                faults[name] = sc.fault
        return {'enabled': self.enabled, 'faults': faults,
                'disabled': sorted(self.disabled)}

class SensorCheck:
    def __init__(self, name, min_temp, max_temp):
        self.name = name
        self.min_temp, self.max_temp = min_temp, max_temp
        self.range_errors = 0
        self.fault = None

def load_config(config):
    return HeaterFaults(config)
