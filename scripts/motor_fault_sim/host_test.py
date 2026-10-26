#!/usr/bin/env python3
# Host side test of the TMC open load (disconnected motor) detection.
# Uses the real TMC register definitions with a mocked printer and MCU.
#
#   python3 scripts/motor_fault_sim/host_test.py
import sys, os, logging, types, importlib
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "klippy"))
# The C helper and serial port are not used here - stub them if missing
for modname in ["cffi", "serial", "greenlet"]:
    try:
        importlib.import_module(modname)
    except ImportError:
        sys.modules[modname] = types.ModuleType(modname)
from extras import tmc, tmc2209, tmc2208, tmc2130, tmc2660, homing
import mcu

logging.basicConfig(level=logging.CRITICAL)

class CommandError(Exception):
    pass

class FakeTimer:
    def __init__(self, cb, waketime):
        self.cb, self.waketime = cb, waketime

class FakeReactor:
    NOW, NEVER = 0., 9999999999999999.
    def __init__(self):
        self.now = 100.
        self.callbacks = []
    def monotonic(self):
        return self.now
    def register_timer(self, cb, waketime):
        return FakeTimer(cb, waketime)
    def update_timer(self, timer, waketime):
        timer.waketime = waketime
    def unregister_timer(self, timer):
        timer.waketime = self.NEVER
    def register_callback(self, cb):
        self.callbacks.append(cb)
    def run_callbacks(self):
        cbs, self.callbacks = self.callbacks, []
        for cb in cbs:
            cb(self.now)

class FakeGCode:
    def __init__(self):
        self.commands = {}
        self.raw = []
        self.scripts = []
    def register_command(self, name, func, desc=None):
        self.commands[name] = func
    def respond_raw(self, msg):
        self.raw.append(msg)
    def run_script(self, script):
        self.scripts.append(script)

class FakeTemplate:
    def __init__(self, script):
        self.script = script
        self.contexts = []
    def create_template_context(self):
        return {'printer': None}
    def render(self, context):
        self.contexts.append(context)
        return self.script.replace("{stepper}", context['stepper'])

class FakeGCodeMacro:
    def load_template(self, config, option, default=None):
        return FakeTemplate(config.opts.get(option, default))

class FakePrinter:
    command_error = CommandError
    def __init__(self):
        self.reactor = FakeReactor()
        self.gcode = FakeGCode()
        self.objects = {'gcode': self.gcode,
                        'gcode_macro': FakeGCodeMacro()}
        self.handlers = {}
        self.shutdown_msg = None
    def get_reactor(self):
        return self.reactor
    def lookup_object(self, name, default=Exception):
        if name in self.objects:
            return self.objects[name]
        if default is Exception:
            raise Exception("no object %s" % (name,))
        return default
    def load_object(self, config, name):
        return self.objects[name]
    def add_object(self, name, obj):
        self.objects[name] = obj
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event, *params):
        return [cb(*params) for cb in self.handlers.get(event, [])]
    def invoke_shutdown(self, msg):
        self.shutdown_msg = msg

class ConfigError(Exception):
    pass

class FakeConfig:
    error = ConfigError
    def __init__(self, printer, name, opts):
        self.printer, self.name, self.opts = printer, name, opts
    def get_printer(self):
        return self.printer
    def get_name(self):
        return self.name
    def getchoice(self, option, choices, default):
        val = self.opts.get(option, default)
        if val not in choices:
            raise ConfigError("bad choice %s" % (val,))
        return val
    def getint(self, option, default, minval=None):
        return int(self.opts.get(option, default))
    def getfloat(self, option, default, minval=None):
        return float(self.opts.get(option, default))

class FakeMcuTmc:
    def __init__(self, fields, freq=12000000.):
        self.fields = fields
        self.regs = {}
        self.freq = freq
        self.reads = []
    def get_fields(self):
        return self.fields
    def get_register(self, name):
        self.reads.append(name)
        return self.regs.get(name, 0)
    def set_register(self, name, val, print_time=None):
        pass
    def get_tmc_frequency(self):
        return self.freq

class FakeStepper:
    def __init__(self, name, step_dist=0.0025):
        self.name, self.step_dist = name, step_dist
    def get_name(self):
        return self.name
    def get_step_dist(self):
        return self.step_dist

class FakeEndstop:
    def __init__(self, steppers, can_abort=True):
        self.steppers = steppers
        self.aborts = 0
        if can_abort:
            self.home_abort = self._home_abort
    def _home_abort(self):
        self.aborts += 1
    def get_steppers(self):
        return self.steppers

class ProbeWrapper:
    def __init__(self, mcu_endstop):
        self.mcu_endstop = mcu_endstop
        self.get_steppers = mcu_endstop.get_steppers

def FakeHomingMove(endstops):
    return homing.HomingMove(FakePrinter(), [(es, "z") for es in endstops],
                             toolhead=object())

def make_fields(mod):
    return tmc.FieldHelper(mod.Fields, getattr(mod, 'SignedFields', []),
                           mod.FieldFormatters)

def setup(opts=None, name="tmc2209 stepper_z", mod=tmc2209, printer=None):
    printer = printer or FakePrinter()
    fields = make_fields(mod)
    fields.set_field("mres", 4)
    mcu_tmc = FakeMcuTmc(fields)
    config = FakeConfig(printer, name, opts or {})
    ec = tmc.TMCErrorCheck(config, mcu_tmc)
    ec.set_stepper(FakeStepper(name.split()[-1]))
    ec.start_checks()
    return printer, mcu_tmc, ec

def drv(fields, ola=0, olb=0, stst=0, ot=0):
    val = 0
    for f, v in (("ola", ola), ("olb", olb), ("stst", stst), ("ot", ot)):
        if v:
            val = fields.set_field(f, 1, reg_value=val, reg_name="DRV_STATUS")
    return val

def poll(printer, mcu_tmc, ec, value, n=1):
    mcu_tmc.regs["DRV_STATUS"] = value
    res = None
    for i in range(n):
        printer.reactor.now += 1.
        res = ec._do_periodic_check(printer.reactor.now)
        if res == printer.reactor.NEVER:
            break
    printer.reactor.run_callbacks()
    return res

FAILS = []
def check(cond, desc):
    print("%s: %s" % ("ok  " if cond else "FAIL", desc))
    if not cond:
        FAILS.append(desc)

def errors(printer):
    return [m for m in printer.gcode.raw if m.startswith("!!")]

def t_disabled():
    p, m, ec = setup()
    f = m.fields
    poll(p, m, ec, drv(f, ola=1), n=10)
    check(not errors(p) and p.shutdown_msg is None,
          "disabled: no reaction to open load")
    check(ec.get_status()['open_load']['action'] == 'disabled',
          "disabled: status reports action")

def t_warn():
    p, m, ec = setup({'open_load_action': 'warn'})
    f = m.fields
    poll(p, m, ec, drv(f, ola=1), n=2)
    check(not errors(p), "warn: no report before open_load_count reached")
    poll(p, m, ec, drv(f, ola=1))
    check(len(errors(p)) == 1 and "stepper_z" in errors(p)[0]
          and "OpenLoad_A" in errors(p)[0], "warn: reported after 3 polls")
    check(ec.get_status()['open_load']['fault'], "warn: fault in status")
    poll(p, m, ec, drv(f, ola=1), n=5)
    check(len(errors(p)) == 1, "warn: reported only once")
    poll(p, m, ec, drv(f, ola=1, stst=1), n=3)
    check(ec.get_status()['open_load']['fault'],
          "warn: standstill does not clear fault")
    poll(p, m, ec, drv(f))
    check(not ec.get_status()['open_load']['fault'],
          "warn: good moving reading clears fault")
    poll(p, m, ec, drv(f, olb=1), n=3)
    check(len(errors(p)) == 2, "warn: re-armed after recovery")
    check(not p.gcode.scripts and p.shutdown_msg is None,
          "warn: no pause, no shutdown")

def t_standstill():
    p, m, ec = setup({'open_load_action': 'shutdown'})
    poll(p, m, ec, drv(m.fields, ola=1, olb=1, stst=1), n=10)
    check(p.shutdown_msg is None and not errors(p),
          "standstill: open load flags ignored")

def t_flicker():
    p, m, ec = setup({'open_load_action': 'shutdown'})
    f = m.fields
    for i in range(5):
        poll(p, m, ec, drv(f, ola=1), n=2)
        poll(p, m, ec, drv(f))
    check(p.shutdown_msg is None, "debounce: intermittent flags ignored")

def t_gcode():
    p, m, ec = setup({'open_load_action': 'gcode',
                      'open_load_gcode': 'MOTOR_ERROR STEPPER={stepper}'})
    poll(p, m, ec, drv(m.fields, olb=1), n=3)
    check(p.gcode.scripts == ["MOTOR_ERROR STEPPER=stepper_z"],
          "gcode: open_load_gcode executed")
    ctx = ec.ol_gcode.contexts[-1]
    check(ctx['stepper'] == "stepper_z" and ctx['flags'] == "olb",
          "gcode: stepper and flags in template context")
    check(len(errors(p)) == 1, "gcode: error message sent")
    poll(p, m, ec, drv(m.fields, olb=1), n=3)
    check(len(p.gcode.scripts) == 1, "gcode: executed only once per fault")
    poll(p, m, ec, drv(m.fields))
    poll(p, m, ec, drv(m.fields, ola=1, olb=1), n=3)
    check(len(p.gcode.scripts) == 2 and ec.ol_gcode.contexts[-1]['flags']
          == "ola olb", "gcode: executed again after recovery")
    p2, m2, ec2 = setup({'open_load_action': 'gcode'})
    poll(p2, m2, ec2, drv(m2.fields, ola=1), n=3)
    check(len(errors(p2)) == 1, "gcode: empty template only reports")

def t_shutdown():
    p, m, ec = setup({'open_load_action': 'shutdown'})
    res = poll(p, m, ec, drv(m.fields, ola=1), n=3)
    check(p.shutdown_msg is not None and "open load" in p.shutdown_msg,
          "shutdown: printer shut down")
    check(res == p.reactor.NEVER, "shutdown: periodic check stopped")

def t_homing_abort():
    p, m, ec = setup({'open_load_action': 'disabled',
                      'open_load_homing_action': 'abort'})
    f = m.fields
    other = FakeEndstop([FakeStepper("stepper_x")])
    hm_other = FakeHomingMove([other])
    p.send_event("homing:homing_move_begin", hm_other)
    check(ec.homing_move is None, "homing: other axis move not tracked")
    poll(p, m, ec, drv(f, ola=1), n=5)
    check(not errors(p) and other.aborts == 0,
          "homing: normal action used outside own homing move")
    p.send_event("homing:homing_move_end", hm_other)
    es_z = FakeEndstop([FakeStepper("stepper_z")])
    es_z1 = FakeEndstop([FakeStepper("stepper_z1")])
    hmove = FakeHomingMove([es_z, es_z1])
    p.send_event("homing:homing_move_begin", hmove)
    check(abs(ec.check_timer.waketime - p.reactor.now - 0.1) < 1e-9,
          "homing: check rescheduled to fast interval")
    res = poll(p, m, ec, drv(f, ola=1), n=2)
    check(abs(res - p.reactor.now - 0.1) < 1e-9,
          "homing: fast interval while homing")
    check(es_z.aborts == 0, "homing: debounced")
    poll(p, m, ec, drv(f, ola=1))
    check(es_z.aborts == 1 and es_z1.aborts == 1,
          "homing: all endstops of the move aborted")
    check(hmove.abort_message is not None
          and "stepper_z" in hmove.abort_message
          and "homing aborted" in hmove.abort_message,
          "homing: G28 fails with the open load message")
    check(not errors(p), "homing: no extra message (G28 error reports it)")
    poll(p, m, ec, drv(f, ola=1), n=3)
    check(es_z.aborts == 1, "homing: aborted only once")
    check(p.shutdown_msg is None, "homing: no shutdown on abort")
    p.send_event("homing:homing_move_end", hmove)
    check(ec.homing_move is None, "homing: move end tracked")
    res = poll(p, m, ec, drv(f))
    check(abs(res - p.reactor.now - 1.) < 1e-9,
          "homing: back to 1s interval")

def t_homing_default():
    p, m, ec = setup({})
    check(ec.ol_homing_action == 'abort', "homing: abort is the default")
    hmove = FakeHomingMove([FakeEndstop([FakeStepper("stepper_y")])])
    hmove.abort_message = "old"
    hmove.printer.send_event("homing:homing_move_begin", hmove)
    try:
        hmove.homing_move([0., 0., 0., 0.], 5.)
    except Exception:
        pass
    check(hmove.abort_message is None,
          "homing: abort message reset on new homing move")

def t_homing_probe_wrapper():
    p, m, ec = setup({'open_load_homing_action': 'abort'})
    inner = FakeEndstop([FakeStepper("stepper_z")])
    hmove = FakeHomingMove([ProbeWrapper(inner)])
    p.send_event("homing:homing_move_begin", hmove)
    poll(p, m, ec, drv(m.fields, ola=1), n=3)
    check(inner.aborts == 1 and p.shutdown_msg is None,
          "homing: probe wrapper aborted via inner endstop")

def t_homing_fallback():
    p, m, ec = setup({'open_load_homing_action': 'abort'})
    es = FakeEndstop([FakeStepper("stepper_z")], can_abort=False)
    p.send_event("homing:homing_move_begin", FakeHomingMove([es]))
    poll(p, m, ec, drv(m.fields, ola=1), n=3)
    check(p.shutdown_msg is not None,
          "homing: shutdown if endstop can not abort")

def t_homing_shutdown():
    p, m, ec = setup({'open_load_action': 'warn',
                      'open_load_homing_action': 'shutdown'})
    es = FakeEndstop([FakeStepper("stepper_z")])
    p.send_event("homing:homing_move_begin", FakeHomingMove([es]))
    poll(p, m, ec, drv(m.fields, ola=1), n=3)
    check(p.shutdown_msg is not None and es.aborts == 0,
          "homing: shutdown action")

def t_velocity():
    p, m, ec = setup({'open_load_action': 'warn',
                      'open_load_max_velocity': '50'})
    f = m.fields
    min_tstep = tmc.TMCtstepHelper(m, 50., pstepper=ec.stepper)
    m.regs["TSTEP"] = min_tstep // 2
    poll(p, m, ec, drv(f, ola=1), n=5)
    check(not errors(p) and "TSTEP" in m.reads,
          "velocity: ignored above max_velocity")
    m.regs["TSTEP"] = min_tstep * 2
    poll(p, m, ec, drv(f, ola=1), n=3)
    check(len(errors(p)) == 1, "velocity: reported below max_velocity")
    m.reads = []
    poll(p, m, ec, drv(f), n=2)
    check("TSTEP" not in m.reads, "velocity: TSTEP only read on open load")

class FakeGCmd:
    def __init__(self, params):
        self.params = params
        self.info = []
    def error(self, msg):
        return CommandError(msg)
    def get(self, name, default):
        return self.params.get(name, default)
    def get_int(self, name, default, minval=None):
        v = self.params.get(name)
        return default if v is None else int(v)
    def get_float(self, name, default, minval=None):
        v = self.params.get(name)
        return default if v is None else float(v)
    def respond_info(self, msg):
        self.info.append(msg)

def t_command():
    p = FakePrinter()
    p, mz, ecz = setup({'open_load_action': 'warn'}, printer=p)
    p, mz1, ecz1 = setup({}, name="tmc2130 stepper_z1", mod=tmc2130,
                         printer=p)
    cmd = p.lookup_object('tmc_open_load').cmd_SET_TMC_OPEN_LOAD
    poll(p, mz, ecz, drv(mz.fields, ola=1), n=3)
    gcmd = FakeGCmd({})
    cmd(gcmd)
    check("stepper_z: action=warn" in gcmd.info[0]
          and "stepper_z1: action=disabled" in gcmd.info[0]
          and "FAULT" in gcmd.info[0],
          "command: reports all steppers")
    check(ecz.ol_fault, "command: report does not clear fault")
    cmd(FakeGCmd({'STEPPER': 'stepper_z1', 'ACTION': 'GCODE',
                  'HOMING_ACTION': 'abort', 'COUNT': '5'}))
    st = ecz1.get_status()['open_load']
    check(st['action'] == 'gcode' and st['homing_action'] == 'abort'
          and st['count'] == 5 and ecz.ol_action == 'warn',
          "command: single stepper changed")
    cmd(FakeGCmd({'ACTION': 'disabled'}))
    check(ecz.ol_action == 'disabled' and ecz1.ol_action == 'disabled'
          and not ecz.ol_fault, "command: all steppers changed, fault cleared")
    for params, desc in [({'ACTION': 'abort'}, "invalid action"),
                         ({'ACTION': 'pause'}, "removed pause action"),
                         ({'HOMING_ACTION': 'pause'}, "invalid homing action"),
                         ({'STEPPER': 'stepper_q'}, "unknown stepper")]:
        try:
            cmd(FakeGCmd(params))
            check(False, "command: %s rejected" % (desc,))
        except CommandError:
            check(True, "command: %s rejected" % (desc,))
    # Switching mode while a fault persists (e.g. ignore during print)
    cmd(FakeGCmd({'STEPPER': 'stepper_z', 'ACTION': 'shutdown'}))
    poll(p, mz, ecz, drv(mz.fields, ola=1), n=3)
    check(p.shutdown_msg is not None, "command: new action takes effect")

def t_config():
    try:
        setup({'open_load_action': 'abort'})
        check(False, "config: invalid action rejected")
    except ConfigError:
        check(True, "config: invalid action rejected")
    p, m, ec = setup({'open_load_action': 'warn'}, name="tmc2660 stepper_y",
                     mod=tmc2660)
    check(ec.ol_mask != 0, "config: tmc2660 supports open load")
    for mod in (tmc2208, tmc2209, tmc2130):
        f = make_fields(mod)
        check(all(x in f.all_fields["DRV_STATUS"]
                  for x in ("ola", "olb", "stst")),
              "config: %s has ola/olb/stst" % (mod.__name__,))

def t_ot_still_errors():
    p, m, ec = setup({'open_load_action': 'warn'})
    poll(p, m, ec, drv(m.fields, ot=1), n=1)
    check(p.shutdown_msg is not None and "ot=1" in p.shutdown_msg,
          "existing over temperature check still active")

class FakeTrsync:
    def __init__(self):
        self.reasons = []
    def trigger(self, reason):
        self.reasons.append(reason)

def t_mcu_abort():
    disp = mcu.TriggerDispatch.__new__(mcu.TriggerDispatch)
    disp._trsyncs = [FakeTrsync(), FakeTrsync()]
    es = mcu.MCU_endstop.__new__(mcu.MCU_endstop)
    es._dispatch = disp
    es.home_abort()
    check(all(t.reasons == [mcu.MCU_trsync.REASON_HOST_REQUEST]
              for t in disp._trsyncs),
          "mcu: home_abort triggers all trsyncs with host request")

def main():
    for t in [t_disabled, t_warn, t_standstill, t_flicker, t_gcode,
              t_shutdown, t_homing_abort, t_homing_default,
              t_homing_probe_wrapper,
              t_homing_fallback, t_homing_shutdown, t_velocity, t_command,
              t_config, t_ot_still_errors, t_mcu_abort]:
        t()
    print("RESULT: %s" % ("FAILED (%d)" % len(FAILS) if FAILS else "OK"))
    sys.exit(1 if FAILS else 0)

if __name__ == '__main__':
    main()
