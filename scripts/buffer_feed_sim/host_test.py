# Test of klippy/extras/buffer_feed.py against mocked Klipper objects
# (config, pins, buttons, mcu, gcode, reactor).  Run with:
#   python3 scripts/buffer_feed_sim/host_test.py
import sys, os, contextlib, re
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
sys.path.insert(0, os.path.join(ROOT, 'klippy'))
sys.path.insert(0, os.path.join(ROOT, 'klippy', 'extras'))
import buffer_feed as bfm

fails = 0
def check(cond, msg):
    global fails
    print(("  ok:   " if cond else "  FAIL: ") + msg)
    if not cond:
        fails += 1

class CmdError(Exception):
    pass

log = []   # (kind, data)

class Cmd:
    def __init__(self, fmt):
        self.name = fmt.split()[0]
    def send(self, args):
        log.append((self.name, list(args)))
class QueryCmd:
    def send(self, args):
        return {'state': 0, 'reason': 0, 'enabled': 3, 'runs': 0,
                'trigger': 0, 'stop': 0, 'gate': 1, 'entry': 1, 'exit': 0,
                'done': 0, 'remaining': 0, 'total': 0}
class MCU:
    def __init__(self):
        self.oids = 0; self.cfg = []; self.cbs = []; self.resp = {}
    def create_oid(self):
        self.oids += 1; return self.oids - 1
    def register_config_callback(self, cb): self.cbs.append(cb)
    def register_serial_response(self, cb, fmt, oid): self.resp[oid] = cb
    def add_config_cmd(self, c, is_init=False): self.cfg.append(c)
    def lookup_command(self, fmt, cq=None): return Cmd(fmt)
    def lookup_query_command(self, fmt, resp, oid=None): return QueryCmd()
    def seconds_to_clock(self, t): return int(t * 64000000)
    def is_fileoutput(self): return False
MCU0 = MCU()
class Stepper:
    def __init__(self, name, oid):
        self.name = name; self.oid = oid
    def get_mcu(self): return MCU0
    def get_pulse_duration(self): return .0000001, True
    def get_name(self): return self.name
    def get_oid(self): return self.oid
    def get_step_dist(self): return 22. / 200 / 16 / 5
    def get_dir_inverted(self): return (0, 0)
class ManualStepper:
    def __init__(self, name, oid): self.steppers = [Stepper(name, oid)]
class EnableLine:
    def __init__(self): self.on = False
    def is_motor_enabled(self): return self.on
class StepperEnable:
    def __init__(self): self.lines = {}
    def lookup_enable(self, name):
        return self.lines.setdefault(name, EnableLine())
    def set_motors_enable(self, names, enable):
        for n in names:
            self.lookup_enable(n).on = enable
            log.append(('motor_enable', [n, enable]))
class Pins:
    def allow_multi_use_pin(self, d): pass
    def lookup_pin(self, desc, can_invert=False, can_pullup=False):
        d = desc.strip(); pullup = invert = 0
        while d[0] in '^!':
            if d[0] == '^': pullup = 1
            else: invert = 1
            d = d[1:]
        chip, pin = d.split(':')
        return {'chip': MCU0, 'chip_name': chip, 'pin': pin,
                'invert': invert, 'pullup': pullup}
class Buttons:
    def __init__(self): self.cbs = {}
    def register_buttons(self, pins, cb): self.cbs[pins[0]] = cb
class Template:
    def __init__(self, script): self.script = script
    def create_template_context(self): return {}
    def render(self, context=None):
        p = (context or {}).get('params', {})
        return re.sub(r'\{params\.(\w+)\}', lambda m: str(p.get(m.group(1))),
                      self.script)
class GcodeMacro:
    def load_template(self, config, name, default=None):
        return Template(config.opts.get(name, default or ''))
class Gcode:
    def __init__(self): self.mux = {}; self.scripts = []; self.raw = []
    def register_mux_command(self, cmd, key, val, func, desc=None):
        self.mux[cmd] = func
    def respond_raw(self, m): self.raw.append(m)
    def run_script(self, s):
        self.scripts.append(s)
        for line in s.split('\n'):
            parts = line.split()
            if parts and parts[0] in self.mux:
                self.mux[parts[0]](GCmd(parts[1:]))
    def get_mutex(self): return contextlib.nullcontext()
class GCmd:
    def __init__(self, args):
        self.p = dict(a.split('=', 1) for a in args); self.info = []
    def get(self, n, default=None): return self.p.get(n, default)
    def get_float(self, n, default=None, **kw):
        return float(self.p[n]) if n in self.p else default
    def get_int(self, n, default=None, **kw):
        return int(self.p[n]) if n in self.p else default
    def error(self, m): return CmdError(m)
    def respond_info(self, m): self.info.append(m)
class Reactor:
    def __init__(self): self.t = 0.; self.q = []
    def monotonic(self): return self.t
    def register_callback(self, cb, waketime=0): self.q.append(cb)
    register_async_callback = register_callback
    def pause(self, t): self.t = t
    def run(self):
        while self.q:
            self.q.pop(0)(self.t)
class Printer:
    def __init__(self):
        self.objs = {'gcode': Gcode(), 'pins': Pins(), 'buttons': Buttons(),
                     'gcode_macro': GcodeMacro(),
                     'stepper_enable': StepperEnable(),
                     'manual_stepper prefeeder_t0':
                         ManualStepper('manual_stepper prefeeder_t0', 10),
                     'manual_stepper prefeeder_t1':
                         ManualStepper('manual_stepper prefeeder_t1', 11)}
        self.reactor = Reactor(); self.events = {}
    def lookup_object(self, n): return self.objs[n]
    def load_object(self, config, n): return self.objs[n]
    def get_reactor(self): return self.reactor
    def register_event_handler(self, e, cb): self.events[e] = cb
    def config_error(self, m): return Exception(m)
    def command_error(self, m): return CmdError(m)
class Config:
    def __init__(self, printer, opts): self.printer = printer; self.opts = opts
    def get_printer(self): return self.printer
    def get_name(self): return 'buffer_feed station'
    def error(self, m): return Exception(m)
    def get(self, n, default=Exception):
        if n in self.opts: return self.opts[n]
        if default is Exception: raise Exception("missing " + n)
        return default
    def getfloat(self, n, default=Exception, **kw):
        v = self.get(n, default); return float(v) if v is not None else v
    def getint(self, n, default=Exception, **kw):
        return int(self.get(n, default))
    def getboolean(self, n, default=Exception, **kw):
        v = self.get(n, default)
        return v if isinstance(v, bool) else v.lower() == 'true'

opts = {
    'buffer_low_pin': '^station:BUF0', 'buffer_high_pin': '^station:BUF2',
    'gate_pin': '^station:BUF1',
    'stepper_t0': 'manual_stepper prefeeder_t0',
    'entry_pin_t0': '^station:T0SW0', 'exit_pin_t0': '^station:T0SW1',
    'stepper_t1': 'manual_stepper prefeeder_t1',
    'entry_pin_t1': '^station:T1SW0', 'exit_pin_t1': '^station:T1SW1',
    'distance': '40', 'velocity': '30', 'accel': '2000',
    'retract_distance': '5', 'load_timeout': '45',
    'load_clear_distance': '-2',
    'runout_gcode': 'M118 runout {params.CHANNEL}',
    'gate_release_gcode': '_GATE_RELEASED CH={params.CHANNEL}',
    'fault_gcode': 'M118 fault {params.CHANNEL} {params.REASON}',
    'state_gcode': '_LED CH={params.CHANNEL} STATE={params.STATE}',
}
printer = Printer()
bf = bfm.BufferFeed(Config(printer, opts))
gcode = printer.objs['gcode']; buttons = printer.objs['buttons']
reactor = printer.reactor
def masks():
    m = {}
    for name, args in log:
        if name == 'buffer_feed_enable':
            m[args[0]] = args[1]
    return m
def press(pin, state):
    buttons.cbs[pin](0., state); reactor.run()
def run(line):
    parts = line.split()
    try:
        gcode.mux[parts[0]](GCmd(parts[1:]))
        return None
    except CmdError as e:
        return str(e)
def event(oid, reason, tag=0, done=0):
    MCU0.resp[oid]({'reason': reason, 'done': done, 'tag': tag})
    reactor.run()

print("== config")
for cb in MCU0.cbs:
    cb()
check(len(bf.channels) == 2, "two channels")
check(sum('config_buffer_feed oid' in c for c in MCU0.cfg) == 2,
      "one MCU object per channel")
check(any('trigger_pin=BUF0' in c for c in MCU0.cfg)
      and any('stop_pin=BUF2' in c for c in MCU0.cfg), "shared buffer pins")
check(any('entry_pin=T1SW0' in c and 'exit_pin=T1SW1' in c
          for c in MCU0.cfg), "channel t1 entry/exit")
check(any('poll_ticks=320000' in c and 'trigger_debounce=2' in c
          for c in MCU0.cfg), "5 ms poll, 10 ms debounce = 2 polls")
check(any('retract_max_runs=10' in c for c in MCU0.cfg),
      "retract_max_runs default 10")

print("== startup: t0 is loaded through (exit active)")
press('^station:T0SW1', 1); press('^station:T0SW0', 1)
press('^station:T1SW0', 1)
printer.events['klippy:ready']()
prof = [a for n, a in log if n == 'buffer_feed_set_profile']
check(len(prof) == 2 and prof[0][7] == round(5 / (22. / 200 / 16 / 5)),
      "profiles with retract steps sent (%s)" % (prof[0][7],))
reactor.run()
check(masks() == {0: 3, 1: 2}, "t0 feed+load, t1 load only (%s)" % masks())
st = bf.get_status(0)
check(st['active_channel'] == 't0', "active_channel t0")
check(st['channels']['t1']['entry'] and not st['channels']['t1']['exit'],
      "t1 sensor states in status")
check('_LED CH=t0 STATE=loaded' in gcode.scripts
      and '_LED CH=t1 STATE=preloaded' in gcode.scripts,
      "state_gcode: t0 loaded, t1 preloaded")

print("== FEED=1 on t1 is refused while t0 is loaded through")
err = run("SET_BUFFER_FEED BUFFER=station CHANNEL=t1 FEED=1")
check(err is not None and 'unload it first' in err, "refused: %s" % err)
check(masks() == {0: 3, 1: 2}, "masks unchanged")

print("== runout on t0, gate releases, macro switches to t1")
press('^station:T0SW0', 0)
event(0, bfm.REASON_RUNOUT)
check('M118 runout t0' in gcode.scripts, "runout_gcode with CHANNEL=t0")
check(bf.get_status(0)['channels']['t0']['runout'], "runout in status")
press('^station:T0SW1', 0)
press('^station:BUF1', 1); press('^station:BUF1', 0)
check('_GATE_RELEASED CH=t0' in gcode.scripts, "gate_release_gcode run")
check(run("SET_BUFFER_FEED BUFFER=station CHANNEL=t0 FEED=0") is None,
      "FEED=0 t0")
log.append(('motor_enable_mark', []))
check(run("SET_BUFFER_FEED BUFFER=station CHANNEL=t1 FEED=1") is None,
      "FEED=1 t1")
check(masks() == {0: 2, 1: 3}, "t1 feeds now (%s)" % masks())
check(run("BUFFER_FEED_MOVE BUFFER=station CHANNEL=t1 DISTANCE=500"
          " STOP=high") is None, "switch move")
mv = [a for n, a in log if n == 'buffer_feed_start'][-1]
check(mv[0] == 1 and mv[6] == 1 and mv[8] == 0, "move on t1, stop high,"
      " forward")
check(bf.get_status(0)['active_channel'] == 't1', "active_channel t1")
event(1, 2, tag=mv[9])
check(not bf.channels[1].move_pending, "move finished")

print("== motors off -> paused, buffer_low wakes")
n0 = len(log)
printer.events['stepper_enable:motor_off']()
check(masks() == {0: 0, 1: 0} and bf.sleeping, "everything paused")
for l in printer.objs['stepper_enable'].lines.values():
    l.on = False
press('^station:BUF0', 1)
check(not bf.sleeping and masks() == {0: 2, 1: 3}, "resumed (%s)" % masks())
check(('motor_enable', ['manual_stepper prefeeder_t1', True]) in log[n0:],
      "t1 driver enabled again")
press('^station:BUF0', 0)

print("== motors off -> filament inserted at t0 starts a load")
printer.events['stepper_enable:motor_off']()
n0 = len(log)
press('^station:T0SW0', 1)
seq = [n for n, a in log[n0:] if n in ('buffer_feed_load',
                                        'buffer_feed_enable')]
check(seq[:1] == ['buffer_feed_load'] and not bf.sleeping,
      "load sent before re-enabling (%s)" % seq)
ld = [a for n, a in log[n0:] if n == 'buffer_feed_load'][0]
check(ld == [0, 0], "load on channel t0 (%s)" % ld)

print("== fault: feeding off, loading stays on")
event(1, bfm.REASON_RETRACT_FAULT)
check(masks()[1] == 2 and bf.get_status(0)['active_channel'] == '',
      "t1 load only (%s)" % masks())
check(any('buffer_high still active' in r for r in gcode.raw),
      "retract fault message")
check(any(s.startswith('M118 fault t1 retract_fault') for s in gcode.scripts),
      "fault_gcode with CHANNEL and REASON")

print("== unload sequence (non blocking) and abort")
check(bf.get_status(0)['channels']['t1']['state'] == 'error',
      "t1 state error after the fault")
check('_LED CH=t1 STATE=error' in gcode.scripts, "state_gcode error")
press('^station:T1SW1', 1)
check(run("SET_BUFFER_FEED BUFFER=station CHANNEL=t1 AUTO_LOAD=1") is None,
      "t1 auto load on")
n0 = len(log)
check(run("BUFFER_FEED_UNLOAD BUFFER=station CHANNEL=t1") is None,
      "BUFFER_FEED_UNLOAD returns at once")
mv = [a for n, a in log[n0:] if n == 'buffer_feed_start']
check(masks()[1] == 0 and len(mv) == 1 and mv[0][6] == 4 and mv[0][7] == 0
      and mv[0][8] == 1, "feeding/loading off, back until exit releases")
check(bf.get_status(0)['channels']['t1']['unloading'], "unloading in status")
event(1, 2, tag=mv[0][9])
mv2 = [a for n, a in log[n0:] if n == 'buffer_feed_start']
check(len(mv2) == 2 and mv2[1][6] == 3 and mv2[1][9] != mv[0][9],
      "second stage: back until entry releases")
check(bf.channels[1].move_pending, "still pending between the stages")
press('^station:T1SW1', 0); press('^station:T1SW0', 0)
event(1, 2, tag=mv2[1][9])
check(not bf.channels[1].unload_stage and masks()[1] == 2,
      "unload done, auto load restored (%s)" % masks())
check(bf.get_status(0)['channels']['t1']['state'] == 'empty',
      "t1 state empty")
check('_LED CH=t1 STATE=empty' in gcode.scripts, "state_gcode empty")
press('^station:T1SW0', 1)
press('^station:T1SW1', 1)
check(run("BUFFER_FEED_UNLOAD BUFFER=station CHANNEL=t1") is None,
      "second unload started")
check(run("BUFFER_FEED_ABORT BUFFER=station CHANNEL=t1") is None, "abort")
check(not bf.channels[1].unload_stage and masks()[1] == 2,
      "aborted: no second stage, auto load restored")
mv3 = [a for n, a in log if n == 'buffer_feed_start'][-1]
event(1, 3, tag=mv3[9])
check(not bf.channels[1].move_pending and not bf.channels[1].unload_stage,
      "late abort event ignored")
n0 = len(log)
check(run("BUFFER_FEED_UNLOAD BUFFER=station CHANNEL=t1") is None,
      "third unload started")
mv4 = [a for n, a in log[n0:] if n == 'buffer_feed_start'][0]
event(1, 1, tag=mv4[9])
check(any('exit sensor did not release within 2000 mm' in r
          for r in gcode.raw), "stage ran out of distance: error shown")

print("== commands need CHANNEL with two channels")
err = run("BUFFER_FEED_LOAD BUFFER=station")
check(err is not None and 'CHANNEL' in err, "BUFFER_FEED_LOAD: %s" % err)
check(run("BUFFER_FEED_MOVE BUFFER=station CHANNEL=t0 DISTANCE=-100"
          " STOP=entry STOP_ON=RELEASE") is None, "reverse move with STOP=entry")
mv = [a for n, a in log if n == 'buffer_feed_start'][-1]
check(mv[0] == 0 and mv[6] == 3 and mv[7] == 0 and mv[8] == 1,
      "t0, entry sensor, on release, reverse")
err = run("BUFFER_FEED_MOVE BUFFER=station CHANNEL=t0 STOP=full")
check(err is not None and 'Unknown STOP' in err, "old name 'full' rejected")
err = run("SET_BUFFER_FEED BUFFER=station ENABLE=0")
check(err is None and masks() == {0: 0, 1: 0}, "ENABLE=0 switches all off")
qi = GCmd([]); gcode.mux['QUERY_BUFFER_FEED'](qi)
check(qi.info and 't1: state=idle' in qi.info[0], "query output:\n" + qi.info[0])

print("\n%s (%d failures)" % ("ALL TESTS PASSED" if not fails else "FAILED",
                               fails))
sys.exit(1 if fails else 0)
