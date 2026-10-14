# heater_faults: soft faults, reset, disable, hard limit, sensors
from common import *
import thermo
HEATER = """
[heater_generic hotend]
heater_pin: PA1
sensor_type: EPCOS 100K B57560G104F
sensor_pin: PA0
control: watermark
min_temp: 0
max_temp: 250
[verify_heater hotend]
check_gain_time: 3
[respond]
"""
FAULTS = """
[heater_faults]
fault_gcode:
  RESPOND MSG="FAULT_GCODE {params.HEATER} {params.REASON}"
"""
H = 'heater_generic hotend'
def temp(h):
    return h.query([H])[H]
def setup(name, cfg, adc=25.):
    h = Harness(name, cfg)
    if adc is None:
        h.ctl('main', 'adc PA0 0.9999')
    else:
        h.ctl('main', 'adc PA0 %f' % thermo.adc(adc))
    h.start_klippy()
    return h

# 1. stock behaviour: open thermistor -> shutdown
h = setup('heat_stock', BASE + HEATER, adc=None)
try:
    i = h.wait_state('shutdown')
    check('ADC out of range' in i['state_message'],
          "stock: open sensor shuts down")
finally:
    h.stop()

# 2. soft faults
h = setup('heat_open', BASE + HEATER + FAULTS, adc=None)
try:
    h.wait_state('ready')
    h.pump(3)
    st = temp(h)
    check(st['fault'] is not None and 'not in range' in st['fault'],
          "open sensor -> fault: %s" % st)
    check(h.info()['state'] == 'ready', "still ready")
    check(any('FAULT_GCODE hotend' in r for r in h.responses),
          "fault_gcode ran")
    r = h.gcode('SET_HEATER_TEMPERATURE HEATER=hotend TARGET=100')
    check('error' in r and 'RESET_HEATER_FAULT' in r['error']['message'],
          "set temp refused: %s" % r)
    r = h.gcode('RESET_HEATER_FAULT HEATER=hotend')
    check('error' in r and 'still not in range' in r['error']['message'],
          "reset refused while open: %s" % r)
    h.ctl('main', 'adc PA0 %f' % thermo.adc(25))
    h.pump(2)
    r = h.gcode('RESET_HEATER_FAULT HEATER=hotend')
    check('error' not in r and temp(h)['fault'] is None, "reset ok: %s" % r)
    # over temperature
    r = h.gcode('SET_HEATER_TEMPERATURE HEATER=hotend TARGET=60')
    h.pump(1)
    check('error' not in r and temp(h)['power'] > 0., "heating")
    h.ctl('main', 'adc PA0 %f' % thermo.adc(300))
    h.pump(3)
    st = temp(h)
    check(st['fault'] is not None and st['target'] == 0.
          and st['power'] == 0., "overtemp -> fault, off: %s" % st)
    check(h.info()['state'] == 'ready', "still ready after overtemp")
    h.ctl('main', 'adc PA0 %f' % thermo.adc(25))
    h.pump(2)
    h.gcode('RESET_HEATER_FAULT')
    # not heating at expected rate
    h.gcode('SET_HEATER_TEMPERATURE HEATER=hotend TARGET=200')
    h.pump(6)
    st = temp(h)
    check(st['fault'] is not None and 'expected rate' in st['fault'],
          "verify_heater -> fault: %s" % st)
    h.gcode('RESET_HEATER_FAULT')
    # fault while waiting for the temperature
    h.responses[:] = []
    h.gcode('SET_HEATER_TEMPERATURE HEATER=hotend TARGET=200')
    t0 = time.time()
    r = h.gcode('TEMPERATURE_WAIT SENSOR="heater_generic hotend"'
                ' MINIMUM=190', timeout=30)
    check(r is not None and 'error' not in r and time.time() - t0 < 15,
          "wait returns on fault (%.1fs)" % (time.time() - t0,))
    h.pump(1)
    check(any('FAULT_GCODE' in x for x in h.responses),
          "fault_gcode after wait")
    h.gcode('RESET_HEATER_FAULT')
    # disable (not stored)
    r = h.gcode('DISABLE_HEATER HEATER=hotend')
    check('error' not in r, "disable ok: %s" % r)
    check(not os.path.exists(h.path('heater_faults.json')), "no state file")
    r = h.gcode('SET_HEATER_TEMPERATURE HEATER=hotend TARGET=100')
    check('error' in r and 'disabled' in r['error']['message'],
          "disabled refuses temp")
    h.ctl('main', 'adc PA0 0.9999')
    h.pump(3)
    check(temp(h)['fault'] is None, "disabled: no fault on open sensor")
    h.gcode('RESTART')
    time.sleep(1)
    h.wait_state('ready')
    h.pump(3)
    st = temp(h)
    check(not st['disabled'] and st['fault'] is not None,
          "after restart: enabled, fault again: %s" % st)
    h.gcode('DISABLE_HEATER HEATER=hotend')
    check(temp(h)['fault'] is None, "disable clears fault")
    h.ctl('main', 'adc PA0 %f' % thermo.adc(25))
    h.pump(1)
    r = h.gcode('ENABLE_HEATER HEATER=hotend')
    check('error' not in r and not temp(h)['disabled'], "enable ok")
finally:
    h.stop()

# 3. hard limit
h = setup('heat_hard', BASE + HEATER + FAULTS + "hard_max_temp_margin: 20\n")
try:
    h.wait_state('ready')
    h.ctl('main', 'adc PA0 %f' % thermo.adc(260))
    h.pump(3)
    check(temp(h)['fault'] is not None and h.info()['state'] == 'ready',
          "260C: soft fault")
    h.ctl('main', 'adc PA0 %f' % thermo.adc(300))
    i = h.wait_state('shutdown')
    check('ADC out of range' in i['state_message'], "300C: hard shutdown")
finally:
    h.stop()

# 4. margin 0, open PT1000, temperature_sensor range
PT = """
[heater_generic pt]
heater_pin: PA3
sensor_type: PT1000
sensor_pin: PA2
control: watermark
min_temp: 0
max_temp: 300
[temperature_sensor chamber]
sensor_type: EPCOS 100K B57560G104F
sensor_pin: PA4
min_temp: 0
max_temp: 80
"""
h = Harness('heat_pt', BASE + HEATER + PT + FAULTS
            + "hard_max_temp_margin: 0\n")
h.ctl('main', 'adc PA0 %f' % thermo.adc(25))
h.ctl('main', 'adc PA2 0.9999')
h.ctl('main', 'adc PA4 %f' % thermo.adc(25))
try:
    h.start_klippy()
    h.wait_state('ready')
    h.pump(3)
    st = h.query(['heater_generic pt'])['heater_generic pt']
    check(st['fault'] is not None and h.info()['state'] == 'ready',
          "open PT1000: soft fault: %s" % st)
    h.ctl('main', 'adc PA0 %f' % thermo.adc(400))
    h.pump(3)
    check(h.info()['state'] == 'ready' and temp(h)['fault'] is not None,
          "margin 0: 400C soft")
    h.ctl('main', 'adc PA4 0.9999')
    h.pump(3)
    q = h.query(['heater_faults'])['heater_faults']
    check(h.info()['state'] == 'ready'
          and 'temperature_sensor chamber' in q['faults'],
          "sensor out of range: error, no shutdown")
    h.gcode('DISABLE_HEATER SENSOR="temperature_sensor chamber"')
    q = h.query(['heater_faults'])['heater_faults']
    check('temperature_sensor chamber' not in q['faults'],
          "disabled sensor: no fault")
    h.gcode('ENABLE_HEATER SENSOR="temperature_sensor chamber"')
    h.pump(3)
    q = h.query(['heater_faults'])['heater_faults']
    check('temperature_sensor chamber' in q['faults'], "fault again")
    h.ctl('main', 'adc PA4 %f' % thermo.adc(25))
    h.pump(2)
    q = h.query(['heater_faults'])['heater_faults']
    check('temperature_sensor chamber' not in q['faults']
          and any('back in range' in r for r in h.responses),
          "sensor back in range")
finally:
    h.stop()
done()
