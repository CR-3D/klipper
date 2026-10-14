# heater_faults: a fault during a print pauses it (also during a wait)
from common import *
import thermo
CFG = BASE + """
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
[virtual_sdcard]
path: @DIR@/gcodes
[pause_resume]
[heater_faults]
"""
h = Harness('print', CFG)
os.makedirs(h.path('gcodes'))
with open(h.path('gcodes/test.gcode'), 'w') as f:
    f.write('SET_HEATER_TEMPERATURE HEATER=hotend TARGET=200\n')
    f.write('TEMPERATURE_WAIT SENSOR="heater_generic hotend" MINIMUM=190\n')
    f.write('RESPOND MSG=after_wait\n')
    f.write('G4 P500\n' * 30)
h.ctl('main', 'adc PA0 %f' % thermo.adc(25))
try:
    h.start_klippy()
    h.wait_state('ready')
    h.gcode('SDCARD_PRINT_FILE FILENAME=test.gcode')
    h.pump(8)
    st = h.query(['print_stats', 'heater_generic hotend'])
    check(st['print_stats']['state'] == 'paused', "print paused")
    check(not any('after_wait' in r for r in h.responses),
          "paused before next line")
    h.gcode('RESET_HEATER_FAULT HEATER=hotend')
    h.gcode('RESUME')
    h.pump(3)
    check(h.query(['print_stats'])['print_stats']['state'] == 'printing',
          "resumed")
    check(any('after_wait' in r for r in h.responses), "continues")
    h.ctl('main', 'adc PA0 0.9999')
    h.pump(4)
    check(h.query(['print_stats'])['print_stats']['state'] == 'paused',
          "paused on sensor fault")
finally:
    h.stop()
done()
