# Non-critical mcu: critical mcus still shut down, auto reconnect,
# FIRMWARE_RESTART without the mcu, canbus_stats style queries
from common import *
ST = """
[mcu station]
serial: /tmp/klipper_host_fake_station_%d
%s
[output_pin st_led]
pin: station:PA5
"""
# 1. critical secondary mcu: loss -> shutdown
h = Harness('crit', BASE + ST % (PID, ""), mcus=('main', 'station'))
try:
    h.start_klippy()
    h.wait_state('ready')
    h.ctl('station', 'unplug')
    i = h.wait_state('shutdown', timeout=15)
    check("Lost communication with MCU 'station'" in i['state_message'],
          "critical mcu loss -> shutdown")
finally:
    h.stop()
# 2. main mcu loss with a non-critical station -> shutdown
h = Harness('mainloss', BASE + ST % (PID, "is_non_critical: True"),
            mcus=('main', 'station'))
try:
    h.start_klippy()
    h.wait_state('ready')
    h.ctl('main', 'unplug')
    i = h.wait_state('shutdown', timeout=15)
    check("Lost communication with MCU 'mcu'" in i['state_message'],
          "main mcu loss -> shutdown")
finally:
    h.stop()
# 3. automatic reconnect and FIRMWARE_RESTART without the station
h = Harness('auto', BASE + ST % (PID, "is_non_critical: True\n"
                                 "reconnect_interval: 2"),
            mcus=('main', 'station'))
try:
    h.start_klippy()
    h.wait_state('ready')
    h.ctl('station', 'unplug')
    h.pump(8)
    st = h.query(['mcu station'])['mcu station']
    check(st['non_critical_disconnected'], "offline")
    h.ctl('station', 'plug')
    h.pump(12)
    st = h.query(['mcu station'])['mcu station']
    check(not st['non_critical_disconnected'], "auto reconnected")
    check(h.info()['state'] == 'ready', "ready")
    h.ctl('station', 'unplug')
    h.pump(8)
    h.request("gcode/firmware_restart")
    time.sleep(2)
    h.wait_state('ready', timeout=40)
    st = h.query(['mcu station'])['mcu station']
    check(st['non_critical_disconnected'],
          "firmware_restart without station -> ready, offline")
finally:
    h.stop()
# 4. the primary mcu can't be non-critical
h = Harness('mainnc', BASE.replace('[printer]',
                                   'is_non_critical: True\n[printer]'))
try:
    h.start_klippy()
    time.sleep(3)
    i = h.info()
    check('primary mcu can not be non-critical' in i['state_message'],
          "main non-critical rejected")
finally:
    h.stop()
# 5. dummy responses with enumerations (canbus_stats on an offline mcu)
sys.path.insert(0, KLIPPY)
import json, msgproto
h = Harness('dummy', BASE, mcus=('station',), start_mcus=())
with open(h.path('station.dict'), 'rb') as f:
    mp = msgproto.MessageParser()
    mp.process_identify(f.read(), decompress=False)
try:
    r = mp.create_dummy_response('canbus_status')
    check(r['canbus_bus_state'] == 'active', "dummy canbus_status: %s" % r)
except Exception as e:
    check(False, "dummy canbus_status: %s" % e)
h.stop()
done()
