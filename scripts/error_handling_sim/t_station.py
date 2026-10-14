from common import *
STATION = """
[mcu station]
serial: /tmp/klipper_host_fake_station_%d
is_non_critical: True
connect_timeout: 3
[manual_stepper st]
step_pin: station:PA1
dir_pin: station:PA2
enable_pin: !station:PA3
microsteps: 16
rotation_distance: 22
velocity: 50
accel: 500
[temperature_sensor st_temp]
sensor_type: EPCOS 100K B57560G104F
sensor_pin: station:PA0
[output_pin st_led]
pin: station:PA5
[temperature_sensor st_mcu]
sensor_type: temperature_mcu
sensor_mcu: station
[canbus_stats station]
[neopixel st_np]
pin: station:PA6
chain_count: 3
[respond]
[virtual_sdcard]
path: @DIR@/gcodes
[pause_resume]
""" % PID
CFG = BASE + STATION
def flog(h, m):
    return h.flog(m)
def st_status(h):
    return h.query(['mcu station'])['mcu station']

h = Harness('station', CFG, mcus=('main', 'station'))
try:
    h.start_klippy()
    h.wait_state('ready')
    check(os.path.exists(h.path('.mcu_station.dict')), "dictionary cached")
    st = st_status(h)
    check(st['non_critical_disconnected'] is False, "station connected: %s" % {k: st.get(k) for k in ('non_critical_disconnected', 'disconnect_reason')})
    r = h.gcode('MANUAL_STEPPER STEPPER=st ENABLE=1\nMANUAL_STEPPER STEPPER=st MOVE=10')
    check('error' not in r, "move ok: %s" % r)
    h.pump(1)
    check('queue_step' in flog(h, 'station'), "station got steps")
    # 1. lose the station during operation
    h.ctl('station', 'unplug')
    h.pump(8)
    i = h.info()
    check(i['state'] == 'ready', "still ready after unplug: %s" % i['state'])
    st = st_status(h)
    check(st['non_critical_disconnected'] is True, "station offline: %s" % st.get('disconnect_reason'))
    check(any("MCU 'station' disconnected" in x for x in h.responses), "disconnect message")
    r = h.gcode('MANUAL_STEPPER STEPPER=st MOVE=20\nSET_PIN PIN=st_led VALUE=1\nSET_LED LED=st_np RED=1\nG4 P200')
    check('error' not in r, "commands work while offline: %s" % r)
    r = h.gcode('MCU_RECONNECT MCU=station', timeout=30)
    check('error' in r, "reconnect fails while unplugged: %s" % r)
    check(st_status(h)['non_critical_disconnected'] is True, "still offline")
    r = h.gcode('MANUAL_STEPPER STEPPER=st MOVE=25')
    check('error' not in r, "move after failed reconnect: %s" % r)
    # 2. reconnect
    h.ctl('station', 'plug')
    time.sleep(0.5)
    r = h.gcode('MCU_RECONNECT MCU=station', timeout=30)
    check(r is not None and 'error' not in r, "reconnect ok: %s" % r)
    check(st_status(h)['non_critical_disconnected'] is False, "station online again")
    check('finalize_config' in flog(h, 'station').split('plugged')[-1], "config resent")
    r = h.gcode('MANUAL_STEPPER STEPPER=st ENABLE=1\nMANUAL_STEPPER STEPPER=st MOVE=0')
    check('error' not in r, "move after reconnect: %s" % r)
    h.pump(1)
    check('queue_step' in flog(h, 'station').split('plugged')[-1], "steps sent after reconnect")
    check('neopixel_update' in flog(h, 'station').split('plugged')[-1], "neopixel resent")
    check(h.info()['state'] == 'ready', "ready after reconnect")
    # 3. firmware shutdown of the station
    h.ctl('station', 'shutdown')
    h.pump(2)
    check(h.info()['state'] == 'ready', "station shutdown -> still ready")
    st = st_status(h)
    check(st['non_critical_disconnected'] is True and 'shutdown' in st['disconnect_reason'], "offline after shutdown: %s" % st.get('disconnect_reason'))
    r = h.gcode('MCU_RECONNECT MCU=station', timeout=40)
    check(r is not None and 'error' not in r, "reconnect after shutdown (reset): %s" % r)
    check('cmd reset' in flog(h, 'station'), "station was reset")
    r = h.gcode('MANUAL_STEPPER STEPPER=st MOVE=5')
    check('error' not in r and h.info()['state'] == 'ready', "move after shutdown recovery")
    # 4. disconnect during a print -> pause
    os.makedirs(h.path('gcodes'), exist_ok=True)
    with open(h.path('gcodes/p.gcode'), 'w') as f:
        for i in range(60):
            f.write('G4 P250\n')
    h.gcode('SDCARD_PRINT_FILE FILENAME=p.gcode')
    h.pump(1)
    h.ctl('station', 'unplug')
    h.pump(8)
    ps = h.query(['print_stats'])['print_stats']
    check(ps['state'] == 'paused', "print paused on disconnect: %s" % ps['state'])
    h.gcode('CANCEL_PRINT')
finally:
    h.stop()
print("\n".join(h.responses[-12:]))

# 5. start without the station (cache from the first run exists)
cache = open(h.path('.mcu_station.dict'), 'rb').read()
h2 = Harness('station_off', CFG, mcus=('main', 'station'), start_mcus=('main',))
try:
    with open(h2.path('.mcu_station.dict'), 'wb') as f:
        f.write(cache)
    h2.start_klippy()
    i = h2.wait_state('ready', timeout=30)
    check(i['state'] == 'ready', "ready without station")
    st = h2.query(['mcu station', 'configfile'])
    check(st['mcu station']['non_critical_disconnected'] is True, "offline at start")
    warns = [w['message'] for w in st['configfile']['warnings']]
    check(any("MCU 'station' is not connected" in w for w in warns), "runtime warning: %s" % warns)
    r = h2.gcode('MANUAL_STEPPER STEPPER=st ENABLE=1\nMANUAL_STEPPER STEPPER=st MOVE=10\nSET_PIN PIN=st_led VALUE=1')
    check('error' not in r, "commands work without station: %s" % r)
    h2.start_mcu('station')
    h2.ctl('station', 'adc ADC_TEMPERATURE 0.2249')
    r = h2.gcode('MCU_RECONNECT MCU=station', timeout=30)
    check(r is not None and 'error' not in r, "late connect ok: %s" % r)
    r = h2.gcode('MANUAL_STEPPER STEPPER=st ENABLE=1\nMANUAL_STEPPER STEPPER=st MOVE=0')
    h2.pump(1)
    check('error' not in r and 'queue_step' in flog(h2, 'station'), "moves reach station")
    check(h2.info()['state'] == 'ready', "ready")
    h2.pump(2)
    t = h2.query(['temperature_sensor st_mcu'])['temperature_sensor st_mcu']['temperature']
    check(10. < t < 80., "station mcu temperature calibrated after late connect: %s" % t)
finally:
    h2.stop()
print("\n".join(h2.responses[-6:]))

# 6. no cache and no station -> clear error
h3 = Harness('station_nocache', CFG, mcus=('main', 'station'), start_mcus=('main',))
try:
    h3.start_klippy()
    time.sleep(6)
    i = h3.info()
    check(i['state'] != 'ready' and 'cached data dictionary' in i['state_message'], "no cache -> error: %s" % i['state_message'][:200])
finally:
    h3.stop()
done()
