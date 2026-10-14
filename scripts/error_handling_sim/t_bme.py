# BME280: I2C errors report 9999 and the sensor recovers
from common import *
CFG = BASE + """
[temperature_sensor bme]
sensor_type: BME280
i2c_mcu: mcu
i2c_bus: i2c1
"""
h = Harness('bme', CFG)
try:
    h.start_klippy()
    h.wait_state('ready')
    h.pump(2)
    objs = ['temperature_sensor bme', 'bme280 bme']
    st = h.query(objs)
    check(abs(st['temperature_sensor bme']['temperature'] - 25.1) < 1,
          "bme reads 25C: %s" % st)
    h.ctl('main', 'i2c_fail 3')
    h.pump(2)
    st = h.query(objs)
    check(st['temperature_sensor bme']['temperature'] == 9999.,
          "error value reported: %s" % st)
    check(st['bme280 bme']['error'] is not None, "error in status")
    check(h.info()['state'] == 'ready', "printer still ready")
    check(any('START_NACK' in r for r in h.responses), "error message")
    h.pump(17)
    st = h.query(objs)
    check(abs(st['temperature_sensor bme']['temperature'] - 25.1) < 1,
          "recovered: %s" % st)
    check(st['bme280 bme']['error'] is None, "error cleared")
    check(any('restored' in r for r in h.responses), "restored message")
finally:
    h.stop()
done()
