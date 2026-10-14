# ADC values of the EPCOS 100K thermistor (4.7k pullup) used by the tests
import sys
from common import KLIPPY
sys.path.insert(0, KLIPPY)
from extras import thermistor
_t = thermistor.Thermistor(4700., 0.)
_t.setup_coefficients(25., 100000., 150., 1641.9, 250., 226.15)
def adc(temp):
    return _t.calc_adc(temp)
