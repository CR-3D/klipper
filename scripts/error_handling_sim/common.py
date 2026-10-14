# Shared test helpers
import sys, os, time, faulthandler
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from harness import Harness, KLIPPY
faulthandler.dump_traceback_later(150, exit=True)
PID = os.getpid()
BASE = """
[mcu]
serial: /tmp/klipper_host_fake_main_%d
[printer]
kinematics: none
max_velocity: 100
max_accel: 100
""" % PID
FAILS = []
def check(cond, msg):
    print(("PASS " if cond else "FAIL ") + msg)
    if not cond:
        FAILS.append(msg)
def done():
    print("RESULT: %s" % ("OK" if not FAILS else "FAILED: %s" % FAILS))
