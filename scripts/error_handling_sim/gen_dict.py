#!/usr/bin/env python3
# Build a fake klipper data dictionary from the DECL_COMMAND/sendf strings
# in the firmware sources (test helper, no cross compiler needed)
import sys, os, re, json

def c_strings(text):
    # Concatenate adjacent C string literals
    return "".join(re.findall(r'"((?:[^"\\]|\\.)*)"', text))

def scan(srcdir):
    commands, responses, enums = set(), set(), {}
    files = []
    for sub in ['', 'generic']:
        d = os.path.join(srcdir, sub)
        files += [os.path.join(d, f) for f in os.listdir(d) if f.endswith('.c')]
    for fn in files:
        data = open(fn).read()
        data = re.sub(r'//.*', '', data)
        for m in re.finditer(r'DECL_COMMAND(?:_FLAGS)?\s*\(([^;]*?)\)\s*;', data,
                             re.S):
            s = c_strings(m.group(1))
            if s:
                commands.add(s)
        for m in re.finditer(r'sendf\s*\(\s*((?:"(?:[^"\\]|\\.)*"\s*)+)', data):
            s = c_strings(m.group(1))
            if s:
                responses.add(s)
        for m in re.finditer(r'DECL_ENUMERATION\s*\(\s*"([^"]*)"\s*,\s*"([^"]*)"'
                             r'\s*,\s*([^)]*)\)', data):
            enums.setdefault(m.group(1), {})[m.group(2)] = len(
                enums.get(m.group(1), {}))
    return commands, responses, enums

def main():
    srcdir, outfile = sys.argv[1], sys.argv[2]
    mcu_name = sys.argv[3] if len(sys.argv) > 3 else "fake"
    commands, responses, enums = scan(srcdir)
    fixed = {"identify_response offset=%u data=%.*s": 0,
             "identify offset=%u count=%c": 1}
    commands.discard("identify offset=%u count=%c")
    responses.discard("identify_response offset=%u data=%.*s")
    msgid = 2
    cmd_ids, resp_ids = {"identify offset=%u count=%c": 1}, {
        "identify_response offset=%u data=%.*s": 0}
    for c in sorted(commands):
        cmd_ids[c] = msgid
        msgid += 1
    for r in sorted(responses - commands):
        resp_ids[r] = msgid
        msgid += 1
    shutdown_msgs = ["Command request", "ADC out of range", "I2C START NACK",
                     "Timer too close", "Rescheduled timer in the past"]
    enums["static_string_id"] = {m: i for i, m in enumerate(shutdown_msgs)}
    enums["pin"] = {"PA0": [0, 16], "PB0": [16, 16], "PC0": [32, 16], "ADC_TEMPERATURE": 48}
    enums["i2c_bus"] = {"i2c1": 0}
    enums["spi_bus"] = {"spi1": 0}
    enums.setdefault("i2c_bus_status", {
        "SUCCESS": 0, "NACK": 1, "START_NACK": 2, "START_READ_NACK": 3,
        "BUS_TIMEOUT": 4})
    d = {"commands": cmd_ids, "responses": resp_ids,
         "enumerations": enums,
         "config": {"CLOCK_FREQ": 72000000, "MCU": ("stm32g0b1xx" if mcu_name == "station" else "fake-" + mcu_name),
                    "ADC_MAX": 4095, "STATS_SUMSQ_BASE": 256,
                    "PWM_MAX": 255, "RECEIVE_WINDOW": 192,
                    "BUS_PINS_i2c1": "PB6,PB7"},
         "version": "fake-" + mcu_name, "build_versions": "fake",
         "app": "Klipper"}
    with open(outfile, 'w') as f:
        json.dump(d, f, indent=1, sort_keys=True)

if __name__ == '__main__':
    main()
