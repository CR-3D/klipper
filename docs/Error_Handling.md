# Error handling

This document describes the extensions of this Klipper fork that keep
the printer running after errors that are not safety critical. By
default (without the config options below) Klipper behaves exactly
like upstream Klipper.

## Informative I2C sensors (BME280)

A BMP180/BMP280/BME280/BMP388/BME680 sensor no longer shuts down the
printer on a communication error such as `I2C START NACK`. Instead:

- an error is reported (`!! BME280 'name': ... (reporting 9999,
  retrying every 5s)`),
- the sensor reports a temperature, pressure and humidity of 9999,
- the `error` field of the `bme280 <name>` status object contains the
  error message,
- the sensor is re-initialized every 5 seconds. Once it answers again
  `communication restored` is reported.

A sensor that does not answer at startup does not prevent the printer
from starting either.

The micro-controller must run code that reports I2C errors to the host
(the `i2c_transfer` command). Older micro-controller code shuts down
by itself (`MCU 'mcu' shutdown: I2C START NACK`) and has to be
recompiled and flashed.

## Heater faults

Add a `[heater_faults]` section to make heater errors non-fatal (see
the [config reference](Config_Reference.md#heater_faults)):

```
[heater_faults]
```

A heater faults when

- its temperature is outside of `min_temp`/`max_temp` for about one
  second (for example a disconnected or shorted thermistor), or
- it is "not heating at expected rate" (see
  [verify_heater](Config_Reference.md#verify_heater)).

On a fault the heater is switched off and can not be turned on again,
an error is reported and `fault_gcode` is run. The default
`fault_gcode` pauses a running print. A `M109`/`TEMPERATURE_WAIT` that
waits for the heater returns, so that the print is paused (and not
aborted). Once the problem is fixed:

```
RESET_HEATER_FAULT HEATER=extruder1
```

then heat up again and `RESUME` the print (the `RESUME` macro should
restore the temperatures).

With `hard_max_temp_margin` (above 0) the micro-controller still shuts
down the printer if a heater exceeds `max_temp` by more than the given
margin. Do not use it with PT100/PT1000 sensors: a disconnected sensor
reads an extremely high temperature and would trigger the hard limit.
With the default of 0 all range errors are handled by the host. The
micro-controller always switches off a heater if the host stops
updating it.

Faults are not stored. Faults found during startup (for example a tool
head with a disconnected sensor) do not prevent the printer from
starting, they are shown as warnings once the printer is ready and
have to be cleared after every start.

The range (`min_temp`/`max_temp`) of `temperature_sensor` sections is
checked by the host as well. Out of range is reported as an error
(and in the `faults` of the `heater_faults` status) instead of shutting
down the printer.

## Disabling heaters and sensors

A heater or temperature sensor can be disabled, for example to print
with one head of an IDEX printer while the other head is defective:

```
DISABLE_HEATER HEATER=extruder1
DISABLE_HEATER SENSOR="temperature_sensor chamber"
```

A disabled heater can not be turned on and its temperature is not
checked. The state is not stored: after a restart the heater/sensor is
enabled again (a defective one faults again and has to be cleared or
disabled again). Enable it with `ENABLE_HEATER`. `QUERY_HEATER_FAULTS` lists all faults and
disabled heaters. Disabling requires the `[heater_faults]` section.

## Non-critical micro-controllers

An additional micro-controller (for example the filament station) can
be marked as non-critical:

```
[mcu station]
canbus_uuid: 0123456789ab
is_non_critical: True
#reconnect_interval: 10
```

- If it is not available at startup the printer starts without it and
  a warning is shown. All commands for it are discarded (steppers on
  it can be moved, nothing happens). The micro-controller must have
  been connected at least once, its data dictionary is cached in
  `.mcu_station.dict` next to the printer config.
- If the connection is lost or the micro-controller shuts down during
  operation the printer keeps running. An error is reported and
  `disconnect_gcode` is run (default: pause a running print).
- `MCU_RECONNECT MCU=station` brings it back online (it is reset if
  needed and configured again). With `reconnect_interval` this is done
  automatically while the printer is not printing.
- After a reconnect the stepper drivers (TMC) and neopixels on it are
  initialized again and its motors are disabled. Other output pins
  start with their configured start values. Modules with their own
  state on the micro-controller can use the `mcu:reconnected` event
  (with the mcu object as parameter) to restore it.
- If the firmware of the micro-controller has changed a `RESTART` is
  required.
- `printer["mcu station"].non_critical_disconnected` and
  `disconnect_reason` report the state.

Do not mark a micro-controller as non-critical if it controls the
motion system or heaters that must be supervised.
