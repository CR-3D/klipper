# Buffer Feed

`buffer_feed` drives filament pre-feeders that fill a spring buffer **without
the host in the timing path**. Everything time critical (reading sensors,
generating steps, ramps, stopping) runs on the micro-controller. The steps are
merged into the step stream of the existing stepper, so they run in parallel
with normal moves of the same stepper.

A `[buffer_feed]` section describes one buffer. It has one or two channels
(pre-feeders) `t0` and `t1`:

```
Two channels into a Y splitter, one buffer:

entry_pin_t0 -> pre-feeder t0 -> exit_pin_t0 --\
                                                Y -> gate_pin -> buffer
entry_pin_t1 -> pre-feeder t1 -> exit_pin_t1 --/   (buffer_low_pin /
                                                    buffer_high_pin)

One channel per buffer (eg one buffer per IDEX head):

entry_pin_t0 -> pre-feeder t0 -> exit_pin_t0 -> gate_pin -> buffer
```

In a buffer with two channels only one channel (the "active channel") feeds
into the buffer. The other one is preloaded up to its exit sensor and takes
over when the active channel runs out (see "Switching channels").

## Why not `gcode_button` + `MANUAL_STEPPER`?

Every move issued by G-Code (including `MANUAL_STEPPER ... SYNC=0`) is
scheduled at the end of the toolhead lookahead queue, which can be 1-2 s ahead
during a print. On top of that a button command only runs between two print
commands. With `buffer_feed` the MCU decides itself. Reaction time with the
default settings: about 5-10 ms, independent of the print, motion planning
and CAN load.

## Functions

### Feeding

* The MCU polls `buffer_low_pin` (buffer nearly empty). Once it is active
  (debounced), the active channel feeds `distance` mm. The step interval ramps
  linearly from `start_velocity` to `velocity` (duration
  `(velocity-start_velocity)/accel`).
* `buffer_high_pin` (buffer full) is checked before every step. When it
  triggers, the stepper decelerates with `decel` and stops. Choose `distance`
  so that `buffer_high_pin` is not normally reached.
* `gate_pin` is optional. If it is configured, feeding only happens while it
  is active (filament has arrived at the buffer).
* While the host moves the same stepper in the opposite direction (retract),
  feeding waits until that move is done. The direction pin is never changed
  under a running move and is restored before the next host move.
* If `buffer_low_pin` is still active after `max_runs` runs in a row, this
  is a fault (jam, broken sensor): feeding stops, an error is shown and
  `fault_gcode` runs. Without filament at the entry sensor it is a runout
  instead (see below).
* A new feed run only starts after `buffer_low_pin` was released in between.

### Runout

When the entry sensor of the active channel releases while its exit sensor
still detects filament (the channel is loaded through), the MCU reports a
runout at once and `runout_gcode` runs. From then on feeding reacts to
edges of `buffer_low_pin` only: every time it triggers, one run follows, with
no run limit and **no fault**. While the end of the filament is still in the
pre-feeder, each run lifts the buffer above `buffer_low_pin`, the extruder
uses it up and the next edge follows. Once the end has left the pre-feeder,
a run no longer releases `buffer_low_pin` and feeding stops. The print is
paused by the printer's own filament sensor. The runout ends when filament
is inserted at the entry sensor again.

Filament pulled out of a channel that is not loaded through (exit sensor
free, eg preloaded or after a failed load) is not a runout.

### Retract

If `buffer_high_pin` triggers while feeding is enabled (buffer pushed beyond
full, eg by a long retraction or by unloading through the extruder), the
active channel moves back until `buffer_high_pin` releases (with
deceleration), at most `retract_distance` mm per run. If it is still active
after `retract_max_runs` retract runs in a row, this is a fault.
`retract_distance: 0` disables the function.

### Loading

If `entry_pin_<channel>` and `exit_pin_<channel>` are configured:

1. When the entry sensor triggers, the pre-feeder starts at once (ramp
   `load_*`) and runs until the exit sensor triggers.
2. If the exit sensor does not trigger within `load_timeout`: abort, error
   and `load_fault_gcode`.
3. After the exit sensor triggered, the pre-feeder moves
   `load_clear_distance`:
   * positive: further forward, measured from the exit sensor, deceleration
     included (choose a value larger than the braking distance),
   * negative: stop, then move back this distance (preload position: the
     exit sensor is free again),
   * 0: stop with deceleration.
4. A new load only starts after the entry sensor was released in between.
   If the exit sensor is already active, nothing moves.

Loading works independently of feeding. In a buffer with two channels both
channels load automatically, only the active channel feeds.

Filling the buffer is done with a macro (see `BUFFER_FEED_MOVE` and
`config/sample-buffer-feed.cfg`).

### Unloading

`BUFFER_FEED_UNLOAD` switches feeding and loading of the channel off, moves
back until the exit sensor releases (at most `unload_exit_distance`), then
until the entry sensor releases (at most `unload_entry_distance`) and
switches loading on again. The command returns at once, so a running unload
can be aborted with `BUFFER_FEED_ABORT` (eg by holding a button, see the
sample configs). If a sensor does not release within its distance, the
unload stops with an error.

### Channel state

Every channel has a simple state for status LEDs:

* `empty`: no filament at the entry sensor,
* `preloaded`: filament at the entry sensor, exit sensor free,
* `loaded`: exit sensor detects filament or the channel feeds,
* `error`: feed, retract or load fault. Cleared by the next move, by
  enabling the channel or when the entry sensor changes.

When the state of a channel changes, `state_gcode` runs with
`params.CHANNEL` and `params.STATE`.

### Motors off

`M84` and the idle timeout also pause `buffer_feed`. It resumes on its own:

* filament inserted at an entry sensor: driver on, load starts,
* `buffer_low_pin` triggers: driver on, the active channel feeds.

Any `SET_BUFFER_FEED`, `BUFFER_FEED_MOVE` or `BUFFER_FEED_LOAD` command
resumes as well. The host watches the sensors for this, so the wake up takes
a few milliseconds longer than the reaction while running.

### Selecting the active channel

At startup (`enable: True`) and on `SET_BUFFER_FEED ... ENABLE=1` without
`CHANNEL`:

* one channel: it is the active channel,
* two channels: the channel whose exit sensor detects filament is the active
  channel (the other one is preloaded and stopped before its exit sensor).
  If both exit sensors detect filament, no channel feeds and an error is
  shown.

`SET_BUFFER_FEED ... CHANNEL=<ch> FEED=1` makes a channel the active channel.
This is refused while the other channel is loaded through to the buffer (its
exit sensor detects filament).

### Switching channels

When `gate_pin` releases, `gate_release_gcode` runs. A macro can then decide
whether to switch channels, for example: the print is running, the active
channel has no filament at its entry sensor and the other channel has. The
macro switches feeding over and feeds the new channel once until the buffer
is full (see `config/sample-buffer-feed-dual.cfg`).

## Requirements

* Firmware update of the MCU with the stepper and the sensors
  (`make menuconfig`: "Support autonomous filament buffer feeding" is enabled
  by default, except on AVR). Host and MCU must match: after every update of
  this module rebuild and flash the MCU, otherwise Klipper stops at startup
  with "Protocol error". The `out/klipper.dict` does not need to be copied,
  the host reads the command definitions from the firmware when it connects.
  It is only needed for tests in batch mode (`klippy.py -d`).
* The steppers must use "step on both edges". TMC drivers with step/dir do
  this by default, as long as no large `step_pulse_duration` is set.
  Otherwise Klipper stops at startup with an error.
* All steppers and sensors of a buffer must be on the same MCU.
* The sensor pins may also be used by `gcode_button` and similar.

## Configuration

See `config/sample-buffer-feed.cfg` (one channel) and
`config/sample-buffer-feed-dual.cfg` (two channels into a Y splitter).

| Option | Default | Description |
|---|---|---|
| `buffer_low_pin` | | Sensor "buffer nearly empty", starts feeding |
| `buffer_high_pin` | | Sensor "buffer full", ends feeding, starts a retract (optional) |
| `gate_pin` | | Feeding only while active, `gate_release_gcode` when it releases (optional) |
| `stepper_t0`, `stepper_t1` | | Config section of the pre-feeder stepper (`manual_stepper x` or `extruder_stepper x`), at least one |
| `entry_pin_t0`, `entry_pin_t1` | | Entry sensor of the channel: starts loading, detects a runout |
| `exit_pin_t0`, `exit_pin_t1` | | Exit sensor of the channel: ends loading |
| `distance` | | Distance per feed run (mm) |
| `velocity` | | Feed velocity (mm/s), max. 25000 steps/s |
| `accel` | 0 | Acceleration (mm/s²), 0 = full speed at once |
| `decel` | `accel` | Deceleration, eg when `buffer_high_pin` triggers (mm/s²) |
| `start_velocity` | 2.0 | Start and end velocity of the ramps (mm/s) |
| `retract_distance` | 0 | Maximum distance of a retract run (mm), 0 = off |
| `max_runs` | 3 | Feed runs in a row without effect until a fault (0 = off) |
| `retract_max_runs` | 10 | Retract runs in a row while `buffer_high_pin` stays active until a fault (0 = off) |
| `unload_exit_distance` | 2000 | `BUFFER_FEED_UNLOAD`: maximum distance back until the exit sensor releases (mm) |
| `unload_entry_distance` | 600 | `BUFFER_FEED_UNLOAD`: maximum distance back until the entry sensor releases (mm) |
| `poll_interval` | 0.005 | Sensor poll interval on the MCU (s) |
| `buffer_low_debounce` | 0.010 | Debounce time of `buffer_low_pin` (s) |
| `buffer_high_samples` | 2 | Consecutive samples (one per poll or step) until `buffer_high_pin` counts |
| `entry_debounce` | 0.010 | Debounce time of the entry sensors (s) |
| `exit_samples` | 2 | Samples (one per step) until the exit sensor counts |
| `load_velocity` | `velocity` | Load velocity (mm/s) |
| `load_accel` | `accel` | Load acceleration (mm/s²) |
| `load_decel` | `load_accel` | Load deceleration (mm/s²) |
| `load_start_velocity` | `start_velocity` | Start and end velocity of the load ramps (mm/s) |
| `load_timeout` | 10 | Time (s) until the exit sensor must trigger, max. 60 |
| `load_clear_distance` | 0 | Distance after the exit sensor (mm), negative = back |
| `fault_gcode` | | G-Code on a feed or retract fault |
| `load_fault_gcode` | | G-Code on a load timeout |
| `runout_gcode` | | G-Code on a runout |
| `gate_release_gcode` | | G-Code when `gate_pin` releases |
| `state_gcode` | | G-Code when the state of a channel changes (`params.STATE`) |
| `enable` | True | Enable feeding and loading at startup |
| `enable_stepper` | True | Switch the driver on when needed |

The templates get `params.BUFFER`, `params.CHANNEL` (`t0`, `t1`, empty if no
channel is active) and `params.REASON`.

The ramps are linear in the step interval and therefore approximate a
constant acceleration. The ramp duration matches the formula above.

## G-Code commands

`CHANNEL=t0|t1` selects the channel. It may be omitted if the buffer has only
one channel.

* `SET_BUFFER_FEED BUFFER=<name> [CHANNEL=] [ENABLE=0|1] [FEED=0|1]
  [AUTO_LOAD=0|1] [DISTANCE=] [VELOCITY=] [ACCEL=] [DECEL=]
  [RETRACT_DISTANCE=] [LOAD_VELOCITY=] [LOAD_ACCEL=] [LOAD_DECEL=]
  [LOAD_TIMEOUT=] [LOAD_CLEAR_DISTANCE=]`
  * `FEED`: feeding (and retract) of the channel. `FEED=1` makes it the
    active channel and switches feeding of the other channel off.
  * `AUTO_LOAD`: automatic loading of the channel.
  * `ENABLE`: with `CHANNEL` both functions of that channel. Without
    `CHANNEL`: `ENABLE=0` switches everything off, `ENABLE=1` switches
    loading on for all channels and selects the active channel (see above).
  * `FEED=0`, `AUTO_LOAD=0` and `ENABLE=0` stop a running automatic run of
    that function at once. With both off a running manual move stops too.
  * The other parameters apply to the whole buffer.
* `BUFFER_FEED_MOVE BUFFER=<name> [CHANNEL=] [DISTANCE=] [VELOCITY=]
  [ACCEL=] [DECEL=] [STOP=] [STOP_ON=TRIGGER|RELEASE] [WAIT=0|1]
  [WAIT_TIMEOUT=]` starts a single move. Without `CHANNEL` the active channel
  moves. The command returns at once (unless `WAIT=1`), the MCU runs the move
  on its own. If a previous move of the channel is still running, the
  command is refused with an error.
  * `WAIT=1`: wait until the MCU reports the end of the move (needed if
    another move of the channel follows). If the move is aborted or takes
    longer than `WAIT_TIMEOUT` (default: computed from distance and velocity),
    the macro stops with an error. Do not use `WAIT=1` in macros that run
    during a print, the print would stop until the move is done.
  * `DISTANCE`: maximum distance. **Negative values move backwards**
    (unloading). Default: `distance` of the config (forward).
  * `STOP`: sensor that ends the move (with deceleration): `none`, `high`,
    `low`, `gate`, `entry`, `exit` (`entry` and `exit` of the moving
    channel). Default: `high` when moving forward and `buffer_high_pin` is
    configured, otherwise `none`.
  * `STOP_ON`: `TRIGGER` (default) stops when the sensor triggers,
    `RELEASE` when it releases (eg filament leaves the entry sensor).
  * If the sensor is already in the stop state, the move stops after a few
    steps. Without a stop sensor it runs the full `DISTANCE`.
* `BUFFER_FEED_LOAD BUFFER=<name> [CHANNEL=] [WAIT=1] [WAIT_TIMEOUT=]`
  starts the load sequence manually, eg for testing. With `WAIT=1` a timeout
  (`load_timeout`) makes the macro fail.
* `BUFFER_FEED_UNLOAD BUFFER=<name> [CHANNEL=] [EXIT_DISTANCE=]
  [ENTRY_DISTANCE=] [WAIT=1] [WAIT_TIMEOUT=]` unloads the channel (see
  "Unloading"). With `WAIT=1` the macro waits for the end and fails if the
  unload stops early.
* `BUFFER_FEED_WAIT BUFFER=<name> [CHANNEL=] [TIMEOUT=]` waits for the end of
  running moves (all channels if `CHANNEL` is omitted).
* `BUFFER_FEED_ABORT BUFFER=<name> [CHANNEL=]` aborts running moves, loads
  and unloads at once, without deceleration (all channels if `CHANNEL` is
  omitted).
* `QUERY_BUFFER_FEED BUFFER=<name>` shows the state of the buffer and all
  sensors. Useful to check wiring and pin polarity.

## Status

`printer["buffer_feed <name>"]` (Moonraker, macros):

* `active_channel`: `t0`, `t1` or empty
* `sleeping`: paused because the motors are off
* `buffer_low`, `buffer_high`, `gate`: sensor states (True = active)
* `channels.<ch>.entry`, `channels.<ch>.exit`: sensor states
* `channels.<ch>.feed`, `channels.<ch>.auto_load`: functions enabled
* `channels.<ch>.runout`: True after a runout until filament is inserted
* `channels.<ch>.state`: `empty`, `preloaded`, `loaded` or `error`
* `channels.<ch>.unloading`: `BUFFER_FEED_UNLOAD` is running
* `channels.<ch>.last_result`: `done`, `stop_sensor`, `aborted`, `fault`,
  `busy`, `load_timeout`, `loaded`, `runout`, `retract_fault`
* `channels.<ch>.last_steps`, `fault_count`, `load_fault_count`
* `last_result`, `distance`, `velocity`, `accel`, `decel`,
  `retract_distance`, `retract_max_runs`

The sensor states come from the host and may lag a few milliseconds behind
the MCU.

## Notes

* **Own unload macros:** with `BUFFER_FEED_MOVE` switch feeding and loading
  of the channel off first (`FEED=0 AUTO_LOAD=0`, this aborts a running
  move), otherwise the buffer feeds again while moving back. Manual moves
  also run with the automatic functions switched off. Use `WAIT=1` if
  another move follows. `BUFFER_FEED_UNLOAD` does all of this itself.
* **Buttons:** macros started by a `gcode_button` that use `WAIT=1` block
  G-Code until the move is done, so a second button press only arrives
  afterwards. Commands that return at once (`BUFFER_FEED_MOVE` without
  `WAIT`, `BUFFER_FEED_UNLOAD`) can be aborted by another button.
* **Moves are tagged:** every move started by the host carries a tag. Reports
  of other runs (automatic runs, an aborted earlier move) do not end a
  `WAIT=1` early.
* **Position:** the extra steps are invisible to the host. The host position
  of the stepper stays exact for host moves.
* **Sizing:** the feed velocity must be well above the maximum consumption
  of the extruder. If the buffer is not refilled, `max_runs` applies.
* Step pulses of host moves and extra steps are generated one after another
  (never at the same time). Whether the minimum spacing is enough for the
  driver used has to be checked on the hardware.
* Macro names must not contain digits followed by letters (`FEEDER1_LOAD` is
  parsed as `FEEDER1`). Use names like `FEEDER_LOAD_T0`.

## Test

`scripts/buffer_feed_sim/run.sh` builds the real code of `src/stepper.c` and
`src/buffer_feed.c` on the PC and tests it against a simulated scheduler,
GPIO and spring buffer (both stepper code paths): ramps, exact step counts,
closed loop with consumption, parallel host extrusion, retract, position
tracking, stop sensor with deceleration, gate, loading (latency, clear
forward and backward, timeout, restart), runout (at once, one run per
`buffer_low_pin` edge, no fault, not for a channel that is not loaded
through), separate
enable of feeding and loading, retract on `buffer_high_pin` with
`retract_max_runs`, abort, faults and re-enable.

`python3 scripts/buffer_feed_sim/host_test.py` tests the host module against
mocked Klipper objects: channel selection, refusing a second channel, runout,
gate event and channel switch, pause and wake up after motors off, faults,
unload sequence and abort, channel states (`state_gcode`) and the G-Code
commands.

Both tests do not replace a test on real hardware.
