# Logging

Every opmode that has the flight recorder in it saves one log file per run on the hub.
You copy the files to the laptop and replay them in BuzzScope.

## Using it in an opmode

```java
FlightRecorder log = FlightRecorder.start(this);            // or start(this, follower) to also save the pose
while (opModeIsActive()) {
    ...
    log.put("Ultrasonic/leftIn", left);                     // any extra numbers you want, optional
    log.update();                                           // once per loop, after the puts
}
```

- `start` goes after the hardware is set up, before the first loop. It never throws and
  never returns null. If the file can't be opened you get a recorder that does nothing.
- Every loop in the opmode needs a `log.update()`, including init loops and "waiting"
  loops. A loop without one just leaves a gap in the log.
- There is no close call. The recorder finishes the file when the opmode ends.
- `log.put` takes a number, a true/false, text, or a `double[]`. Text is only written when
  it changes. The first `put` decides a channel's type, so don't put text into a channel
  that had numbers. Whole numbers are fine: `log.put("Auto/balls", balls)`.
- `log.event("ALIGNED")` saves something that happened, with its exact time.
- `log.pose(x, y, heading)` is for opmodes with no Pedro follower that still know where the
  robot is (inches and radians).
- `FlightRecorder.ENABLED = false` turns logging off everywhere.
- The ultrasonics are logged on their own, nothing to add. To see them live while the robot
  runs, turn on `LiveView.ultrasonic` in Panels (http://192.168.43.1:8001): the driver hub
  shows `Ultrasonic  L … R … skew …`, and Panels' Graph tab can plot `Ultrasonic/leftIn` and
  `rightIn`. It is off by default and goes back to off after a redeploy. It is done by the
  recorder, so it needs `FlightRecorder.ENABLED` on.

## Where the files are

`/sdcard/FIRST/logs/<OpMode>-<date>-<time>-<n>.wpilog` on the hub. The newest 30 are kept.

| Command | What it does |
|---|---|
| `make pull-logs 2` | copies the newest 2 logs into `robot-logs/` (no number = all 30) |
| `make viewer` | opens BuzzScope at http://127.0.0.1:8008 |
| `make analyze` | prints a one-page summary of the newest log |
| `make tuned` | lists numbers changed live in Panels and the line each goes on |

The files also open in AdvantageScope (File → Open Log). Drag `Field/Robot` onto a 2D Field
tab for the field view; `pose` is in inches and is for graphs.

## Channels

Sampled at most 100 times a second unless noted. Times are microseconds from when the
recorder started.

| Channel | Type | What it is |
|---|---|---|
| `events` | text | `init <OpMode>`, `start`, `stop`, `TUNED …`, and anything from `log.event`. Each has its own exact time |
| `opMode/kind` | text | `auto` or `teleop`, written once |
| `battery` | number | volts at the Control Hub. One hub is read every quarter second, so with one hub that is 4 times a second |
| `hubs/control/volts`, `hubs/control/currentAmps` | number | the Control Hub's input voltage and total current, written with the same timestamp. An Expansion Hub is `hubs/expansion<address>/…` |
| `gamepad1/axes`, `gamepad2/axes` | `double[]` | `[left x, left y, right x, right y, left trigger, right trigger]`, y is + up |
| `gamepad1/buttons`, `gamepad2/buttons` | whole number | one bit per button: a, b, x, y, left bumper, right bumper, dpad up, down, left, right, start, back, left stick, right stick, touchpad |
| `loop/totalNanos` | whole number | time since the previous `log.update()` |
| `loop/windowMaxTotalNanos` | whole number | the slowest loop since the last sample |
| `Drive/motors/power`, `Drive/motors/velocityTps` | `double[]` | `[front left, front right, back left, back right]`: motor power and encoder speed in ticks per second. 50 times a second at most. Only when all four drive motors are in the config |
| `Motors/<name>/power`, `Motors/<name>/velocityTps` | number | every other motor in the config |
| `Ultrasonic/leftIn`, `Ultrasonic/rightIn` | number | the two front ultrasonics in inches, calibrated (`UltrasonicConfig`). NaN while a sensor's supply reads under 1 V. 20 times a second at most. Only when all four ultrasonic ports are in the config |
| `Ultrasonic/leftRawIn`, `…/rightRawIn`, `…/leftSignalVolts`, `…/leftVccVolts`, `…/rightSignalVolts`, `…/rightVccVolts` | number | the same reading before calibration, and the two voltages it came from |
| `Ultrasonic/mount/spacingIn`, `Ultrasonic/mount/setbackIn` | number | where the sensors sit (`UltrasonicConfig`), written once, so BuzzScope draws the beams in the right place |
| `pose` | `double[]` | `[x in, y in, heading rad]` in Pedro's frame. Only with a follower or `log.pose` |
| `velocity` | `double[]` | `[vx in/s, vy in/s, turn rad/s]`. Only with a follower |
| `Field/Robot` | struct | the same pose in meters from the field center, for AdvantageScope |
| anything else | | whatever the opmode puts: `WallApproach/…`, `Align/…`, `Ball/…`, `Auto/…`, `DriveToWall/…`, `FlowerApproach/…` |

### Names the tools depend on

`tools/logs` and `tools/buzzscope` look these up by exact name. Renaming one doesn't
break the robot, it silently empties a graph or a check:

`events` (and the exact texts `start` and `stop`), `battery`, `hubs/control/volts`,
`hubs/control/currentAmps`, `gamepad1/axes`, `gamepad1/buttons`, `gamepad2/axes`,
`gamepad2/buttons`, `loop/totalNanos`, `loop/windowMaxTotalNanos`, `Drive/motors/power`,
`Drive/motors/velocityTps`, `pose`, `velocity`, `opMode/kind`, `Ultrasonic/leftIn`, `Ultrasonic/rightIn`,
`Ultrasonic/leftRawIn`, `Ultrasonic/rightRawIn`, `Ultrasonic/leftSignalVolts`, `Ultrasonic/rightSignalVolts`,
`Ultrasonic/leftVccVolts`, `Ultrasonic/rightVccVolts`, `Ultrasonic/mount/spacingIn`, `Ultrasonic/mount/setbackIn`,
`WallApproach/wallIn`, `DriveToWall/targetIn`.

The `TUNED` event format is read by `tools/logs/tuned.py`:
`TUNED <File.java> <Class>.<field> <code value> -> <value now>`.

Channels an opmode makes up itself can be named anything. Use `Group/name` so BuzzScope
puts them in a folder, and end the name with the unit (`In`, `Deg`, `Volts`, `Px`).

## What it costs the robot

- The loop only copies numbers into memory. A background thread writes the file, so a slow
  SD card can't hold the loop up. If the writer falls far behind, samples are skipped and
  the log says how many (`RECORDER SKIPPED …`).
- Battery volts and hub current are each a separate hub read of about 2 ms. They are read
  on the background thread, one hub every quarter second.
- Motor speed comes out of the hub's bulk read. A hub that had bulk caching OFF is switched
  to AUTO when the recorder starts, so the four speeds are one hub read, at most 50 times a
  second. AUTO never gives stale values. Hubs in MANUAL (`BulkReads.manual`) are
  left alone, and there the read is free because the opmode already did it.
- Motor power is free while the motor is being driven. For a motor nobody has set in half a
  second it is one hub read, and the recorder does at most one of those per loop.
- The ultrasonics are read from the hub's bulk read too, at most 20 times a second (the sensor
  only makes a new value every 100 ms). When the opmode already logged them that loop
  (`WallApproach.log`), the recorder doesn't read them again and keeps the opmode's numbers,
  which may use the flower dash's own calibration.
- Any error inside the recorder turns the recorder off for that run. The opmode keeps going.

These costs have not been measured on the robot yet. Compare `loop/totalNanos` with the
`log.update()` line commented out if a loop feels slower.

## Tests

`make test` runs the recorder's tests on the laptop (`TeamCode/src/test/java/…/logging`).
One of them writes `TeamCode/build/sample-logs/SampleTeleOp-….wpilog`, which the Python
tests read, so the Java and Python sides are checked against each other.
