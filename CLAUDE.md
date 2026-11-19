# 23209 BioBuzz robot code

FTC robot code for team 23209, BIOBUZZ season. Plain Java `LinearOpMode`s on the FTC SDK
with Pedro Pathing. The team talks to Claude, then puts the code on the robot with
`make deploy` or from Android Studio. Explain things simply: the students are learning.

## Commands

Run `make` to list them. They pick JDK 21 by themselves.

| Command | Use it for |
|---|---|
| `make build` | compile everything, don't touch the robot |
| `make test` | tests that run on the laptop (Java, Python, JS) |
| `make deploy` | put the code on the robot: hot reload if enough, full install if not |
| `make check-deploy` | say which of the two it would do, change nothing |
| `make install` / `make hot` | force a full install / a hot reload |
| `make unsloth` | remove hot-reloaded code from the hub |
| `make pull-logs 2` | copy the newest 2 logs off the hub into `robot-logs/` |
| `make viewer` | BuzzScope log viewer at http://127.0.0.1:8008 |
| `make analyze` / `make tuned` | summary of the newest log / numbers changed live in Panels |

- Plain `./gradlew` fails on this Mac: the default Java is 24 and Gradle 8.9 can't run on
  it. Use `make`, or set `JAVA_HOME=/Library/Java/JavaVirtualMachines/jdk-21.jdk/Contents/Home`.
- `adb` is not on PATH. The scripts find it through `local.properties`
  (`tools/hub-connect.sh --adb` prints the path).
- The hub is tried over Wi-Fi first (`192.168.43.1:5555`), then USB.

## Where things are

Robot code is in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`:

| Folder | What's in it |
|---|---|
| `config/` | **the numbers students edit**: `HardwareNames`, `UltrasonicConfig` (calibration, geometry, limits), `BallVisionConfig`, `DriveConfig`, `LiveView` (on/off switches for live sensor views). See `config/README.md` |
| `sensors/` | `UltrasonicSensor` (one MaxBotix sensor, raw distance), `BulkReads` |
| `mechanisms/` | `WallApproach` (drive up to a wall on the ultrasonics), `PointDrive` (drive to a spot on odometry, used by path demo), `MecanumMotors` (drive motor setup) |
| `opmodes/teleop/` | Starter Drive teleops |
| `opmodes/auto/pedro/` | Pedro autos (`PathDemoAllCombos`, `RedTippedExport`) |
| `opmodes/tests/` | test opmodes (`FlowerApproachTest`, `DriveToWallTest`, `UltrasonicReadout`, ...) |
| `opmodes/tests/flowerdash/` | the flower dash: a small web page served by the robot on port 8765+ |
| `opmodes/vision/` | ball finding with the webcam |
| `pedro/` | `Constants.java` (drive + Pinpoint + follower tuning), Pedro's own tuners |
| `logging/` | flight recorder (also logs the two ultrasonics in every opmode) |
| `tuning/` | `Tunables`: tells you what to copy back after live tuning |

Outside the robot code: `tools/` (deploy scripts, log tools, BuzzScope), `docs/LOGGING.md`,
`TeamCode/src/test/` (laptop tests), `TeamCode/src/main/assets/flower_dash/` (the web page).

Hardware names in the robot config (use the constants in `config/HardwareNames.java`): `frontLeftMotor`, `frontRightMotor`, `backLeftMotor`,
`backRightMotor`, `shooter`, `pinpoint`, `limelight`, `Webcam 1`, `ultrasonic_left_signal`,
`ultrasonic_left_vcc`, `ultrasonic_right_signal`, `ultrasonic_right_vcc`.

Ports on the hub: 8080 robot controller page, 8001 Panels, 10158 Pedro AutoTune (websocket
on 12649), 8765-8770 flower dash.

## How code is written here

- One opmode is one `LinearOpMode` class with its own numbers at the top. Numbers shared by
  several opmodes go in a `config/` class. Shared code goes in `sensors/` or `mechanisms/` as
  plain classes; there is no subsystem framework. Don't add one unless asked (see "For later").
- Mecanum math stays written out in each opmode, so students can read it; `MecanumMotors` only
  does setup.
- Comments are short, lowercase, plain words, and say why. Put them above the line or after
  it with three spaces: `static final double maxTurn = 0.35;   // why this number`.
  No Javadoc blocks in new code.
- Units go in the name or the comment: `stopAtIn`, `settleSec`, `// in`, `// rad/s`.
- Opmodes that read sensors from the hub call `BulkReads.manual(hardwareMap)` once
  and `BulkReads.clear(hubs)` at the top of every loop. Forgetting the clear means
  readings never change.
- Anything that drives the robot from a test stops the motors in a `finally` block.
- Keep the dead-man habit in test opmodes: the robot only moves while a button is held.

## Rules for new code

1. **New opmodes get the flight recorder.** `FlightRecorder.start(this)` (or
   `start(this, follower)`) after hardware setup, `log.update()` once in every loop,
   `log.put(...)` for the numbers that matter. See `docs/LOGGING.md`.
2. **Numbers someone will want to tune go in Panels.** For PID gains, speeds, tolerances and
   similar: mark the class `@Configurable`, make each number `public static` (not `final`),
   one per line, and put `static { Tunables.track(TheClass.class); }` below them. See the
   comment at the top of `tuning/Tunables.java`. Do this without being asked.
3. **Safety limits are not tunable.** Power caps, timeouts and "is this reading sane" limits
   stay `static final`, and the code must clip to them after any tunable is applied.
4. **Don't rename a log channel the tools read.** The list is in `docs/LOGGING.md`.
5. **Assets need a full install.** The flower dash page is read from the app's assets, so a
   hot reload won't update it. `make deploy` already knows this.

## Things not to touch

- `FtcRobotController/`, `build.common.gradle`, `libs/`: FTC SDK files.
- `pedro/procedures/` and `pedro/Tuning.java`: Pedro's tuners.
- The follower numbers in `pedro/Constants.java`: they come from AutoTune runs.
- Version numbers. The pins that matter, all in `TeamCode/build.gradle`: Sloth is held at
  0.2.4 with the Load plugin 0.2.4 and `com.bylazar.sloth:fullpanels:0.2.4+1.0.12`. Newer
  Sloth needs Android Gradle Plugin 8.13 and this project is on 8.7.0. Pedro's tuning
  library asks for Sloth 0.3.0, which is why the pin is `strictly`.
- **Any gradle file: show the exact diff and wait for an OK before editing.** Try the change
  in a scratch copy first. Use the `dependency-verifier` agent before changing a version.
- Don't commit or push unless asked.

## Putting code on the robot

Use the `deploy` skill, or `make deploy` directly. A hot reload takes about a second and
only swaps the classes under `teamcode`; it takes effect at the next opmode init and resets
anything changed in Panels. A full install is needed after gradle, asset, resource or
manifest changes and the script picks it when needed. Installing from Android Studio still
works; the next `make deploy` after that is a full install.

## After a run: finding out what happened

1. `make pull-logs 2` (the hub must be reachable).
2. `python3 tools/logs/analyze_wpilog.py --json robot-logs/<file>` for loop time, battery
   and the event timeline. With no file it reads the newest log.
3. Drill in with `--channel Ultrasonic/leftIn,WallApproach/toGoIn` (any channel names).
4. Times count from `start`; init is negative, like BuzzScope shows.
5. If the log doesn't settle it, say so: give the likely causes in order and the one channel
   or test that would decide between them. If the opmode has no recorder in it, add it.

## "Save my tuning"

When the user says they tuned something in Panels and wants to keep it:

1. `make pull-logs 1`, then `python3 tools/logs/tuned.py --json`.
2. For `copy` entries, edit that line: replace the old number with the new one.
3. For `expr` entries the line is a formula (`0.0040 / pixelScale`); replace the whole right
   side with the new number and say so.
4. `stale` or `lost`: don't guess. Show the user what the log says and what the code says.
5. `make build`, then tell the user to `make deploy`.

Panels changes are never saved on the robot. The code is the only place tuning is kept.

## Agents and skills in this repo

- `build-checker` (Haiku): runs `make build` and reports compiler errors as `file:line`.
  Use it after a batch of edits instead of reading build output yourself.
- `dependency-verifier` (Sonnet): checks a library version really exists before a version
  change. It only checks; it never edits.
- `deploy` skill: how to deploy and how to read what the script says.

## For later

- **Subsystems.** `mechanisms/` holds plain shared classes now (`WallApproach`,
  `PointDrive`, `MecanumMotors`). When a shooter or intake gets shared code, add it there the same way, with
  its own log channels. A scaffolding agent for new mechanisms is worth adding at that point.
- **Loop cost of the recorder** has not been measured on the robot. Starter Drive and the
  ball opmodes are the ones to check, because the recorder adds a hub read there.
- **EGM plugin.** `/egm:egm-drift-report` checks that this file and `docs/` still match the
  code; worth running when they feel out of date. `/egm:egm-bootstrap` writes a README per
  folder and becomes useful once there are subsystems.
