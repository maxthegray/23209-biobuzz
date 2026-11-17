# Tool guide

Every tool in this repo, what it is for, and how to use it. Run commands from the repo
folder. `make` with no arguments lists the commands.

Contents: [build and test](#build-and-test) · [deploy](#deploy-to-the-robot) ·
[logging](#the-flight-recorder) · [pull logs](#pull-logs) · [BuzzScope](#buzzscope-replaying-a-run) ·
[analyze](#analyze-a-summary-in-the-terminal) · [live tuning](#live-tuning-with-panels) ·
[Claude Code](#claude-code-agents-and-skills) · [CI](#ci-and-pull-requests) · [problems](#when-something-fails)

## Build and test

| Command | What it does |
|---|---|
| `make build` | compiles the robot code. Doesn't touch the robot. Run this before deploying |
| `make test` | runs the tests that need no robot: the recorder (Java), the log tools (Python) and BuzzScope's math (JS) |

Use `make`, not plain `./gradlew`. Plain Gradle fails here because the Mac's default Java
is 24 and Gradle 8.9 can't run on it. `make` uses JDK 21 for you.

## Deploy to the robot

The laptop must reach the hub: on the robot's Wi-Fi, or plugged in by USB. Wi-Fi is tried
first, then USB, and it waits up to 30 seconds.

```bash
make deploy
```

It prints its decision first:

```
hub: Wi-Fi (192.168.43.1:5555)
RECOMMENDATION: HOT
REASON: only teamcode Java can be different from what's on the hub
```

| Result | Time | What happens |
|---|---|---|
| **HOT** | ~1 s | only the opmode classes are swapped. **Stop and re-init the opmode** to run the new code |
| **FULL** | ~1 min | the whole app is installed and the robot app restarts. Wait for the Driver Hub to reconnect |

It chooses FULL by itself after a gradle, asset (flower dash page), resource or manifest
change, after an Android Studio install, and on a hub it has never seen.

| Command | When to use it |
|---|---|
| `make check-deploy` | see what `deploy` would do, change nothing |
| `make install` | force a full install, for example when the robot seems to run old code |
| `make hot` | force a hot reload |
| `make unsloth` | remove hot-reloaded code from the hub so the installed app runs as installed |
| `make connect` | just check the hub is reachable |
| `make scrcpy` | show the hub's screen on the laptop (needs `brew install scrcpy`) |

Good to know:

- Uploading from Android Studio still works. The next `make deploy` after it is a FULL
  install, which is correct.
- A hot reload resets numbers you changed in Panels. Save your tuning first
  (see [live tuning](#live-tuning-with-panels)).
- Don't deploy while an opmode is driving the robot.

## The flight recorder

Every opmode with `FlightRecorder` in it saves one file per run on the hub:
`/sdcard/FIRST/logs/<OpMode>-<date>-<time>-<n>.wpilog` (newest 30 are kept). It is
already in the teleops, the ultrasonic opmodes, the two ball-chasing opmodes, `Path Demo` and
`Red Tipped`. Not yet in: `AutoTestingPedro`, `OtherAutoTest`, `LimelightHiveCheck`,
`GoToBallPIDLearningTest`, `ShooterVelocityTest`, `DriveMotorSpeedTest`, `VisionDetectYellow`.

To add it to a new opmode:

```java
FlightRecorder log = FlightRecorder.start(this);   // after hardware setup, before the loop
while (opModeIsActive()) {
    ...
    log.put("Ultrasonic/leftIn", left);            // any numbers you want to see later
    log.update();                                  // once per loop
}
```

Every loop (including init loops) needs a `log.update()`. A recorder problem never
stops the opmode. Full details and the channel list: [LOGGING.md](LOGGING.md).

## Pull logs

```bash
make pull-logs 2      # newest 2 logs into robot-logs/   (no number = all 30)
```

Needs the hub reachable (Wi-Fi or USB). Files land in `robot-logs/`, which git ignores.

## BuzzScope: replaying a run

```bash
make viewer           # then open http://127.0.0.1:8008
```

A browser viewer for the logs in `robot-logs/`. It runs on your laptop and needs no
internet. What you get:

- **Graphs** for any channel, with presets: Overview, Drive, Timing, Ultrasonic, Ball.
- **Field replay** and a **3D view** of the robot's path (opmodes with a Pedro follower).
- **Match timeline** of the events (`init`, `start`, `stop`, and the opmode's own events).
- **Run summary** with battery, loop time and the **TUNED VALUES** list.
- A traction check from motor speed against how the robot actually moved.

Starter Drive and the ball opmodes have no position, so their field is empty but all the
graphs work. More in [tools/buzzscope/README.md](../tools/buzzscope/README.md).
Logs also open in AdvantageScope (File → Open Log).

## Analyze: a summary in the terminal

```bash
make analyze                                                          # newest log
python3 tools/logs/analyze_wpilog.py --json robot-logs/<file>         # machine-readable
python3 tools/logs/analyze_wpilog.py --json --channel Ultrasonic/leftIn,WallApproach/toGoIn
```

Prints loop time, battery and the event timeline. `--channel` adds the channels you ask for,
but only together with `--json` (without it the option is ignored). This is
also what Claude uses to answer "what happened in that run?". Times count from `start`;
init is negative.

## Live tuning with Panels

Panels is a web page served by the robot at **http://192.168.43.1:8001** while the robot
app is running. Numbers marked `@Configurable` can be changed there with no redeploy.

Tunable now: ultrasonic calibration (`config/UltrasonicConfig`: `leftScale`, `leftOffset`,
`rightScale`, `rightOffset`), the wall approach and alignment gains
(`mechanisms/WallApproach`, `UltrasonicAutoAlignment`), the ball PID gains
(`config/BallVisionConfig`) and the teleop speed (`config/DriveConfig`). Safety limits (power caps,
timeouts) are deliberately not tunable.

**Panels changes are never saved.** To keep them:

1. Run the opmode so the log is recorded. Changed numbers show on the Driver Hub
   telemetry as **COPY INTO CODE** and go into the log as `TUNED` events.
2. `make pull-logs 1`, then:

```bash
make tuned
```

   It prints each changed number, the file it lives in, and the line to put it on.
3. Edit the code, `make build`, `make deploy`.

With Claude Code you can just say "save my tuning".

To make a new number tunable: mark the class `@Configurable`, make the field
`public static` (not `final`), and add `static { Tunables.track(TheClass.class); }`.
See the comment at the top of `tuning/Tunables.java`.

## Claude Code: agents and skills

| Name | Say it like | What it does |
|---|---|---|
| `deploy` skill | "deploy this", "put it on the robot" | runs `make deploy` and explains HOT or FULL |
| `build-checker` agent | "does it build?" | runs `make build` and lists errors as `file:line` |
| `dependency-verifier` agent | "can we update X?" | checks a library version exists before any gradle change. Never edits |
| save my tuning (a procedure in `CLAUDE.md`, not a skill) | "I tuned this in Panels, keep it" | pulls the log, runs `tuned.py`, edits the code lines |
| what happened (a procedure in `CLAUDE.md`, not a skill) | "why did that run stop?" | pulls and analyzes the newest log |

`CLAUDE.md` holds the rules Claude follows here: new opmodes get the recorder, tunable
numbers go in Panels, gradle changes need your OK first, and nothing is committed or
pushed unless you ask.

## CI and pull requests

`.github/workflows/ci.yml` runs on every push and pull request: it compiles the robot code
and runs the Java, Python and JS tests. The pull request template has a checklist
(`make build`, `make test`, tested on the robot or not). If no CI run shows up, an admin of
the GitHub repo needs to enable Actions.

## When something fails

| You see | Do this |
|---|---|
| `error: no hub after 30s` | join the robot's Wi-Fi or plug in USB, then `make connect` |
| `error: can't find adb` | open the project in Android Studio once (creates `local.properties`) |
| `error: can't find JDK 21` | install JDK 21 |
| compile errors | fix the `file:line` shown, `make build`, then deploy again |
| robot seems to run old code | `make unsloth`, or `make install` |
| hot reload fails right after a gradle change | `make install` once |
| `no logs in robot-logs/` | `make pull-logs 1` first |
| BuzzScope shows nothing | check `robot-logs/` has `.wpilog` files, then reload the page |
| an opmode's graph is empty | the channel may have been renamed. See "Names the tools depend on" in [LOGGING.md](LOGGING.md) |
