# 23209 BioBuzz robot code

FTC robot code for team 23209 (BIOBUZZ). Plain Java `LinearOpMode`s on the FTC SDK 11.2.0
with Pedro Pathing. You can still upload from Android Studio like always. The terminal
tools below are extras that make it faster.

## Quick start (new laptop)

1. Install **Android Studio** and open this folder once (it creates `local.properties`).
2. Install **JDK 21**. The Mac's default Java is too new for Gradle 8.9. `make` finds
   JDK 21 by itself.
3. Install Python 3.10+ (and Node 20 if you want every test to run).
4. In a terminal, in this folder:

```bash
make
```

That prints every command. The ones you will use most:

| I want to… | Run |
|---|---|
| check the code compiles | `make build` |
| run the tests (no robot needed) | `make test` |
| put the code on the robot | `make deploy` |
| copy the last 2 run logs to my laptop | `make pull-logs 2` |
| look at a run | `make viewer` then open http://127.0.0.1:8008 |
| get a quick summary of the newest log | `make analyze` |
| tune numbers live (Panels) | open [192.168.43.1:8001](http://192.168.43.1:8001) while connected to the robot |
| see which deploy it would do, change nothing | `make check-deploy` |
| force a full install / force a hot reload | `make install` / `make hot` |
| remove hot-reloaded code from the hub | `make unsloth` |
| see the numbers I changed live in Panels | `make tuned` |

## Ports on the hub

| Port | What |
|---|---|
| [8080](http://192.168.43.1:8080) | robot controller page |
| [8001](http://192.168.43.1:8001) | Panels |
| [10158](http://192.168.43.1:10158) | Pedro AutoTune (websocket on 12649) |
| [8765](http://192.168.43.1:8765), [8766](http://192.168.43.1:8766), [8767](http://192.168.43.1:8767), [8768](http://192.168.43.1:8768), [8769](http://192.168.43.1:8769), [8770](http://192.168.43.1:8770) | flower dash (uses the first free one, the driver hub shows which) |

## A normal day

```
edit code  →  make build  →  make deploy  →  run an opmode  →  make pull-logs 1  →  make viewer
```

- **Wi-Fi first, then USB.** Connect the laptop to the robot's Wi-Fi, or plug in a USB
  cable. The tools try Wi-Fi (`192.168.43.1:5555`) and fall back to USB.
- `make deploy` picks the fast way (hot reload, about a second) when only the opmode Java
  changed, and the slow way (full install, about a minute) when it has to. After a hot
  reload, **stop the opmode and init it again** to get the new code.
- Every opmode records a log of the run (battery, motors, gamepads, loop time, and the
  numbers that opmode cares about). BuzzScope replays it.

## What's in here

| Path | What it is |
|---|---|
| `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/` | the robot code |
| `…/teamcode/config/` | **the numbers you edit**: hardware names, ultrasonic calibration, ball vision, drive speed |
| `…/teamcode/sensors/`, `…/teamcode/mechanisms/` | shared code the opmodes use (ultrasonic sensor, wall approach, drive to a spot, drive motors) |
| `…/teamcode/opmodes/` | the opmodes: `teleop/`, `auto/`, `tests/`, `vision/` |
| `…/teamcode/logging/` | the flight recorder that writes the run logs |
| `…/teamcode/tuning/` | `Tunables`, which tells you what to copy back after tuning in Panels |
| `tools/` | the laptop tools (deploy, logs, BuzzScope) |
| `docs/TOOLS.md` | **the guide to every tool**, with examples |
| `docs/LOGGING.md` | how logging works and every channel name |
| `CLAUDE.md` | instructions for Claude Code in this repo |

## Docs

- [docs/TOOLS.md](docs/TOOLS.md): how to use each tool, step by step, and what to do when
  it fails.
- [docs/LOGGING.md](docs/LOGGING.md): adding logging to an opmode, and the channel list.
- [tools/buzzscope/README.md](tools/buzzscope/README.md): the log viewer in detail.

## Working with Claude Code

This repo has a `CLAUDE.md`, two agents and a skill, so you can say things like
"deploy this", "why did that run stop short?" or "save my tuning" and Claude knows which
commands to run. See the last section of [docs/TOOLS.md](docs/TOOLS.md).

## After a run: reading a log

1. `make pull-logs 2` (the hub must be reachable).
2. `make analyze` gives loop time, battery and the event timeline for the newest log.
3. To look at one thing closely:
   `python3 tools/logs/analyze_wpilog.py --channel Ultrasonic/leftIn,WallApproach/toGoIn`
   (any channel names; the list is in [docs/LOGGING.md](docs/LOGGING.md)).
4. Times count from `start`, so anything during init is negative.

## If something goes wrong

- Plain `./gradlew` fails on this Mac (default Java is 24). Use `make`, or set `JAVA_HOME`
  to JDK 21.
- `adb` is not on PATH. The scripts find it through `local.properties`.
- The flower dash page is an asset, so a hot reload won't update it. It needs a full
  install (`make deploy` knows this).

## Things to know

- Don't touch these: `FtcRobotController/`, `build.common.gradle`, `libs/` (FTC SDK files),
  `pedro/procedures/` and `pedro/Tuning.java` (Pedro's tuners), and the follower numbers
  in `pedro/Constants.java` (they come from AutoTune runs).
- Don't edit gradle files without asking: the versions are pinned on purpose (Sloth 0.2.4).
- Numbers changed live in Panels are **not saved**. They reset on the next init. Use
  `make tuned` to see what to copy into the code.
- Pull requests: the template has a checklist. CI runs the build and tests on every push.

Original FTC SDK readme and license: [LICENSE](LICENSE), [FtcRobotController](FtcRobotController).
