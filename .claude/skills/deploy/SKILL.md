---
name: deploy
description: Put the current code on the robot's hub. Use when the user says deploy, upload, push to the robot, install, hot reload, or "put it on the robot".
---

# Deploy to the robot

## Before deploying

1. The code has to compile. If Java files changed since the last successful build, run
   `make build` first (or use the `build-checker` agent) and stop if it fails.
2. The laptop has to reach the hub: on the robot's Wi-Fi, or plugged in by USB. Wi-Fi is
   tried first.

## Deploy

```sh
make deploy
```

It prints what it decided before doing it:

```
hub: Wi-Fi (192.168.43.1:5555)
RECOMMENDATION: HOT
REASON: only teamcode Java can be different from what's on the hub
```

- **HOT** is a hot reload: about a second, only the classes under
  `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/` are swapped. It takes effect the
  next time an opmode is initialized, so tell the user to stop and re-init the opmode.
- **FULL** is a full install: about a minute, the whole app is rebuilt and installed and the
  robot controller app restarts.

To see the decision without deploying: `make check-deploy`.

## How it decides

After each full install the script leaves a note on the hub: a hash of everything a hot
reload cannot update (gradle files, assets, resources, the manifest, `FtcRobotController/`)
and the time the app was installed. It hot reloads only when both still match. So it picks
FULL by itself after:

- a change to a gradle file, an asset (the flower dash page), a resource or the manifest
- an install from Android Studio or from another laptop
- the first deploy to a hub

Nothing is remembered on the laptop, so this stays right with several laptops and hubs.

## Forcing one or the other

| Command | When |
|---|---|
| `make install` | force a full install, for example when the robot behaves as if it has old code |
| `make hot` | force a hot reload |
| `make unsloth` | remove hot-reloaded code from the hub, so the installed app runs as installed |

## Tell the user

- Which one happened, HOT or FULL, and the reason line.
- After HOT: stop the opmode and init it again to get the new code. Numbers changed live in
  Panels go back to what the code says, so offer "save my tuning" first if they were tuning
  (see CLAUDE.md).
- After FULL: the app restarts; wait for the Driver Hub to reconnect.

## When it fails

- `error: no hub after 30s`: the laptop isn't on the robot's Wi-Fi and no USB cable is
  connected. Ask the user to fix that. Don't loop.
- `error: can't find adb`: `local.properties` is missing. Opening the project in Android
  Studio once creates it.
- `error: can't find JDK 21`: report it. Don't switch to another Java.
- A compile error: report the `file:line` errors and stop. Don't deploy an older build.
- A hot reload that fails right after a gradle change: run `make install` once, then hot
  reloads work again.

## Never

- Edit gradle files to get a deploy through.
- Deploy while an opmode is driving the robot without the user saying it's safe.
