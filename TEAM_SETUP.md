# Team 23209 BioBuzz

This repository is based on the official FIRST Tech Challenge Robot Controller SDK v11.2.

## Basic drive OpMode

`BioBuzzTeleOp` is a simple manual mecanum-drive TeleOp:

- Left stick up/down: drive forward and backward
- Left stick left/right: strafe
- Right stick left/right: turn

Configure these four DC motors on the Robot Controller:

| Configuration name | Position |
| --- | --- |
| `front_left` | Front left |
| `front_right` | Front right |
| `back_left` | Back left |
| `back_right` | Back right |

If a wheel spins backward, reverse that motor's direction in `BioBuzzTeleOp.java`.

Open the folder in Android Studio, connect the Robot Controller, and run the `TeamCode` configuration. On the Driver Station, select **BioBuzz: Basic Drive** under the **23209** group.

## Git remotes

The official FTC repository is saved as the `upstream` remote so SDK updates can be fetched later. No team GitHub remote has been added yet.
