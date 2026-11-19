# config: the numbers you edit

Open the file for the thing you want to change. After editing, `make deploy`.
Numbers marked `public static` (no `final`) can also be tried live in Panels
(http://192.168.43.1:8001). Panels changes are not saved: copy them back here.
`final` numbers are measurements or safety limits, so change those only in the code.

| I want to change... | File |
|---|---|
| a name from the Driver Hub robot configuration | `HardwareNames.java` |
| the ultrasonic calibration (scale / offset) | `UltrasonicConfig.java`, section 1 |
| where the ultrasonic sensors sit, or the flower stop distance | `UltrasonicConfig.java`, sections 2 and 3 |
| ultrasonic smoothing or the "bad reading" limits | `UltrasonicConfig.java`, sections 4 and 5 |
| the wall approach drive gains (kpDrive, kpTurn, ...) | `../mechanisms/WallApproach.java`, top of the class |
| path demo's drive-to-a-spot gains (kpDrive, kdDrive, ...) | `../mechanisms/PointDrive.java`, top of the class |
| the auto-alignment turn gains | `../opmodes/tests/flowerdash/UltrasonicAutoAlignment.java`, top |
| ball PID gains, camera exposure, the yellow colour range | `BallVisionConfig.java` |
| Starter Drive speed, or the 30 in limit | `DriveConfig.java` |
| Pedro drive / Pinpoint / follower numbers | `../pedro/Constants.java` (follower numbers come from AutoTune, leave them) |
