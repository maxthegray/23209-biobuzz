package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorEx;   // turn back on with the shooter
// import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/*
 * BIOBUZZ demo auto (30 s). Pinpoint odometry + a simple go-to-point, no pedro yet.
 *
 * field coords (inches, origin = field centre):
 *   +x = toward the audience, +y = red wall -> blue wall, heading 0 = facing audience, ccw +
 *   red owns y < 0 (columns A-C), blue owns y > 0 (columns D-F). never cross in auto (G402)
 *
 * plan (red, blue is the same thing spun 180 degrees around the centre):
 *   1. launch the 4 preloads into our raised hive cell (audience side at the start)
 *      3 nectar already sit in it, so ~3 pollen should tip it = 20 pts
 *   2. measured approach to our 2 flowers and pull pollen out the bottom (G418 allows that)
 *   3. go round to the rear side, our raised cell flipped over there after the tip, launch again
 *   4. park partly in our loading zone off the wall = leave 3 + park 5
 *   if the clock runs low we skip straight to parking
 *
 * no shooter or intake on the bot yet, so launching and slurping are just timed waits
 * that hold the robot still for as long as the real thing would take.
 *
 * INIT controls: dpad left = red, dpad right = blue, dpad up = audience wall start, dpad down = alliance wall start
 */
@Autonomous(name = "Hypothetical Demo Auton", group = "teaching", preselectTeleOp = "Starter Drive")
public class HypotheticalDemoAuton extends LinearOpMode {

    // ---------- field spots (red side, blue gets spun 180) ----------

    // hive pivots sit 12.75 in either side of centre, cells are ~13.4 in out along x
    private static final double hiveLaneY = -12.75;
    private static final double cellX = 13.4;

    // flowers sit on tile seams against the wall, d = how far their middle is off the wall (guess, measure it)
    private static final double flowerOffWall = 3.0;
    private static final double flowerReach = 2.4;   // top ring sticks out this far toward the field (Fig 9-12)
    private static final double halfBot = 9.0;   // 18 in robot
    private static final double snuggleGap = 1.0;   // leave a hair of space, we can't grab the flower (G415)

    // two starts, both backed onto a wall and facing the rear (heading 180)
    private static final Spot startAudienceWall = new Spot(72 - halfBot, hiveLaneY, 180);   // lined up with the raised cell, shoot right away
    private static final Spot startAllianceWall = new Spot(44, -72 + halfBot, 180);   // clear of the flower at x=24 and the loading zone

    private static final Spot launchSpotOne = new Spot(44, hiveLaneY, 180);   // audience side, shoot toward the rear
    private static final Spot launchSpotTwo = new Spot(-44, hiveLaneY, 0);   // rear side, shoot toward the audience

    // red flowers: one on the red wall at x=24, one on the rear wall at y=-24
    private static final Spot wallFlowerLineup = new Spot(24, -44, -90);
    private static final Spot wallFlowerSnug = new Spot(24, -72 + flowerOffWall + flowerReach + halfBot + snuggleGap, -90);
    private static final Spot sneakAroundHive = new Spot(-36, -44, 180);   // keeps us clear of the hive frame (edge at y=-24.7)
    private static final Spot rearFlowerLineup = new Spot(-44, -24, 180);
    private static final Spot rearFlowerSnug = new Spot(-72 + flowerOffWall + flowerReach + halfBot + snuggleGap, -24, 180);

    // loading zone is tile A5 against the red wall, x -48..-24, y -72..-61
    private static final Spot parkSpot = new Spot(-36, -58, 90);   // bumper hangs into the zone, body off the wall

    // ---------- timing ----------

    private static final double autoLength = 30.0;
    private static final double bailToParkAt = 24.0;   // parking takes ~3-4 s, don't gamble
    private static final double hardStopAt = 29.5;

    // ---------- driving ----------

    private static final double kpDrive = 0.06;   // power per inch
    private static final double kpTurn = 0.02;   // power per degree
    private static final double zoomSpeed = 0.7;
    private static final double measuredSpeed = 0.2;   // slow creep for the last few inches to a flower
    private static final double minPush = 0.08;   // below this the wheels just buzz
    private static final double closeEnoughIn = 1.0;
    private static final double closeEnoughDeg = 2.0;

    // ---------- pretend mechanisms (no shooter or intake mounted yet) ----------

    private static final long launchMsPerBall = 700;   // guess at how long one shot takes
    private static final long slurpMs = 1500;   // guess at intake time per flower

    // real shooter settings, turn back on once it's mounted
    // private static final double launcherTicksPerSec = 1800;   // guess, tune on the real flywheel
    // private static final double feederPush = 0.8;
    // private static final double feederRest = 0.2;
    // private static final long feederBeatMs = 350;
    private static final int maxBallsHeld = 4;   // G407, never control more than 4
    private static final int ballsPerFlower = 2;   // 2 + 2 keeps us at 4, needs a real ball counter later

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private GoBildaPinpointDriver pinpoint;
    // private DcMotorEx launcher;
    // private Servo feeder;
    // private DcMotor intake;

    private final ElapsedTime autoClock = new ElapsedTime();
    private boolean isBlue = false;
    private int ballsOnBoard = 4;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // same pod setup as the teleops, measure the offsets on the real robot
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                      GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        // shooter + intake hookup, turn back on once they're mounted
        // launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        // feeder = hardwareMap.get(Servo.class, "feeder");
        // intake = hardwareMap.get(DcMotor.class, "intake");
        // launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // feeder.setPosition(feederRest);

        // pick alliance + start during INIT
        boolean useAudienceStart = true;
        while (opModeInInit()) {
            if (gamepad1.dpad_left) isBlue = false;
            if (gamepad1.dpad_right) isBlue = true;
            if (gamepad1.dpad_up) useAudienceStart = true;
            if (gamepad1.dpad_down) useAudienceStart = false;

            telemetry.addData("Alliance", "%s  (dpad left red / right blue)", isBlue ? "BLUE" : "RED");
            telemetry.addData("Start", "%s  (dpad up / down)", useAudienceStart ? "audience wall" : "alliance wall");
            telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
            telemetry.addLine("robot must touch the wall, stay on our side, 4 pollen touching it (G304)");
            telemetry.update();
        }
        if (isStopRequested()) return;

        autoClock.reset();

        Spot start = useAudienceStart ? startAudienceWall : startAllianceWall;
        Spot here = forUs(start);
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, here.x, here.y, AngleUnit.DEGREES, here.h));

        // 1. launch the preloads into the raised cell (audience side at the start)
        // spinUpLauncher();
        if (useAudienceStart) {
            aimAndLaunch(cellX, hiveLaneY);   // already lined up, just fire
        } else {
            if (driveTo(launchSpotOne, zoomSpeed)) aimAndLaunch(cellX, hiveLaneY);
        }

        // 2. measured approach to both flowers, pull pollen from the bottom
        boolean stillOnTime = true;
        stillOnTime = stillOnTime && driveTo(wallFlowerLineup, zoomSpeed);
        stillOnTime = stillOnTime && driveTo(wallFlowerSnug, measuredSpeed);
        if (stillOnTime) slurpFlower();
        stillOnTime = stillOnTime && driveTo(wallFlowerLineup, zoomSpeed);   // back out straight

        stillOnTime = stillOnTime && driveTo(sneakAroundHive, zoomSpeed);
        stillOnTime = stillOnTime && driveTo(rearFlowerLineup, zoomSpeed);
        stillOnTime = stillOnTime && driveTo(rearFlowerSnug, measuredSpeed);
        if (stillOnTime) slurpFlower();
        stillOnTime = stillOnTime && driveTo(rearFlowerLineup, zoomSpeed);

        // 3. after the tip our raised cell is the rear one, shoot at it from the rear side
        stillOnTime = stillOnTime && driveTo(launchSpotTwo, zoomSpeed);
        if (stillOnTime && ballsOnBoard > 0) aimAndLaunch(-cellX, hiveLaneY);

        // 4. park, always, even if we bailed early
        // spinDownLauncher();
        // intake.setPower(0);
        driveTo(parkSpot, zoomSpeed);
        stopWheels();

        while (opModeIsActive()) {
            telemetry.addData("Done", "%.1f s", autoClock.seconds());
            telemetry.update();
        }
    }

    // ---------- moving ----------

    // drive + turn at the same time until we're on the spot. false = out of time, go park
    private boolean driveTo(Spot redSpot, double speedCap) {
        Spot target = forUs(redSpot);
        boolean parking = redSpot == parkSpot;

        while (opModeIsActive()) {
            double t = autoClock.seconds();
            if (t > hardStopAt || (!parking && t > bailToParkAt)) {
                stopWheels();
                return false;
            }

            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            double x = pose.getX(DistanceUnit.INCH);
            double y = pose.getY(DistanceUnit.INCH);
            double h = pose.getHeading(AngleUnit.DEGREES);

            double errX = target.x - x;
            double errY = target.y - y;
            double errH = wrapDegrees(target.h - h);
            double howFar = Math.hypot(errX, errY);

            if (howFar < closeEnoughIn && Math.abs(errH) < closeEnoughDeg) {
                stopWheels();
                return true;
            }

            // field direction -> robot direction
            double hRad = Math.toRadians(h);
            double forward = errX * Math.cos(hRad) + errY * Math.sin(hRad);
            double left = -errX * Math.sin(hRad) + errY * Math.cos(hRad);

            // speed from distance, capped, with a minimum so it doesn't stall right before the spot
            double push = Range.clip(kpDrive * howFar, 0, speedCap);
            if (howFar > closeEnoughIn) push = Math.max(push, minPush);
            double scale = howFar > 1e-6 ? push / howFar : 0;
            forward *= scale;
            left *= scale;

            double spin = Range.clip(kpTurn * errH, -speedCap, speedCap);

            makeWheelsGo(forward, -left, -spin);   // teleop math wants strafe right + and turn clockwise +

            telemetry.addData("Going to", "(%.0f, %.0f) %.0f deg", target.x, target.y, target.h);
            telemetry.addData("At", "(%.1f, %.1f) %.1f deg", x, y, h);
            telemetry.addData("Off by", "%.1f in  %.1f deg", howFar, errH);
            telemetry.addData("Clock", "%.1f / %.0f s", t, autoLength);
            telemetry.update();
        }
        return false;
    }

    private void makeWheelsGo(double drive, double strafe, double turn) {
        double fl = drive + strafe + turn;
        double fr = drive - strafe - turn;
        double bl = drive - strafe + turn;
        double br = drive + strafe - turn;

        // shrink everything together so the direction doesn't get bent when one wheel maxes out
        double biggest = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                                                Math.max(Math.abs(bl), Math.abs(br))));
        frontLeft.setPower(fl / biggest);
        frontRight.setPower(fr / biggest);
        backLeft.setPower(bl / biggest);
        backRight.setPower(br / biggest);
    }

    private void stopWheels() {
        makeWheelsGo(0, 0, 0);
    }

    // ---------- launching ----------

    // private void spinUpLauncher() {
    //     launcher.setVelocity(launcherTicksPerSec);
    // }

    // private void spinDownLauncher() {
    //     launcher.setVelocity(0);
    // }

    // turn in place to face the cell, then sit still for as long as shooting would take
    private void aimAndLaunch(double redCellX, double redCellY) {
        Spot cell = forUs(new Spot(redCellX, redCellY, 0));

        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        double faceIt = Math.toDegrees(Math.atan2(cell.y - y, cell.x - x));

        // aim spot is already in our alliance's coords, so undo forUs before handing it to driveTo
        if (!driveTo(forUs(new Spot(x, y, faceIt)), zoomSpeed)) return;

        // wait for the flywheel to get up to speed
        // spinUpLauncher();
        // ElapsedTime waitForWheel = new ElapsedTime();
        // while (opModeIsActive() && waitForWheel.seconds() < 1.5
        //         && launcher.getVelocity() < launcherTicksPerSec * 0.95) {
        //     idle();
        // }

        while (opModeIsActive() && ballsOnBoard > 0 && autoClock.seconds() < hardStopAt) {
            sleep(launchMsPerBall);   // delete this line once the feeder below is on

            // push one ball into the flywheel
            // feeder.setPosition(feederPush);
            // sleep(feederBeatMs);
            // feeder.setPosition(feederRest);
            // sleep(feederBeatMs);
            ballsOnBoard--;
            telemetry.addData("Launched", "%d left", ballsOnBoard);
            telemetry.update();
        }
    }

    // ---------- flowers ----------

    // sit at the retrieval opening like an intake would, only pollen comes out the bottom (G418)
    private void slurpFlower() {
        int room = maxBallsHeld - ballsOnBoard;
        if (room <= 0) return;   // already full, G407

        // intake.setPower(1.0);
        sleep(slurpMs);   // how long the intake runs
        // intake.setPower(0);

        ballsOnBoard += Math.min(room, ballsPerFlower);
    }

    // ---------- helpers ----------

    // blue is red spun 180 around the centre, the field is built that way
    private Spot forUs(Spot red) {
        if (!isBlue) return red;
        return new Spot(-red.x, -red.y, wrapDegrees(red.h + 180));
    }

    private static double wrapDegrees(double deg) {
        while (deg > 180) deg -= 360;
        while (deg <= -180) deg += 360;
        return deg;
    }

    private static class Spot {
        final double x, y, h;
        Spot(double x, double y, double h) {
            this.x = x;
            this.y = y;
            this.h = h;
        }
    }
}
