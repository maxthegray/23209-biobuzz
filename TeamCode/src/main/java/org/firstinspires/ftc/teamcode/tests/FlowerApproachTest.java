package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.Arrays;

// step by step ultrasonic check, the driver hub tells you what to do
// part 1: checks the distance formula against a tape measure, the robot never moves
// part 2: drives up to the object, but only while you hold RB
@TeleOp(name = "Flower Approach Test (ultrasonic)", group = "test")
public class FlowerApproachTest extends LinearOpMode {

    private static final double[] tapeDistances = {4, 6, 8, 12, 18, 24};   // in, sensor face to object. extra points near where flowers stop
    private static final double settleSec = 1.0;   // datasheet: the output goes through a 2 Hz filter, so give it time to catch up
    private static final double recordSec = 1.5;
    private static final double goodEnough = 0.25;   // in, worst miss we still call accurate
    private static final double jumpy = 0.3;   // in, readings wobbling more than this while still = something's wrong
    private static final double tooFar = 30;   // in, won't drive unless both sensors see something closer than this

    private enum Stage { CALIBRATE, RESULTS, DRIVE }

    private Stage stage = Stage.CALIBRATE;
    private int step = 0;
    private final double[] leftAvg = new double[tapeDistances.length];
    private final double[] rightAvg = new double[tapeDistances.length];
    private final double[] leftWobble = new double[tapeDistances.length];
    private final double[] rightWobble = new double[tapeDistances.length];
    private final boolean[] recorded = new boolean[tapeDistances.length];
    private Fit leftFit, rightFit;
    private boolean usingFix = false;

    private boolean lastA, lastB, lastX, lastRb;
    private boolean driving = false;
    private WallApproach.Result result = null;
    private String driveNote = "";

    @Override
    public void runOpMode() {
        Follower follower = Constants.create(hardwareMap);
        WallApproach approach = new WallApproach(hardwareMap);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        while (opModeInInit()) {
            telemetry.addLine("Press START, then follow the steps on this screen");
            telemetry.addLine("The robot only ever moves while you hold RB");
            approach.showReadings(telemetry);
            telemetry.update();
            sleep(50);
        }
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            boolean a = gamepad1.a && !lastA, b = gamepad1.b && !lastB, x = gamepad1.x && !lastX;
            boolean rbPressed = gamepad1.right_bumper && !lastRb;
            lastA = gamepad1.a;
            lastB = gamepad1.b;
            lastX = gamepad1.x;
            lastRb = gamepad1.right_bumper;

            if (stage == Stage.CALIBRATE) calibrate(approach, a, b, x);
            else if (stage == Stage.RESULTS) results(approach, a, b);
            else drive(follower, approach, rbPressed, b);

            follower.update();
            telemetry.update();
        }
        follower.manual(0, 0, 0);
        follower.update();
    }

    private void calibrate(WallApproach approach, boolean a, boolean b, boolean x) {
        double tape = tapeDistances[step];
        telemetry.addLine(String.format("FORMULA CHECK  step %d of %d", step + 1, tapeDistances.length));
        telemetry.addLine(String.format("1. Put the object %.1f in from the front of the round sensor parts", tape));
        telemetry.addLine("2. Tape BOTH sensors, nudge it till both read the same");
        telemetry.addLine("3. Press A, then don't touch anything for 2.5 s");
        telemetry.addLine("");
        telemetry.addLine(String.format("Live  left %5.2f in   right %5.2f in", approach.rawLeft(), approach.rawRight()));
        telemetry.addLine(String.format("Sensor supply %.2f V (should be about 3.3)", approach.supplyVolts()));
        telemetry.addLine("");
        telemetry.addLine("A = record   B = redo last step   X = done, show results");

        if (a) record(approach, step);
        else if (b && step > 0) step--;
        else if (x) finishCalibration();
    }

    // waits for the sensor's filter to settle, then averages 1.5 s of readings at one tape distance
    private void record(WallApproach approach, int i) {
        ElapsedTime settling = new ElapsedTime();
        while (opModeIsActive() && settling.seconds() < settleSec) {
            approach.rawLeft();
            approach.rawRight();
            telemetry.addLine(String.format("Settling at %.1f in... don't touch anything", tapeDistances[i]));
            telemetry.update();
            sleep(20);
        }

        double leftSum = 0, rightSum = 0, leftMin = 999, leftMax = -999, rightMin = 999, rightMax = -999;
        int count = 0;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < recordSec) {
            double l = approach.rawLeft(), r = approach.rawRight();
            if (Double.isNaN(l) || Double.isNaN(r)) {
                telemetry.addLine("A sensor isn't reading (supply under 1 V). Check the wiring, then press A again");
                telemetry.update();
                sleep(1500);
                return;
            }
            leftSum += l;
            rightSum += r;
            leftMin = Math.min(leftMin, l);
            leftMax = Math.max(leftMax, l);
            rightMin = Math.min(rightMin, r);
            rightMax = Math.max(rightMax, r);
            count++;
            telemetry.addLine(String.format("Recording %.1f in... don't touch anything", tapeDistances[i]));
            telemetry.addLine(String.format("left %5.2f   right %5.2f", l, r));
            telemetry.update();
            sleep(20);
        }
        if (count == 0) return;
        leftAvg[i] = leftSum / count;
        rightAvg[i] = rightSum / count;
        leftWobble[i] = leftMax - leftMin;
        rightWobble[i] = rightMax - rightMin;
        recorded[i] = true;
        if (step < tapeDistances.length - 1) step++;
        else finishCalibration();
    }

    private void finishCalibration() {
        leftFit = Fit.of(tapeDistances, leftAvg, recorded);
        rightFit = Fit.of(tapeDistances, rightAvg, recorded);
        stage = Stage.RESULTS;
    }

    private void results(WallApproach approach, boolean a, boolean b) {
        telemetry.addLine("FORMULA CHECK RESULTS (inches)");
        telemetry.addLine(" tape    left   right   wobble");
        for (int i = 0; i < tapeDistances.length; i++) {
            if (!recorded[i]) continue;
            telemetry.addLine(String.format("%5.1f  %6.2f  %6.2f   %.2f/%.2f", tapeDistances[i], leftAvg[i], rightAvg[i], leftWobble[i], rightWobble[i]));
        }
        if (leftFit.points < 3) {
            telemetry.addLine("Need at least 3 distances to judge the formula");
            telemetry.addLine("A = drive check with no fix   B = redo formula check");
            if (a) {
                usingFix = false;
                stage = Stage.DRIVE;
            } else if (b) {
                restartCalibration();
            }
            return;
        }
        telemetry.addLine("Left:  " + leftFit.verdict());
        telemetry.addLine("Right: " + rightFit.verdict());
        if (maxWobble() > jumpy) {
            telemetry.addLine("Readings wobble a lot. Unplug one sensor, if the other calms down they hear each other");
            telemetry.addLine("If not, datasheet says put a 100uF capacitor across the sensor's V+ and GND");
        }
        if (leftFit.worstBefore > goodEnough || rightFit.worstBefore > goodEnough) {
            telemetry.addLine("Paste into WallApproach in FlowerApproachTest.java:");
            telemetry.addLine(String.format("  leftScale = %.4f, leftOffset = %.3f", leftFit.scale, leftFit.offset));
            telemetry.addLine(String.format("  rightScale = %.4f, rightOffset = %.3f", rightFit.scale, rightFit.offset));
        }
        telemetry.addLine("");
        telemetry.addLine("A = drive check with this fix   B = redo formula check");

        if (a) {
            usingFix = leftFit.sane() && rightFit.sane();
            if (usingFix) approach.useCorrection(leftFit.scale, leftFit.offset, rightFit.scale, rightFit.offset);
            stage = Stage.DRIVE;
        } else if (b) {
            restartCalibration();
        }
    }

    private void restartCalibration() {
        step = 0;
        Arrays.fill(recorded, false);
        stage = Stage.CALIBRATE;
    }

    private double maxWobble() {
        double worst = 0;
        for (int i = 0; i < tapeDistances.length; i++) {
            if (recorded[i]) worst = Math.max(worst, Math.max(leftWobble[i], rightWobble[i]));
        }
        return worst;
    }

    // only drives while RB is held, letting go stops it right away
    private void drive(Follower follower, WallApproach approach, boolean rbPressed, boolean b) {
        telemetry.addLine("DRIVE CHECK" + (usingFix ? " (using this run's fix)" : " (no fix applied)"));
        telemetry.addLine("Aim at the object from 1-2 ft, a little crooked is fine");
        telemetry.addLine("HOLD RB to drive, let go to stop instantly");
        telemetry.addLine(String.format("Should stop with the bumper %.1f in from it", WallApproach.stopDistance));
        if (WallApproach.sensorSetback == 0) telemetry.addLine("(sensorSetback is 0, so 'bumper' = sensor face for now)");
        telemetry.addLine("");

        if (b) {
            follower.manual(0, 0, 0);
            driving = false;
            stage = Stage.RESULTS;
            return;
        }

        if (!gamepad1.right_bumper) {
            if (driving) follower.manual(0, 0, 0);
            driving = false;
        } else if (rbPressed) {
            double l = approach.leftNow(), r = approach.rightNow();
            if (Double.isNaN(l) || Double.isNaN(r) || l > tooFar || r > tooFar) {
                result = null;
                driveNote = "Didn't start: both sensors need something within " + (int) tooFar + " in";
            } else {
                approach.start();
                result = WallApproach.Result.RUNNING;
                driving = true;
                driveNote = "";
            }
        }
        if (driving && result == WallApproach.Result.RUNNING) result = approach.update(follower);

        approach.addTelemetry(telemetry);
        if (!driveNote.isEmpty()) telemetry.addLine(driveNote);
        if (result == WallApproach.Result.DONE) {
            telemetry.addLine(String.format("DONE. Thinks the bumper is %.2f in away. Tape it and compare", approach.bumperToWall()));
            telemetry.addLine("Press RB again for another try");
        } else if (result == WallApproach.Result.FAILED) {
            telemetry.addLine("Gave up (4 s timeout or a bad reading). Press RB to try again");
        } else if (driving) {
            telemetry.addLine("Driving... let go of RB to stop");
        }
        telemetry.addLine("B = back to results");
    }

    // best line through tape vs reading, so corrected = scale * reading + offset
    static class Fit {
        int points;
        double scale = 1, offset = 0, worstBefore, worstAfter;

        static Fit of(double[] tape, double[] reading, boolean[] use) {
            Fit f = new Fit();
            double n = 0, sr = 0, st = 0, srr = 0, srt = 0;
            for (int i = 0; i < tape.length; i++) {
                if (!use[i]) continue;
                n++;
                sr += reading[i];
                st += tape[i];
                srr += reading[i] * reading[i];
                srt += reading[i] * tape[i];
            }
            f.points = (int) n;
            if (n >= 2 && n * srr - sr * sr != 0) {
                f.scale = (n * srt - sr * st) / (n * srr - sr * sr);
                f.offset = (st - f.scale * sr) / n;
            }
            for (int i = 0; i < tape.length; i++) {
                if (!use[i]) continue;
                f.worstBefore = Math.max(f.worstBefore, Math.abs(reading[i] - tape[i]));
                f.worstAfter = Math.max(f.worstAfter, Math.abs(f.scale * reading[i] + f.offset - tape[i]));
            }
            return f;
        }

        String verdict() {
            if (worstBefore <= goodEnough) return String.format("formula is good (worst %.2f in off)", worstBefore);
            if (worstAfter <= goodEnough) return String.format("%.2f in off, the fix gets it to %.2f", worstBefore, worstAfter);
            return String.format("%.2f in off even with the fix, check the object/wiring and redo", worstAfter);
        }

        // a real sensor won't need more than this, bigger means the data's bad
        boolean sane() {
            return points >= 3 && Math.abs(scale - 1) < 0.15 && Math.abs(offset) < 3;
        }
    }

    // average of the two sensors = distance to the wall, difference = how crooked we are
    // drives through pedro's manual mode so pedro keeps tracking position
    public static class WallApproach {

        public enum Result { RUNNING, DONE, FAILED }

        static final double sensorSpacing = 16.0;   // in, center to center. measure this
        static final double sensorSetback = 0.0;   // in, how far the sensors sit behind the bumper. measure this

        static final double flowerFromWall = 3.0;   // guess, measure on a real field
        static final double flowerReach = 2.4;   // top ring sticks out this far
        static final double clearance = 1.0;   // don't touch the flower
        public static final double stopDistance = flowerFromWall + flowerReach + clearance;   // bumper to wall

        static final double kpDrive = 0.06;
        static final double kpTurn = 0.015;
        static final double maxDrive = 0.25;   // sensors only update ~10x a second
        static final double maxTurn = 0.15;
        static final double minPower = 0.07;
        static final double kpStrafe = 0.06;
        static final double maxStrafe = 0.2;
        static final double minStrafe = 0.05;
        static final double minTurn = 0.05;
        static final double pushRate = 0.6;   // power per second added while a direction is stuck, low powers can't beat the wheel friction
        static final double maxPush = 0.25;

        static final double distanceTolerance = 0.5;
        static final double angleTolerance = 2.0;
        static final double strafeTolerance = 0.75;
        static final double settleSec = 0.3;   // sensors only update every 100 ms, so wait for a few fresh readings
        static final double maxSkew = 6.0;   // bigger difference than this = one sensor is probably seeing the flower
        static final double timeoutSec = 4.0;

        // paste the numbers from this test's results screen here. corrected = scale * reading + offset
        static final double leftScale = 1.0, leftOffset = 0.0;
        static final double rightScale = 1.0, rightOffset = 0.0;
        private double leftScaleNow = leftScale, leftOffsetNow = leftOffset;
        private double rightScaleNow = rightScale, rightOffsetNow = rightOffset;
        static final double minReading = 1.0;   // in. under this is a loose signal wire (it reads as 0.8), not a real wall

        private final Ultrasonic.HrlvShortRange leftSensor, rightSensor;
        private final ElapsedTime timer = new ElapsedTime();
        private final ElapsedTime sinceUpdate = new ElapsedTime();
        private double drivePush, turnPush, strafePush;
        private final ElapsedTime onTargetFor = new ElapsedTime();
        private double target = stopDistance;

        private double left, right, wallDist, error, angle, drivePower, strafePower, turnPower;
        private boolean useAngle = true;

        public WallApproach(HardwareMap hardwareMap) {
            leftSensor = new Ultrasonic.HrlvShortRange(
                    hardwareMap.get(AnalogInput.class, "ultrasonic_left_signal"),
                    hardwareMap.get(AnalogInput.class, "ultrasonic_left_vcc"));
            rightSensor = new Ultrasonic.HrlvShortRange(
                    hardwareMap.get(AnalogInput.class, "ultrasonic_right_signal"),
                    hardwareMap.get(AnalogInput.class, "ultrasonic_right_vcc"));
        }

        public void start() {
            start(stopDistance);
        }

        public void start(double bumperToWall) {
            target = bumperToWall;
            timer.reset();
            onTargetFor.reset();
            sinceUpdate.reset();
            drivePush = turnPush = strafePush = 0;
        }

        public Result update(Follower follower) {
            return update(follower, 0);
        }

        // sets pedro's manual powers. caller still has to call follower.update()
        // leftError = inches the target is to the robot's left (from pedro, the sensors only see ahead), 0 to skip
        public Result update(Follower follower, double leftError) {
            if (timer.seconds() > timeoutSec) {
                follower.manual(0, 0, 0);
                return Result.FAILED;
            }

            left = leftNow();
            right = rightNow();
            if (Double.isNaN(left) || Double.isNaN(right) || left < minReading || right < minReading) {
                follower.manual(0, 0, 0);
                return Result.FAILED;
            }

            useAngle = Math.abs(left - right) < maxSkew;
            wallDist = useAngle ? (left + right) / 2.0 : Math.max(left, right);
            error = wallDist - sensorSetback - target;
            angle = useAngle ? angleFrom(left, right) : 0;

            boolean onTarget = Math.abs(error) < distanceTolerance && Math.abs(angle) < angleTolerance;
            if (!onTarget) onTargetFor.reset();
            if (onTargetFor.seconds() >= settleSec) {
                follower.manual(0, 0, 0);
                return Result.DONE;
            }

            double dt = Math.min(sinceUpdate.seconds(), 0.1);
            sinceUpdate.reset();
            Velocity v = follower.velocity();
            double heading = follower.pose().heading();
            double forwardSpeed = v.vx * Math.cos(heading) + v.vy * Math.sin(heading);
            double leftSpeed = -v.vx * Math.sin(heading) + v.vy * Math.cos(heading);
            drivePush = push(drivePush, error, distanceTolerance, Math.abs(forwardSpeed) > 1, dt);
            turnPush = push(turnPush, angle, angleTolerance, Math.abs(v.omega) > 0.05, dt);
            strafePush = push(strafePush, leftError, strafeTolerance, Math.abs(leftSpeed) > 1, dt);

            drivePower = atLeast(Range.clip(kpDrive * error, -maxDrive, maxDrive), error, distanceTolerance, minPower + drivePush);
            turnPower = -atLeast(Range.clip(kpTurn * angle, -maxTurn, maxTurn), angle, angleTolerance, minTurn + turnPush);
            strafePower = atLeast(Range.clip(kpStrafe * leftError, -maxStrafe, maxStrafe), leftError, strafeTolerance, minStrafe + strafePush);

            follower.manual(drivePower, strafePower, turnPower);   // forward, left, ccw (robot-centric)
            return Result.RUNNING;
        }

        public void showReadings(Telemetry telemetry) {
            double l = leftNow();
            double r = rightNow();
            telemetry.addData("Left / Right", "%.1f in / %.1f in", l, r);
            telemetry.addData("Angle", "%.1f deg", angleFrom(l, r));
            telemetry.addData("Stops at", "%.1f in from the wall", stopDistance);
        }

        public void addTelemetry(Telemetry telemetry) {
            telemetry.addData("Left / Right", "%.1f in / %.1f in", left, right);
            telemetry.addData("Wall", "%.1f in (%.1f to go)", wallDist, error);
            telemetry.addData("Angle", useAngle ? String.format("%.1f deg", angle) : "ignored (one sensor sees the flower)");
            telemetry.addData("Power", "drive %.2f  strafe %.2f  turn %.2f", drivePower, strafePower, turnPower);
        }

        // grows while that direction is stuck, stays once it gets going, back to 0 once it's there
        static double push(double push, double error, double tolerance, boolean moving, double dt) {
            if (Math.abs(error) <= tolerance) return 0;
            if (!moving) push += pushRate * dt;
            return Math.min(push, maxPush);
        }

        // at least min power while it's outside the tolerance
        static double atLeast(double power, double error, double tolerance, double min) {
            if (Math.abs(error) <= tolerance) return power;
            return Math.copySign(Math.max(Math.abs(power), min), error);
        }

        // for this run only, the test uses it to try a fix before you paste it in
        public void useCorrection(double leftScale, double leftOffset, double rightScale, double rightOffset) {
            leftScaleNow = leftScale;
            leftOffsetNow = leftOffset;
            rightScaleNow = rightScale;
            rightOffsetNow = rightOffset;
        }

        // inches straight from the datasheet formula, no fix
        public double rawLeft() {
            return leftSensor.getDistanceMm() / 25.4;
        }

        public double rawRight() {
            return rightSensor.getDistanceMm() / 25.4;
        }

        public double leftNow() {
            return leftScaleNow * rawLeft() + leftOffsetNow;
        }

        public double rightNow() {
            return rightScaleNow * rawRight() + rightOffsetNow;
        }

        public double supplyVolts() {
            return Math.min(leftSensor.getVccVoltage(), rightSensor.getVccVoltage());
        }

        // bumper to wall from the last reading
        public double bumperToWall() {
            return wallDist - sensorSetback;
        }

        // left reading longer than right = turned ccw
        static double angleFrom(double left, double right) {
            return Math.toDegrees(Math.atan2(left - right, sensorSpacing));
        }
    }
}
