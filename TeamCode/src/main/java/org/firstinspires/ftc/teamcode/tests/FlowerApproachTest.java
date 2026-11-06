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

// last few inches of the flower approach using the two front ultrasonics
// test: robot 1-2 ft from a wall facing it, START squares up and drives in. keep a hand on stop
@TeleOp(name = "Flower Approach Test (ultrasonic)", group = "test")
public class FlowerApproachTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Follower follower = Constants.create(hardwareMap);
        WallApproach approach = new WallApproach(hardwareMap);

        while (opModeInInit()) {
            approach.showReadings(telemetry);
            telemetry.update();
            sleep(50);
        }
        if (isStopRequested()) return;

        approach.start();
        WallApproach.Result result = WallApproach.Result.RUNNING;
        while (opModeIsActive() && result == WallApproach.Result.RUNNING) {
            result = approach.update(follower);
            follower.update();
            approach.addTelemetry(telemetry);
            telemetry.update();
        }

        follower.manual(0, 0, 0);
        follower.update();
        while (opModeIsActive()) {
            telemetry.addData("Result", result == WallApproach.Result.DONE ? "Done" : "Failed (timeout or bad readings)");
            telemetry.update();
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

            left = leftSensor.getDistanceMm() / 25.4;
            right = rightSensor.getDistanceMm() / 25.4;
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
            double l = leftSensor.getDistanceMm() / 25.4;
            double r = rightSensor.getDistanceMm() / 25.4;
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
