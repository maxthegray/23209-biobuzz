package org.firstinspires.ftc.teamcode.vision.ultrasonic;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// flower approach used by path demo and the flower approach test, so the tuned numbers live in one place
// average of the two sensors = distance to the wall, difference = how crooked we are
// the sensors lag (2 Hz filter) and move in ~0.3 in steps, so it measures while stopped, then moves that much using pedro's odometry
public class WallApproach {

    public enum Result { RUNNING, DONE, FAILED }

    public static final double defaultSpacing = 7.5;   // in, center to center, measured 2026-11-09 (the website can change it for a run)
    public static final double defaultSetback = 0.0;   // in, how far the sensors sit behind the bumper. measure this
    private double sensorSpacing = defaultSpacing;
    private double sensorSetback = defaultSetback;

    static final double flowerFromWall = 3.0;   // guess, measure on a real field
    static final double flowerReach = 2.4;   // top ring sticks out this far
    static final double clearance = 1.0;   // don't touch the flower
    public static final double stopDistance = flowerFromWall + flowerReach + clearance;   // bumper to wall

    static final double settleSec = 0.4;   // after stopping, let the sensor's filter catch up
    static final double averageSec = 0.2;   // then average this long, the flicker between steps averages out
    static final double maxStep = 4.0;   // in, longest single move before it measures again
    static final double farStep = 0.8;   // further than maxStep away = go 80% of it, then measure again

    // these steer the moves, and they use odometry so there's no lag
    static final double kpDrive = 0.08;   // power per inch
    static final double kpTurn = 0.02;   // power per degree
    static final double kpStrafe = 0.09;
    static final double maxDrive = 0.3, maxTurn = 0.2, maxStrafe = 0.35;   // sideways takes more power to get going
    static final double minPower = 0.07, minTurn = 0.05, minStrafe = 0.1;
    static final double pushRate = 0.6;   // power per second added while a direction is stuck, low powers can't beat the wheel friction
    static final double maxPush = 0.25;
    static final double arrivedIn = 0.3, arrivedDeg = 1.0, arrivedSide = 0.3;
    static final double moveTimeoutSec = 1.5;

    static final double distanceTolerance = 0.75;   // the sensor is only good to about half an inch, chasing less than that just wastes time
    static final double angleTolerance = 2.5;
    static final double strafeTolerance = 0.75;
    static final double maxSkew = 2.5;   // in. 7.5 in apart, so 2.5 in is already ~18 deg crooked. more = a sensor is seeing the flower
    static final double timeoutSec = 6.0;
    static final double trustMove = 2.5;   // in. a straight move this short lands where it measured, no need to stop and measure again
    static final int maxShifts = 3;   // sideways tries at finding the flower (3.75, 1.9, 0.9 in)

    // TUNED: paste the numbers from the Flower Approach Test results here. corrected = scale * reading + offset
    public static final double leftScale = 1.3844, leftOffset = 1.471;
    public static final double rightScale = 1.2481, rightOffset = 2.256;

    public static double[] pastedFix() {
        return new double[]{leftScale, leftOffset, rightScale, rightOffset};
    }
    private double leftScaleNow = leftScale, leftOffsetNow = leftOffset;
    private double rightScaleNow = rightScale, rightOffsetNow = rightOffset;
    static final double minReading = 1.0;   // in. under this is a loose signal wire (it reads as 0.8), not a real wall

    private final HrlvShortRange leftSensor, rightSensor;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime sinceUpdate = new ElapsedTime();
    private final ElapsedTime phaseTimer = new ElapsedTime();
    private double drivePush, turnPush, strafePush;
    private double target = stopDistance;
    private boolean measuring = true;
    private double sumLeft, sumRight;
    private int count;
    private Pose moveTarget;
    private double flowerShift, nextShift;
    private int shifts;
    private boolean checkAgain;
    private double plannedForward;

    private double left, right, wallDist, error, angle, drivePower, strafePower, turnPower;
    private boolean useAngle = true;

    public WallApproach(HardwareMap hardwareMap) {
        leftSensor = new HrlvShortRange(
                hardwareMap.get(AnalogInput.class, "ultrasonic_left_signal"),
                hardwareMap.get(AnalogInput.class, "ultrasonic_left_vcc"));
        rightSensor = new HrlvShortRange(
                hardwareMap.get(AnalogInput.class, "ultrasonic_right_signal"),
                hardwareMap.get(AnalogInput.class, "ultrasonic_right_vcc"));
    }

    public void start() {
        start(stopDistance);
    }

    public void start(double bumperToWall) {
        target = bumperToWall;
        flowerShift = 0;
        nextShift = sensorSpacing / 2;
        shifts = 0;
        timer.reset();
        sinceUpdate.reset();
        startMeasuring(null);
    }

    private void startMeasuring(Follower follower) {
        if (follower != null) follower.manual(0, 0, 0);
        measuring = true;
        phaseTimer.reset();
        sumLeft = sumRight = 0;
        count = 0;
        drivePower = strafePower = turnPower = 0;
    }

    private Result finish(Follower follower, Result result) {
        follower.manual(0, 0, 0);
        drivePower = strafePower = turnPower = 0;
        return result;
    }

    public Result update(Follower follower) {
        return update(follower, 0);
    }

    // sets pedro's manual powers. caller still has to call follower.update()
    // leftError = inches the target is to the robot's left (from pedro, the sensors only see ahead), 0 to skip
    public Result update(Follower follower, double leftError) {
        if (timer.seconds() > timeoutSec) return finish(follower, Result.FAILED);
        left = leftNow();
        right = rightNow();
        if (Double.isNaN(left) || Double.isNaN(right) || left < minReading || right < minReading) return finish(follower, Result.FAILED);
        double dt = Math.min(sinceUpdate.seconds(), 0.1);
        sinceUpdate.reset();

        if (measuring) {
            follower.manual(0, 0, 0);
            if (phaseTimer.seconds() < settleSec) return Result.RUNNING;
            sumLeft += left;
            sumRight += right;
            count++;
            if (phaseTimer.seconds() < settleSec + averageSec) return Result.RUNNING;

            double l = sumLeft / count, r = sumRight / count;
            useAngle = Math.abs(l - r) < maxSkew;
            // one sensor reading way shorter = that sensor is on the flower, so the robot is off to the side
            // slide toward that sensor, half as far each time. first try puts the flower from one sensor to the middle
            boolean offCenter = !useAngle && shifts < maxShifts;
            if (offCenter) {
                flowerShift += l < r ? nextShift : -nextShift;
                nextShift /= 2;
                shifts++;
            }
            wallDist = useAngle ? (l + r) / 2.0 : Math.max(l, r);
            error = wallDist - sensorSetback - target;
            angle = useAngle ? angleFrom(l, r) : 0;
            if (!offCenter && Math.abs(error) < distanceTolerance && Math.abs(angle) < angleTolerance) return finish(follower, Result.DONE);

            // forward by what it measured (most of it when far, closer than the target = only backs up), turn out the angle, sideways from pedro plus the flower shift
            double forward = Math.abs(error) < distanceTolerance ? 0 : Math.abs(error) <= maxStep ? error : Math.copySign(Math.max(maxStep, farStep * Math.abs(error)), error);
            double side = Math.abs(leftError + flowerShift) > strafeTolerance ? leftError + flowerShift : 0;
            double turn = Math.abs(angle) < angleTolerance ? 0 : -Math.toRadians(angle);   // left reading longer = turned ccw, so turn back cw
            Pose p = follower.pose();
            double h = p.heading();
            moveTarget = new Pose(p.x() + forward * Math.cos(h) - side * Math.sin(h), p.y() + forward * Math.sin(h) + side * Math.cos(h), h + turn);
            checkAgain = offCenter || turn != 0 || Math.abs(forward) > trustMove;
            plannedForward = forward;
            measuring = false;
            phaseTimer.reset();
            drivePush = turnPush = strafePush = 0;
            return Result.RUNNING;
        }

        // moving: steer by odometry toward the spot worked out from the last measurement
        Pose p = follower.pose();
        double h = p.heading(), dx = moveTarget.x() - p.x(), dy = moveTarget.y() - p.y();
        double forwardLeft = dx * Math.cos(h) + dy * Math.sin(h);
        double sideLeft = -dx * Math.sin(h) + dy * Math.cos(h);
        double turnLeft = Math.toDegrees(Math.atan2(Math.sin(moveTarget.heading() - h), Math.cos(moveTarget.heading() - h)));
        boolean arrived = Math.abs(forwardLeft) < arrivedIn && Math.abs(sideLeft) < arrivedSide && Math.abs(turnLeft) < arrivedDeg;
        if (arrived && !checkAgain) {
            wallDist -= plannedForward;
            error -= plannedForward;
            return finish(follower, Result.DONE);
        }
        if (arrived || phaseTimer.seconds() > moveTimeoutSec) {
            startMeasuring(follower);
            return Result.RUNNING;
        }

        Velocity v = follower.velocity();
        double forwardSpeed = v.vx * Math.cos(h) + v.vy * Math.sin(h);
        double leftSpeed = -v.vx * Math.sin(h) + v.vy * Math.cos(h);
        drivePush = push(drivePush, forwardLeft, arrivedIn, Math.abs(forwardSpeed), 1, dt);
        turnPush = push(turnPush, turnLeft, arrivedDeg, Math.abs(Math.toDegrees(v.omega)), 3, dt);
        strafePush = push(strafePush, sideLeft, arrivedSide, Math.abs(leftSpeed), 1, dt);

        drivePower = atLeast(Range.clip(kpDrive * forwardLeft, -maxDrive, maxDrive), forwardLeft, arrivedIn, minPower + drivePush);
        turnPower = atLeast(Range.clip(kpTurn * turnLeft, -maxTurn, maxTurn), turnLeft, arrivedDeg, minTurn + turnPush);
        strafePower = atLeast(Range.clip(kpStrafe * sideLeft, -maxStrafe, maxStrafe), sideLeft, arrivedSide, minStrafe + strafePush);
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
        telemetry.addData("Flower shift", "%+.1f in (+ = left)", flowerShift);
        telemetry.addData("Power", "drive %.2f  strafe %.2f  turn %.2f", drivePower, strafePower, turnPower);
        telemetry.addData("Doing", measuring ? "measuring (standing still)" : "moving by odometry");
    }

    // grows while that direction is stuck, eases off if it starts going too fast, back to 0 once it's there
    static double push(double push, double error, double tolerance, double speed, double stuckSpeed, double dt) {
        if (Math.abs(error) <= tolerance) return 0;
        if (speed < stuckSpeed) push += pushRate * dt;
        else if (speed > 5 * stuckSpeed) push -= pushRate * dt;
        return Range.clip(push, 0, maxPush);
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

    // the website's measurements, for this run only
    public void setGeometry(double spacing, double setback) {
        sensorSpacing = spacing;
        sensorSetback = setback;
    }

    public double sensorSetback() {
        return sensorSetback;
    }

    // left, right, wall, toGo, angle, drivePower, strafePower, turnPower, bumper from the last update()
    public double[] snapshot() {
        return new double[]{left, right, wallDist, error, angle, drivePower, strafePower, turnPower, bumperToWall()};
    }

    // inches it slid left (+) or right (-) of pedro's spot to get on the flower
    public double flowerShift() {
        return flowerShift;
    }

    // bumper to wall from the last reading
    public double bumperToWall() {
        return wallDist - sensorSetback;
    }

    // left reading longer than right = turned ccw
    public double angleFrom(double left, double right) {
        return Math.toDegrees(Math.atan2(left - right, sensorSpacing));
    }
}
