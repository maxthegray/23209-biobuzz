package org.firstinspires.ftc.teamcode.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.UltrasonicConfig;
import org.firstinspires.ftc.teamcode.logging.FlightRecorder;
import org.firstinspires.ftc.teamcode.sensors.UltrasonicSensor;
import org.firstinspires.ftc.teamcode.tuning.Tunables;

// average of the two sensors = distance to the wall, difference = how crooked we are
// drives through pedro's manual mode so pedro keeps tracking position
@Configurable
public class WallApproach {

    public enum Result { RUNNING, DONE, FAILED }

    // sensor spacing/setback, the flower stop distance and the sanity limits are in config/UltrasonicConfig.java
    private double sensorSpacing = UltrasonicConfig.sensorSpacingIn;   // the website can change these for one run
    private double sensorSetback = UltrasonicConfig.sensorSetbackIn;

    // the public static ones can be changed live in Panels (http://192.168.43.1:8001), see tuning/Tunables.java.
    // the final ones are limits and stay put, so a typo in Panels can't send the robot into the wall
    public static double kpDrive = 0.06;
    public static double kpTurn = 0.015;
    static final double maxDrive = 0.25;   // sensors only update ~10x a second
    static final double maxTurn = 0.15;
    public static double minPower = 0.07;
    public static double kpStrafe = 0.06;
    static final double maxStrafe = 0.2;
    public static double minStrafe = 0.05;
    public static double minTurn = 0.05;
    public static double pushRate = 0.6;   // power per second added while a direction is stuck, low powers can't beat the wheel friction
    public static double maxPush = 0.25;
    static final double hardLimit = 0.5;   // no power goes past this whatever the numbers above are

    public static double distanceTolerance = 0.5;
    public static double angleTolerance = 2.0;
    public static double strafeTolerance = 0.75;
    public static double settleSec = 0.3;   // sensors only update every 100 ms, so wait for a few fresh readings
    static final double timeoutSec = 4.0;

    // remembers the numbers above as "what the code says". has to stay below them
    static {
        Tunables.track(WallApproach.class);
    }

    // the fix lives in config/UltrasonicConfig.java, edit it there. corrected = scale * reading + offset
    public static double[] pastedFix() {
        return UltrasonicConfig.values();
    }

    // false = use UltrasonicConfig as it is right now (so a change in Panels shows up straight away).
    // true = useCorrection() gave this run its own fix
    private boolean ownFix = false;
    private double leftScaleNow, leftOffsetNow, rightScaleNow, rightOffsetNow;

    private final UltrasonicSensor leftSensor, rightSensor;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime sinceUpdate = new ElapsedTime();
    private double drivePush, turnPush, strafePush;
    private final ElapsedTime onTargetFor = new ElapsedTime();
    private double target = UltrasonicConfig.stopDistanceIn;

    private double left, right, wallDist, error, angle, drivePower, strafePower, turnPower;
    private double lastRawLeft = Double.NaN, lastRawRight = Double.NaN;
    private boolean useAngle = true;

    public WallApproach(HardwareMap hardwareMap) {
        leftSensor = UltrasonicSensor.left(hardwareMap);
        rightSensor = UltrasonicSensor.right(hardwareMap);
    }

    public void start() {
        start(UltrasonicConfig.stopDistanceIn);
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
        if (Double.isNaN(left) || Double.isNaN(right) || left < UltrasonicConfig.minReadingIn || right < UltrasonicConfig.minReadingIn) {
            follower.manual(0, 0, 0);
            return Result.FAILED;
        }

        useAngle = Math.abs(left - right) < UltrasonicConfig.maxSkewIn;
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
        // the minimums + push can go past the max above on purpose (to get unstuck), but never past this
        drivePower = Range.clip(drivePower, -hardLimit, hardLimit);
        turnPower = Range.clip(turnPower, -hardLimit, hardLimit);
        strafePower = Range.clip(strafePower, -hardLimit, hardLimit);

        follower.manual(drivePower, strafePower, turnPower);   // forward, left, ccw (robot-centric)
        return Result.RUNNING;
    }

    public void showReadings(Telemetry telemetry) {
        double l = leftNow();
        double r = rightNow();
        telemetry.addData("Left / Right", "%.1f in / %.1f in", l, r);
        telemetry.addData("Angle", "%.1f deg", angleFrom(l, r));
        telemetry.addData("Stops at", "%.1f in from the wall", UltrasonicConfig.stopDistanceIn);
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
        ownFix = true;
    }

    // back to the fix in config/UltrasonicConfig.java
    public void useCalibration() {
        ownFix = false;
    }

    private double fixLeft(double raw) {
        return ownFix ? leftScaleNow * raw + leftOffsetNow : UltrasonicConfig.left(raw);
    }

    private double fixRight(double raw) {
        return ownFix ? rightScaleNow * raw + rightOffsetNow : UltrasonicConfig.right(raw);
    }

    // inches straight from the datasheet formula, no fix
    public double rawLeft() {
        return lastRawLeft = leftSensor.getDistanceIn();
    }

    public double rawRight() {
        return lastRawRight = rightSensor.getDistanceIn();
    }

    // puts the last readings and what update() last decided into the flight recorder. doesn't read the hub again
    public void log(FlightRecorder log) {
        log.put("Ultrasonic/leftSignalVolts", leftSensor.lastSignalVoltage());
        log.put("Ultrasonic/leftVccVolts", leftSensor.lastVccVoltage());
        log.put("Ultrasonic/rightSignalVolts", rightSensor.lastSignalVoltage());
        log.put("Ultrasonic/rightVccVolts", rightSensor.lastVccVoltage());
        log.put("Ultrasonic/leftRawIn", lastRawLeft);
        log.put("Ultrasonic/rightRawIn", lastRawRight);
        log.put("Ultrasonic/leftIn", fixLeft(lastRawLeft));
        log.put("Ultrasonic/rightIn", fixRight(lastRawRight));
        log.put("WallApproach/wallIn", wallDist);
        log.put("WallApproach/toGoIn", error);
        log.put("WallApproach/angleDeg", angle);
        log.put("WallApproach/drivePower", drivePower);
        log.put("WallApproach/strafePower", strafePower);
        log.put("WallApproach/turnPower", turnPower);
    }

    public double leftNow() {
        return fixLeft(rawLeft());
    }

    public double rightNow() {
        return fixRight(rawRight());
    }

    // from the last reading, doesn't read the hub again
    public double supplyVolts() {
        return Math.min(leftSensor.lastVccVoltage(), rightSensor.lastVccVoltage());
    }

    // {left signal, left vcc, right signal, right vcc} in volts from the last reading, for the log
    public double[] lastVolts() {
        return new double[]{leftSensor.lastSignalVoltage(), leftSensor.lastVccVoltage(),
                rightSensor.lastSignalVoltage(), rightSensor.lastVccVoltage()};
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

    // bumper to wall from the last reading
    public double bumperToWall() {
        return wallDist - sensorSetback;
    }

    // left reading longer than right = turned ccw
    double angleFrom(double left, double right) {
        return Math.toDegrees(Math.atan2(left - right, sensorSpacing));
    }
}
