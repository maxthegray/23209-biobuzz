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
import org.firstinspires.ftc.teamcode.tests.flowerdash.CalibrationServer;
import org.firstinspires.ftc.teamcode.tests.flowerdash.CalibrationSession;
import org.firstinspires.ftc.teamcode.tests.flowerdash.CalibrationSession.Phase;
import org.firstinspires.ftc.teamcode.tests.flowerdash.CalibrationSession.Stage;
import org.firstinspires.ftc.teamcode.tests.flowerdash.Command;
import org.json.JSONArray;

import java.util.concurrent.ConcurrentLinkedQueue;

// step by step ultrasonic check, the driver hub tells you what to do
// part 1: checks the distance formula against a tape measure, the robot never moves
// part 2: drives up to the object, but only while you hold RB (or hold the button on the website)
//
// there's also a website, open the address shown on the driver hub from a laptop on the robot's wifi.
// it does everything the gamepad does, plus saves and plots every reading and lets you download CSVs.
// it runs on its own port (8765+), separate from the robot controller page (8080) and Pedro's Panels (8001)
@TeleOp(name = "Flower Approach Test (ultrasonic)", group = "test")
public class FlowerApproachTest extends LinearOpMode {

    private static final double[] tapeDistances = {4, 6, 8, 12, 18, 24};   // in, sensor face to object. extra points near where flowers stop
    private static final double settleSec = 1.0;   // datasheet: the output goes through a 2 Hz filter, so give it time to catch up
    private static final double recordSec = 1.5;
    private static final double goodEnough = 0.25;   // in, worst miss we still call accurate
    private static final double jumpy = 0.3;   // in, readings wobbling more than this while still = something's wrong
    private static final double tooFar = 30;   // in, won't drive unless both sensors see something closer than this
    private static final double deadManSec = 0.4;   // website drive button must keep pinging, no ping for this long = stop

    private final ConcurrentLinkedQueue<Command> commands = new ConcurrentLinkedQueue<>();
    private Follower follower;
    private WallApproach approach;
    private CalibrationSession session;

    private boolean lastA, lastB, lastX, lastHeld;
    private boolean driving = false;
    private boolean usingFix = false;
    private double lastBeat = -1e9;
    private WallApproach.Result result = null;
    private String driveNote = "";
    private double cachedLeft, cachedRight;

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        approach = new WallApproach(hardwareMap);
        session = new CalibrationSession(tapeDistances, settleSec, recordSec, goodEnough, jumpy,
                WallApproach.stopDistance, tooFar, WallApproach.defaultSpacing, WallApproach.defaultSetback);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        CalibrationServer server = CalibrationServer.startOnFreePort(hardwareMap.appContext, session, commands);
        String web = server == null
                ? "Website couldn't start (ports 8765-8770 busy). Gamepad still works"
                : "Website: http://192.168.43.1:" + server.port() + "  (Wi-Fi Direct phone: 192.168.49.1)";

        ElapsedTime clock = new ElapsedTime();
        try {
            // runs through init too: the sensor steps never move the robot, only the drive check needs START
            while (!isStopRequested()) {
                boolean canDrive = opModeIsActive();
                double t = clock.seconds();
                session.setStarted(canDrive);

                gamepadCommands();
                for (Command c; (c = commands.poll()) != null; ) handle(c, t);

                sample(t);
                if (canDrive) driveLogic(t);

                telemetry.addLine(web);
                if (!canDrive) telemetry.addLine("Press START to enable the drive check (robot never moves in init)");
                Stage stage = session.stage();
                if (stage == Stage.CALIBRATE) calibrate();
                else if (stage == Stage.RESULTS) results();
                else drive(canDrive);

                if (canDrive) follower.update();
                telemetry.update();
                if (stage != Stage.DRIVE) sleep(20);   // the drive check runs full speed like before, the tuning depends on it
            }
        } finally {
            if (server != null) server.stop();
            session.close();
            follower.manual(0, 0, 0);
            if (opModeIsActive()) follower.update();
        }
    }

    // the gamepad makes the same commands the website does
    private void gamepadCommands() {
        boolean a = gamepad1.a && !lastA, b = gamepad1.b && !lastB, x = gamepad1.x && !lastX;
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        Stage stage = session.stage();
        if (stage == Stage.CALIBRATE) {
            if (a) commands.add(new Command("RECORD"));
            else if (b) commands.add(new Command("REDO"));
            else if (x) commands.add(new Command("DONE"));
        } else if (stage == Stage.RESULTS) {
            if (a) commands.add(new Command("GO_DRIVE"));
            else if (b) commands.add(new Command("RESTART"));
        } else if (b) {
            commands.add(new Command("BACK"));
        }
    }

    private void handle(Command c, double t) {
        String why = session.validate(c.name);
        if (why != null) {
            session.setMessage(why);
            return;
        }
        switch (c.name) {
            case "RECORD": session.startRecord(); break;
            case "CANCEL": session.cancel(); break;
            case "REDO": session.redo(); break;
            case "DONE": session.finishCalibration(); break;
            case "GOTO_STEP": session.gotoStep(c.args.optInt("step", 0)); break;
            case "RESTART":
                stopDriving();
                session.restart();
                break;
            case "SET_TAPES": {
                JSONArray arr = c.args.optJSONArray("tapes");
                if (arr == null) break;
                double[] tapes = new double[arr.length()];
                for (int i = 0; i < tapes.length; i++) tapes[i] = arr.optDouble(i, 0);
                session.setTapes(tapes);
                break;
            }
            case "SET_GEOMETRY": {
                double spacing = c.args.optDouble("spacing", Double.NaN), setback = c.args.optDouble("setback", Double.NaN);
                if (Double.isNaN(spacing) || Double.isNaN(setback) || spacing < 1 || spacing > 40 || setback < 0 || setback > 20) {
                    session.setMessage("Measurements look wrong (spacing 1-40 in, setback 0-20 in)");
                    break;
                }
                approach.setGeometry(spacing, setback);
                session.setGeometry(spacing, setback);
                break;
            }
            case "GO_DRIVE": {
                boolean sane = session.fitsSane();
                usingFix = c.args.optBoolean("fix", true) && sane;
                double[] k = usingFix ? session.correction() : new double[]{1, 0, 1, 0};
                approach.useCorrection(k[0], k[1], k[2], k[3]);
                result = null;
                driveNote = "";
                session.enterDrive(usingFix, k);
                break;
            }
            case "BACK":
                stopDriving();
                session.backToResults();
                break;
            case "DRIVE_HEARTBEAT": lastBeat = t; break;
            case "ESTOP":
                stopDriving();
                session.setEstop(true);
                break;
            case "RESET": session.setEstop(false); break;
            case "LOG_TRIAL":
                session.addTrial(c.args.optDouble("measured", Double.NaN), approach.bumperToWall(), String.valueOf(result));
                break;
        }
    }

    private void stopDriving() {
        if (driving) follower.manual(0, 0, 0);
        driving = false;
    }

    // reads the sensors exactly once per loop. the sensor class keeps a short history that moves on every call,
    // so extra calls would change how much it smooths
    private void sample(double t) {
        double l, r;
        boolean driveStage = session.stage() == Stage.DRIVE;
        if (!driveStage) {
            l = approach.rawLeft();
            r = approach.rawRight();
        } else if (driving && result == WallApproach.Result.RUNNING) {
            double[] s = approach.snapshot();   // approach.update already read them this loop
            l = s[0];
            r = s[1];
        } else {
            l = approach.leftNow();
            r = approach.rightNow();
        }
        cachedLeft = l;
        cachedRight = r;
        session.tick(t, l, r, approach.supplyVolts(), driveStage);
    }

    private void calibrate() {
        double tape = session.tapeNow();
        int step = session.step();
        Phase phase = session.phase();
        if (phase == Phase.SETTLING) {
            telemetry.addLine(String.format("Settling at %.1f in... don't touch anything", tape));
        } else if (phase == Phase.RECORDING) {
            telemetry.addLine(String.format("Recording %.1f in... don't touch anything", tape));
        } else {
            telemetry.addLine(String.format("FORMULA CHECK  step %d of %d", step + 1, session.stepCount()));
            telemetry.addLine(String.format("1. Put the object %.1f in from the front of the round sensor parts", tape));
            telemetry.addLine("2. Tape BOTH sensors, nudge it till both read the same");
            telemetry.addLine("3. Press A, then don't touch anything for 2.5 s");
        }
        String m = session.message();
        if (!m.isEmpty()) telemetry.addLine(m);
        telemetry.addLine("");
        telemetry.addLine(String.format("Live  left %5.2f in   right %5.2f in", cachedLeft, cachedRight));
        telemetry.addLine(String.format("Sensor supply %.2f V (should be about 3.3)", approach.supplyVolts()));
        telemetry.addLine("");
        telemetry.addLine("A = record   B = redo last step   X = done, show results");
    }

    private void results() {
        Fit[] f = session.fits();
        Fit leftFit = f[0], rightFit = f[1];
        telemetry.addLine("FORMULA CHECK RESULTS (inches)");
        telemetry.addLine("See the website for the table and graphs");
        if (leftFit.points < 3) {
            telemetry.addLine("Need at least 3 distances to judge the formula");
            telemetry.addLine("A = drive check with no fix   B = redo formula check");
            return;
        }
        telemetry.addLine("Left:  " + leftFit.verdict());
        telemetry.addLine("Right: " + rightFit.verdict());
        if (session.maxWobble() > jumpy) {
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
    }

    // only drives while RB or the website's button is held, letting go stops it right away
    private void driveLogic(double t) {
        if (session.stage() != Stage.DRIVE) {
            stopDriving();
            return;
        }
        boolean webHeld = t - lastBeat < deadManSec;
        boolean held = !session.isEstop() && (gamepad1.right_bumper || webHeld);
        boolean pressed = held && !lastHeld;
        lastHeld = held;

        if (!held) {
            stopDriving();
        } else if (pressed) {
            double l = cachedLeft, r = cachedRight;
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
        session.setDrive(driving, result == null ? "" : result.name(), driveNote, approach.snapshot());
    }

    private void drive(boolean canDrive) {
        telemetry.addLine("DRIVE CHECK" + (usingFix ? " (using this run's fix)" : " (no fix applied)"));
        if (!canDrive) {
            telemetry.addLine("Press START on the Driver Hub to drive");
            telemetry.addLine("B = back to results");
            return;
        }
        telemetry.addLine("Aim at the object from 1-2 ft, a little crooked is fine");
        telemetry.addLine("HOLD RB (or the website button) to drive, let go to stop instantly");
        telemetry.addLine(String.format("Should stop with the bumper %.1f in from it", WallApproach.stopDistance));
        if (approach.sensorSetback() == 0) telemetry.addLine("(sensor setback is 0, so 'bumper' = sensor face for now)");
        if (session.isEstop()) telemetry.addLine("E-STOP latched from the website, press Reset there");
        telemetry.addLine("");

        approach.addTelemetry(telemetry);
        if (!driveNote.isEmpty()) telemetry.addLine(driveNote);
        if (result == WallApproach.Result.DONE) {
            telemetry.addLine(String.format("DONE. Thinks the bumper is %.2f in away. Tape it and compare", approach.bumperToWall()));
            telemetry.addLine("Hold again for another try");
        } else if (result == WallApproach.Result.FAILED) {
            telemetry.addLine("Gave up (4 s timeout or a bad reading). Hold again to try again");
        } else if (driving) {
            telemetry.addLine("Driving... let go to stop");
        }
        telemetry.addLine("B = back to results");
    }

    // best line through tape vs reading, so corrected = scale * reading + offset
    public static class Fit {
        public int points;
        public double scale = 1, offset = 0, worstBefore, worstAfter;
        public double r2 = 1, rmse;   // of the fitted line, rmse in inches

        public static Fit of(double[] tape, double[] reading, boolean[] use) {
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
            double ssRes = 0, ssTot = 0, mean = n > 0 ? st / n : 0;
            for (int i = 0; i < tape.length; i++) {
                if (!use[i]) continue;
                f.worstBefore = Math.max(f.worstBefore, Math.abs(reading[i] - tape[i]));
                f.worstAfter = Math.max(f.worstAfter, Math.abs(f.scale * reading[i] + f.offset - tape[i]));
                double miss = f.scale * reading[i] + f.offset - tape[i];
                ssRes += miss * miss;
                ssTot += (tape[i] - mean) * (tape[i] - mean);
            }
            if (n > 0) f.rmse = Math.sqrt(ssRes / n);
            if (ssTot > 0) f.r2 = 1 - ssRes / ssTot;
            return f;
        }

        public String verdict() {
            if (worstBefore <= goodEnough) return String.format("formula is good (worst %.2f in off)", worstBefore);
            if (worstAfter <= goodEnough) return String.format("%.2f in off, the fix gets it to %.2f", worstBefore, worstAfter);
            return String.format("%.2f in off even with the fix, check the object/wiring and redo", worstAfter);
        }

        // a real sensor won't need more than this, bigger means the data's bad
        public boolean sane() {
            return points >= 3 && Math.abs(scale - 1) < 0.15 && Math.abs(offset) < 3;
        }
    }

    // average of the two sensors = distance to the wall, difference = how crooked we are
    // drives through pedro's manual mode so pedro keeps tracking position
    public static class WallApproach {

        public enum Result { RUNNING, DONE, FAILED }

        public static final double defaultSpacing = 16.0;   // in, center to center. measure this (the website can change it for a run)
        public static final double defaultSetback = 0.0;   // in, how far the sensors sit behind the bumper. measure this
        private double sensorSpacing = defaultSpacing;
        private double sensorSetback = defaultSetback;

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
        static final double leftScale = 1.2318, leftOffset = 2.144;
        static final double rightScale = 1.2270, rightOffset = 2.487;
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
}
