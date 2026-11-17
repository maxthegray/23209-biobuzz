package org.firstinspires.ftc.teamcode.tests.flowerdash;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.logging.FlightRecorder;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.tests.FlowerApproachTest.WallApproach;
import org.firstinspires.ftc.teamcode.tests.Ultrasonic;
import org.firstinspires.ftc.teamcode.tests.UltrasonicCalibration;
import org.firstinspires.ftc.teamcode.tuning.Tunables;

import java.util.List;

// autonomous: after START the robot turns in place on its own until both ultrasonic sensors read the same distance,
// so it ends up square to the wall. no buttons needed.
// readings use the fix in UltrasonicCalibration.java: corrected = scale * raw + offset
// sign convention: left sensor counts negative, right counts positive. difference = right - left.
// right reads farther (difference > 0) = positive turn, left reads farther (difference < 0) = negative turn.
@Configurable
@Autonomous(name = "UltrasonicAlignment", group = "test")
public class UltrasonicAutoAlignment extends LinearOpMode {

    // the public static ones can be changed live in Panels (http://192.168.43.1:8001), see tuning/Tunables.java.
    // the final ones are limits and stay put. turn power is always clipped to maxTurn, whatever the others are
    public static double kp = 0.05;   // turn power per inch of difference. 1 in difference is about 3.6 deg with 16 in spacing
    static final double maxTurn = 0.35;
    public static double minTurn = 0.08;   // starting minimum, the push below adds to it until the robot actually turns
    public static double pushRate = 0.4;   // power per second added while the robot is stuck (not turning)
    public static double maxPush = 0.25;
    public static double stuckOmega = 0.05;   // rad/s, slower than this counts as not turning
    public static double tolerance = 0.3;   // in, close enough to call it the same
    public static double settleSec = 0.3;   // sensors update every 100 ms, wait for a few fresh readings
    static final double timeoutSec = 8.0;   // give up if it still isn't aligned after this long
    static final double maxSkew = 6.0;   // bigger difference than this = one sensor is probably seeing the flower, not the wall
    static final double minReading = 1.0;   // in, under this is a loose signal wire
    static final double maxReading = 30.0;   // in, farther than this the wall probably isn't in front of both sensors

    // remembers the numbers above as "what the code says". has to stay below them
    static {
        Tunables.track(UltrasonicAutoAlignment.class);
    }

    @Override
    public void runOpMode() {
        Follower follower = Constants.create(hardwareMap);
        WallApproach sensors = new WallApproach(hardwareMap);   // just for the calibrated readings
        List<LynxModule> hubs = Ultrasonic.manualBulkReads(hardwareMap);
        FlightRecorder log = FlightRecorder.start(this, follower);   // saves the run to a file on the hub
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        // show live readings during init too, the robot doesn't move until START
        while (opModeInInit()) {
            Ultrasonic.clearBulkCache(hubs);
            showReadings(sensors);
            telemetry.addLine("Press START, the robot will align itself");
            telemetry.update();
            sensors.log(log);
            log.update();
            sleep(20);
        }

        ElapsedTime runTime = new ElapsedTime();
        ElapsedTime alignedFor = new ElapsedTime();
        ElapsedTime loopTimer = new ElapsedTime();
        double push = 0;
        String finalStatus = null;

        try {
            while (opModeIsActive()) {
                Ultrasonic.clearBulkCache(hubs);
                double dt = Math.min(loopTimer.seconds(), 0.1);
                loopTimer.reset();
                double turn = 0;
                double[] r = showReadings(sensors);
                double left = r[0], right = r[1], diff = right - left;   // left negative, right positive
                String status;

                if (finalStatus != null) {
                    status = finalStatus;   // done, just keep showing readings until STOP
                } else {
                    boolean valid = !Double.isNaN(left) && !Double.isNaN(right)
                            && left > minReading && right > minReading
                            && left < maxReading && right < maxReading;

                    if (!valid) {
                        status = "Bad reading, not turning (check wiring / point at the wall)";
                        alignedFor.reset();
                    } else if (Math.abs(diff) > maxSkew) {
                        status = "Sensors too different, one is probably seeing the flower";
                        alignedFor.reset();
                    } else if (Math.abs(diff) <= tolerance) {
                        status = "Settling...";
                        if (alignedFor.seconds() >= settleSec) {
                            finalStatus = String.format("ALIGNED in %.1f s", runTime.seconds());
                            status = finalStatus;
                        }
                    } else {
                        alignedFor.reset();
                        // stuck = add power a little at a time until the wheels beat friction, keep it once it's moving
                        if (Math.abs(follower.velocity().omega) < stuckOmega) push = Math.min(push + pushRate * dt, maxPush);
                        // right farther (diff > 0) = positive turn, left farther (diff < 0) = negative turn
                        turn = Range.clip(kp * diff, -maxTurn, maxTurn);
                        turn = Math.copySign(Math.max(Math.abs(turn), minTurn + push), turn);
                        turn = Range.clip(turn, -maxTurn, maxTurn);
                        status = diff > 0 ? "Right farther, turning +" : "Left farther, turning -";
                    }
                    if (Math.abs(diff) <= tolerance) push = 0;

                    if (finalStatus == null && runTime.seconds() > timeoutSec) {
                        finalStatus = String.format("TIMED OUT after %.0f s, not aligned", timeoutSec);
                        status = finalStatus;
                        turn = 0;
                    }
                }

                follower.manual(0, 0, turn);   // forward, left, ccw (robot-centric)
                follower.update();

                telemetry.addData("Diff (R - L)", "%.2f in (want within %.2f)", diff, tolerance);
                telemetry.addData("Turn power", "%.3f (push %.2f)", turn, push);
                telemetry.addData("Turn speed", "%.2f rad/s", follower.velocity().omega);
                telemetry.addData("Time", "%.1f / %.0f s", runTime.seconds(), timeoutSec);
                telemetry.addLine(status);
                telemetry.update();

                sensors.log(log);
                log.put("Align/diffIn", diff);
                log.put("Align/turnPower", turn);
                log.put("Align/pushPower", push);
                log.put("Align/status", status);
                log.update();
            }
        } finally {
            follower.manual(0, 0, 0);
            if (opModeIsActive()) follower.update();
        }
    }

    // reads each sensor once, shows raw + fix + corrected, returns {left, right} corrected
    private double[] showReadings(WallApproach sensors) {
        double rawL = sensors.rawLeft(), rawR = sensors.rawRight();
        double left = UltrasonicCalibration.left(rawL);
        double right = UltrasonicCalibration.right(rawR);
        telemetry.addData("Left ", "raw %5.2f x %.4f + %.3f = %5.2f in",
                rawL, UltrasonicCalibration.leftScale, UltrasonicCalibration.leftOffset, left);
        telemetry.addData("Right", "raw %5.2f x %.4f + %.3f = %5.2f in",
                rawR, UltrasonicCalibration.rightScale, UltrasonicCalibration.rightOffset, right);
        return new double[]{left, right};
    }
}
