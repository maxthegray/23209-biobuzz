package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.UltrasonicConfig;
import org.firstinspires.ftc.teamcode.logging.FlightRecorder;
import org.firstinspires.ftc.teamcode.mechanisms.WallApproach;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.sensors.BulkReads;

import java.util.List;

// drives up to the wall and stops a set distance away, using the calibrated ultrasonic readings
// (the fix in config/UltrasonicConfig.java). it also squares up to the wall.
//
// INIT:  D-pad up / down changes the stop distance by STEP_IN. the distance is locked once you press START.
// START: HOLD A to drive, letting go stops it right away. hold A again for another try.
//
// distance is from the bumper, which is the sensor faces while UltrasonicConfig.sensorSetbackIn is 0
@TeleOp(name = "Drive to Wall (ultrasonic)", group = "test")
public class DriveToWallTest extends LinearOpMode {

    static final double DEFAULT_TARGET_IN = 7.0;   // stop distance, bumper to wall, before you change it
    static final double STEP_IN = 0.25;   // how much one D-pad press changes it
    static final double MIN_TARGET_IN = 2.0;   // the sensors read down to ~1 in, keep some margin
    static final double MAX_TARGET_IN = 24.0;

    @Override
    public void runOpMode() {
        Follower follower = Constants.create(hardwareMap);
        WallApproach approach = new WallApproach(hardwareMap);   // uses UltrasonicConfig, nothing overrides it here
        List<LynxModule> hubs = BulkReads.manual(hardwareMap);
        FlightRecorder log = FlightRecorder.start(this, follower);   // saves the run to a file on the hub
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        double target = DEFAULT_TARGET_IN;
        boolean lastUp = false, lastDown = false;

        // init: pick the distance. the robot never moves here
        while (opModeInInit()) {
            BulkReads.clear(hubs);
            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
            if (up && !lastUp) target += STEP_IN;
            if (down && !lastDown) target -= STEP_IN;
            lastUp = up;
            lastDown = down;
            target = Range.clip(target, MIN_TARGET_IN, MAX_TARGET_IN);

            telemetry.addLine(String.format("STOP DISTANCE: %.2f in   (D-pad up/down = +/- %.2f)", target, STEP_IN));
            telemetry.addLine("");
            showReadings(approach, target);
            showFix();
            telemetry.addLine("");
            telemetry.addLine("Point the robot at the wall from 1-2 ft");
            telemetry.addLine("Press START, then HOLD A to drive");
            telemetry.update();
            approach.log(log);
            log.put("DriveToWall/targetIn", target);
            log.update();
            sleep(20);
        }

        final double stopAt = target;   // locked from here on
        WallApproach.Result result = null;
        boolean driving = false, lastHeld = false;
        String note = "";

        try {
            while (opModeIsActive()) {
                BulkReads.clear(hubs);
                boolean held = gamepad1.a;
                boolean pressed = held && !lastHeld;
                lastHeld = held;

                if (!held) {
                    if (driving) follower.manual(0, 0, 0);
                    driving = false;
                } else if (pressed) {
                    double l = approach.leftNow(), r = approach.rightNow();
                    if (Double.isNaN(l) || Double.isNaN(r) || l > UltrasonicConfig.tooFarIn || r > UltrasonicConfig.tooFarIn) {
                        result = null;
                        note = "Didn't start: both sensors need the wall within " + (int) UltrasonicConfig.tooFarIn + " in";
                    } else {
                        approach.start(stopAt);
                        result = WallApproach.Result.RUNNING;
                        driving = true;
                        note = "";
                    }
                }

                telemetry.addLine(String.format("DRIVE TO WALL, stops %.2f in away", stopAt));
                if (driving && result == WallApproach.Result.RUNNING) {
                    result = approach.update(follower);
                    approach.addTelemetry(telemetry);
                } else {
                    showReadings(approach, stopAt);
                }
                follower.update();

                if (!note.isEmpty()) telemetry.addLine(note);
                if (result == WallApproach.Result.DONE) {
                    telemetry.addLine(String.format("DONE. Thinks the bumper is %.2f in away. Tape it and compare", approach.bumperToWall()));
                } else if (result == WallApproach.Result.FAILED) {
                    telemetry.addLine("Gave up (6 s timeout or a bad reading). Let go and hold A again");
                } else if (driving) {
                    telemetry.addLine("Driving... let go of A to stop");
                } else {
                    telemetry.addLine("HOLD A to drive");
                }
                showFix();
                telemetry.update();

                approach.log(log);
                log.put("DriveToWall/targetIn", stopAt);
                log.put("DriveToWall/driving", driving);
                log.put("DriveToWall/result", result == null ? "-" : result.toString());
                log.update();
            }
        } finally {
            follower.manual(0, 0, 0);
            if (opModeIsActive()) follower.update();
        }
    }

    // calibrated readings while not driving
    private void showReadings(WallApproach approach, double target) {
        double l = approach.leftNow(), r = approach.rightNow();
        double wall = (l + r) / 2;
        telemetry.addData("Left / Right", "%.2f in / %.2f in", l, r);
        telemetry.addData("Wall", "%.2f in (%.2f to go)", wall, wall - approach.sensorSetback() - target);
    }

    // so you can see on the Driver Hub which fix is being used
    private void showFix() {
        telemetry.addLine(String.format("Fix: left %.4f x + %.3f   right %.4f x + %.3f",
                UltrasonicConfig.leftScale, UltrasonicConfig.leftOffset,
                UltrasonicConfig.rightScale, UltrasonicConfig.rightOffset));
    }
}
