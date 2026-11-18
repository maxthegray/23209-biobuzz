package org.firstinspires.ftc.teamcode.opmodes.vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.teamcode.config.BallVisionConfig;
import org.firstinspires.ftc.teamcode.logging.FlightRecorder;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumMotors;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * Find a yellow Pollen ball and turn to face it.
 *
 * RIGHT BUMPER (held) - wheels only move while you hold it
 * X                                                                            - flip turn direction if it turns away from the ball
 * Y                                                                            - also drive forward (leave off while tuning the turn)
 * A                                                                            - reset the controller if it gets confused
 * dpad up/down                                - more / less kP
 * dpad right/left                    - more / less kD
 *
 * Camera runs during INIT so you can aim it before anything moves.
 */

@TeleOp(name = "Go to Ball PID (new)", group = "teaching")
public class GoToBallPIDNew extends LinearOpMode {
    // false = no controller needed, just goes on start
    private static final boolean needBumperHeld = true;

    // camera, colour and PID numbers are in config/java (shared with "Auto Go to Closest Ball")

    // false if the camera refused manual exposure
    private boolean exposureLocked = false;

    @Override
    public void runOpMode() {

        MecanumMotors motors = new MecanumMotors(hardwareMap);   // the 4 drive motors, directions and brake already set


        final double camCenterX = BallVisionConfig.cameraWidth / 2.0;
        final double camCenterY = BallVisionConfig.cameraHeight / 2.0;

        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(new ColorRange(ColorSpace.YCrCb, BallVisionConfig.yellowLow, BallVisionConfig.yellowHigh))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())            // was a centre box, which hid the side balls
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(android.graphics.Color.rgb(255, 255, 0))
                .setBlurSize(BallVisionConfig.blurSize)
                .setDilateSize(BallVisionConfig.dilateSize)
                .setErodeSize(BallVisionConfig.erodeSize)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        // grab whatever webcam is in the config, so the name doesn't matter
        List<WebcamName> webcams = hardwareMap.getAll(WebcamName.class);
        if (webcams.isEmpty()) {
            telemetry.addLine("NO WEBCAM IN THE ROBOT CONFIG");
            telemetry.addLine("plug in the arducam, Configure Robot > Scan > Save, then restart");
            telemetry.update();
            waitForStart();
            return;
        }
        WebcamName arducam = webcams.get(0);
        telemetry.log().add("using camera: " + hardwareMap.getNamesOf(arducam));

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(BallVisionConfig.cameraWidth, BallVisionConfig.cameraHeight))
                .setCamera(arducam)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);

        if (BallVisionConfig.manualExposure) {
            lockExposure(portal, BallVisionConfig.exposureMs, BallVisionConfig.gain);
        }

        double  lastSeenX = 0;

        double windup = 0;
        double errorLastTime = 0;
        boolean haveOldError = false;

        double spinFlip = 1.0;
        boolean forwardAllowed = false;
        boolean wasLive = false;

        boolean wasX = false, wasY = false, wasA = false;
        boolean wasUp = false, wasDown = false, wasRight = false, wasLeft = false;

        ElapsedTime stopwatch = new ElapsedTime();
        ElapsedTime blabberTimer = new ElapsedTime();
        ElapsedTime sinceNewPic = new ElapsedTime();
        FlightRecorder log = FlightRecorder.start(this);   // saves the run to a file on the hub, see logging/FlightRecorder.java

        double lastPicX = -1, lastPicR = -1;
        double heldSpin = 0, heldForward = 0;

        // loops during INIT too so you can aim the camera before it's allowed to move
        while (opModeIsActive() || opModeInInit()) {

            boolean started = opModeIsActive();

            // don't carry INIT windup into the first moving frame
            if (started && !wasLive) {
                windup = 0;
                haveOldError = false;
            }
            wasLive = started;

            double dt = stopwatch.seconds();
            stopwatch.reset();
            if (dt <= 0 || dt > 0.25) {
                dt = 0.02;            // a dropped frame shouldn't spike the derivative
            }

            if (gamepad1.x && !wasX) { spinFlip = -spinFlip; windup = 0; }
            if (gamepad1.y && !wasY) { forwardAllowed = !forwardAllowed; }
            if (gamepad1.a && !wasA) { windup = 0; haveOldError = false; }

            if (gamepad1.dpad_up && !wasUp) BallVisionConfig.kP += 0.0005 / BallVisionConfig.pixelScale;
            if (gamepad1.dpad_down && !wasDown) BallVisionConfig.kP = Math.max(0, BallVisionConfig.kP - 0.0005 / BallVisionConfig.pixelScale);
            if (gamepad1.dpad_right && !wasRight) BallVisionConfig.kD += 0.0002 / BallVisionConfig.pixelScale;
            if (gamepad1.dpad_left && !wasLeft) BallVisionConfig.kD = Math.max(0, BallVisionConfig.kD - 0.0002 / BallVisionConfig.pixelScale);

            wasX = gamepad1.x; wasY = gamepad1.y; wasA = gamepad1.a;
            wasUp = gamepad1.dpad_up; wasDown = gamepad1.dpad_down;
            wasRight = gamepad1.dpad_right; wasLeft = gamepad1.dpad_left;

            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

            // one ball for now, the camera is the truth, so the biggest yellow blob is the ball
            ColorBlobLocatorProcessor.Blob b = blobs.isEmpty() ? null : blobs.get(0);

            double turn = 0, forward = 0, errorX = 0, radius = 0;

            if (b != null) {
                Circle ring = b.getCircle();
                lastSeenX = ring.getX();
                radius = ring.getRadius();

                errorX = lastSeenX - camCenterX;            // + means ball is to the right

                // loop runs way faster than the camera, same numbers = same old picture
                boolean freshPic = lastSeenX != lastPicX || radius != lastPicR;

                if (!freshPic) {
                    turn = heldSpin;
                    forward = heldForward;
                } else {
                    lastPicX = lastSeenX;
                    lastPicR = radius;

                    // time between pictures, not between loops, or kD spikes
                    double picDt = sinceNewPic.seconds();
                    sinceNewPic.reset();
                    if (picDt <= 0 || picDt > 0.25) {
                        picDt = 0.033;
                    }

                    if (Math.abs(errorX) < BallVisionConfig.closeEnoughPx) {
                        windup = 0;
                        turn = 0;
                        haveOldError = false;            // old error goes stale while we sit here
                    } else {
                        windup += errorX * picDt;
                        double windupCap = BallVisionConfig.windupLeash / Math.max(BallVisionConfig.kI, 1e-9);
                        windup = Range.clip(windup, -windupCap, windupCap);

                        double errorSlope = haveOldError ? (errorX - errorLastTime) / picDt : 0;
                        errorLastTime = errorX;
                        haveOldError = true;

                        turn = (BallVisionConfig.kP * errorX) + (BallVisionConfig.kI * windup) + (BallVisionConfig.kD * errorSlope);

                        // anything gentler than this doesn't actually turn the robot
                        if (Math.abs(turn) < BallVisionConfig.unstickPower) {
                            turn = Math.signum(turn) * BallVisionConfig.unstickPower;
                        }
                        turn = Range.clip(turn, -BallVisionConfig.spinSpeedLimit, BallVisionConfig.spinSpeedLimit);
                    }

                    turn *= spinFlip;

                    // don't drive at it until we're roughly pointed at it
                    if (forwardAllowed && Math.abs(errorX) < BallVisionConfig.aimedEnoughPx) {
                        double stillTooFar = BallVisionConfig.closeRadiusPx - radius;
                        forward = Range.clip(stillTooFar * BallVisionConfig.kpForward, 0, BallVisionConfig.forwardSpeedLimit);
                    }

                    heldSpin = turn;
                    heldForward = forward;
                }

            } else {
                lastPicX = -1;            // so a ball we find again counts as a new picture
                heldSpin = 0;
                heldForward = 0;
                windup = 0;
                haveOldError = false;
            }

            boolean wheelsHot = started && (!needBumperHeld || gamepad1.right_bumper);

            if (wheelsHot) {
                // front left and back left are +turn, other 2 are -turn
                motors.frontLeft.setPower(Range.clip(forward + turn, -1, 1));
                motors.frontRight.setPower(Range.clip(forward - turn, -1, 1));
                motors.backLeft.setPower(Range.clip(forward + turn, -1, 1));
                motors.backRight.setPower(Range.clip(forward - turn, -1, 1));
            } else {
                motors.stop();
                windup = 0;                                            // or it kicks the moment you grab the bumper
                haveOldError = false;
            }

            // no sleep() in the loop, it ran at 10Hz before and overshot everything
            if (blabberTimer.milliseconds() > 100) {
                blabberTimer.reset();

                telemetry.addData("State", !started ? "INIT - aim the camera"
                        : wheelsHot ? "** WHEELS HOT **" : "safe (hold RB to go)");
                telemetry.addData("Loop", "%.0f Hz", 1.0 / dt);

                telemetry.addData("Yellow blobs", "%d", blobs.size());

                if (b != null) {
                    telemetry.addData("Ball", "x=%.0f        off by %+.0f px        r=%.0f",
                            lastSeenX, errorX, radius);
                    telemetry.addData("Doing", "turn=%+.3f        scoot=%+.3f", turn, forward);
                } else {
                    telemetry.addLine("no yellow out there");
                }

                telemetry.addLine();
                telemetry.addData("kP / kD", "%.5f / %.5f        (dpad)", BallVisionConfig.kP, BallVisionConfig.kD);
                telemetry.addData("Spin dir", "%s        (X flips it)", spinFlip > 0 ? "normal" : "FLIPPED");
                telemetry.addData("Scooting", "%s        (Y toggles)", forwardAllowed ? "ON" : "off");
                telemetry.addData("Exposure", exposureLocked ? "locked" : "AUTO - colour will wander");
                telemetry.update();
            }

            log.put("Ball/seen", b != null);
            log.put("Ball/errorPx", errorX);
            log.put("Ball/radiusPx", radius);
            log.put("Ball/turn", turn);
            log.put("Ball/scoot", forward);
            log.put("Ball/wheelsHot", wheelsHot);
            log.update();
        }

        motors.stop();
        portal.close();
    }

    // pin exposure so the picture stops changing every time we turn toward a window
    private void lockExposure(VisionPortal portal, long openForMs, int gainAmount) {
        if (portal == null) {
            return;
        }

        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("waiting for the camera...");
            telemetry.update();
            while (!isStopRequested()
                    && portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                sleep(20);
            }
        }

        if (isStopRequested()) {
            return;
        }

        // lots of USB cams just refuse manual exposure, don't die over it
        try {
            ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
            if (exposure != null) {
                if (exposure.getMode() != ExposureControl.Mode.Manual) {
                    exposure.setMode(ExposureControl.Mode.Manual);
                    sleep(50);
                }
                exposure.setExposure(openForMs, TimeUnit.MILLISECONDS);
                sleep(20);
            }
            // gamepad.wasY

            GainControl brightness = portal.getCameraControl(GainControl.class);
            if (brightness != null) {
                brightness.setGain(gainAmount);
                sleep(20);
            }
            exposureLocked = true;
        } catch (Exception uncooperativeCamera) {
            exposureLocked = false;
            telemetry.log().add("camera won't do manual exposure: " + uncooperativeCamera.getMessage());
        }
    }
}