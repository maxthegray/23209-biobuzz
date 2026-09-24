package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import org.opencv.core.Scalar;

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

    // 640x480 because the SDK only has AprilTag calibration for a few sizes and 320x240 isn't one
    private static final int eyeballWidth = 640;
    private static final int eyeballHeight = 480;

    // all the pixel numbers below were picked at 320x240, this rescales them
    private static final double zoominess = eyeballWidth / 320.0;

    // global shutter cams are light hungry, start bright and walk it down
    private static final boolean wantFrozenEyeball = true;
    private static final long squintMs = 4;            // exposure
    private static final int brightnessJuice = 10;            // gain, high gain = grainy speckles that look yellow

    // sample had these at 15, way too big for a 3 inch ball, it glued them to the floor
    private static final int smooshSize = 5;            // blur
    private static final int puffSize = 5;            // dilate
    private static final int shrinkSize = 5;            // erode

    // the built in YELLOW lets in anything slightly warm, this is pickier
    // numbers are brightness, redness, blueness. real yellow = bright, kinda red, not blue at all
    private static final Scalar yellowLow = new Scalar(40, 135, 0);   // 40 so the shady half of the ball still counts
    private static final Scalar yellowHigh = new Scalar(255, 180, 100);


    private static double kP = 0.0040 / zoominess;
    private static double kD = 0.0006 / zoominess;

    // PD for now. I on a camera loop just makes it overshoot. unstickPower covers what I would do
    private static double kI = 0.0;

    private static final double closeEnoughPx = 8 * zoominess;
    private static final double unstickPower = 0.10;        // below this the wheels just buzz
    private static final double spinSpeedLimit = 0.45;
    private static final double windupLeash = 0.15;

    // bigger ball on screen = closer ball
    private static final double snuggleRadiusPx = 55 * zoominess;
    private static final double kpScoot = 0.006 / zoominess;
    private static final double scootSpeedLimit = 0.35;
    private static final double aimedEnoughPx = 40 * zoominess;

    // false if the camera refused manual exposure
    private boolean eyeballFrozen = false;

    @Override
    public void runOpMode() {

        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeftMotor");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        final double camCenterX = eyeballWidth / 2.0;
        final double camCenterY = eyeballHeight / 2.0;

        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(new ColorRange(ColorSpace.YCrCb, yellowLow, yellowHigh))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())            // was a centre box, which hid the side balls
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(android.graphics.Color.rgb(255, 255, 0))
                .setBlurSize(smooshSize)
                .setDilateSize(puffSize)
                .setErodeSize(shrinkSize)
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
                .setCameraResolution(new Size(eyeballWidth, eyeballHeight))
                .setCamera(arducam)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);

        if (wantFrozenEyeball) {
            freezeTheEyeball(portal, squintMs, brightnessJuice);
        }

        double  lastSeenX = 0;

        double windup = 0;
        double errorLastTime = 0;
        boolean haveOldError = false;

        double spinFlip = 1.0;
        boolean scootAllowed = false;
        boolean wasLive = false;

        boolean wasX = false, wasY = false, wasA = false;
        boolean wasUp = false, wasDown = false, wasRight = false, wasLeft = false;

        ElapsedTime stopwatch = new ElapsedTime();
        ElapsedTime blabberTimer = new ElapsedTime();
        ElapsedTime sinceNewPic = new ElapsedTime();

        double lastPicX = -1, lastPicR = -1;
        double heldSpin = 0, heldScoot = 0;

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
            if (gamepad1.y && !wasY) { scootAllowed = !scootAllowed; }
            if (gamepad1.a && !wasA) { windup = 0; haveOldError = false; }

            if (gamepad1.dpad_up && !wasUp) kP += 0.0005 / zoominess;
            if (gamepad1.dpad_down && !wasDown) kP = Math.max(0, kP - 0.0005 / zoominess);
            if (gamepad1.dpad_right && !wasRight) kD += 0.0002 / zoominess;
            if (gamepad1.dpad_left && !wasLeft) kD = Math.max(0, kD - 0.0002 / zoominess);

            wasX = gamepad1.x; wasY = gamepad1.y; wasA = gamepad1.a;
            wasUp = gamepad1.dpad_up; wasDown = gamepad1.dpad_down;
            wasRight = gamepad1.dpad_right; wasLeft = gamepad1.dpad_left;

            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

            // one ball for now, the camera is the truth, so the biggest yellow blob is the ball
            ColorBlobLocatorProcessor.Blob b = blobs.isEmpty() ? null : blobs.get(0);

            double turn = 0, scoot = 0, errorX = 0, radius = 0;

            if (b != null) {
                Circle ring = b.getCircle();
                lastSeenX = ring.getX();
                radius = ring.getRadius();

                errorX = lastSeenX - camCenterX;            // + means ball is to the right

                // loop runs way faster than the camera, same numbers = same old picture
                boolean freshPic = lastSeenX != lastPicX || radius != lastPicR;

                if (!freshPic) {
                    turn = heldSpin;
                    scoot = heldScoot;
                } else {
                    lastPicX = lastSeenX;
                    lastPicR = radius;

                    // time between pictures, not between loops, or kD spikes
                    double picDt = sinceNewPic.seconds();
                    sinceNewPic.reset();
                    if (picDt <= 0 || picDt > 0.25) {
                        picDt = 0.033;
                    }

                    if (Math.abs(errorX) < closeEnoughPx) {
                        windup = 0;
                        turn = 0;
                        haveOldError = false;            // old error goes stale while we sit here
                    } else {
                        windup += errorX * picDt;
                        double windupCap = windupLeash / Math.max(kI, 1e-9);
                        windup = Range.clip(windup, -windupCap, windupCap);

                        double errorSlope = haveOldError ? (errorX - errorLastTime) / picDt : 0;
                        errorLastTime = errorX;
                        haveOldError = true;

                        turn = (kP * errorX) + (kI * windup) + (kD * errorSlope);

                        // anything gentler than this doesn't actually turn the robot
                        if (Math.abs(turn) < unstickPower) {
                            turn = Math.signum(turn) * unstickPower;
                        }
                        turn = Range.clip(turn, -spinSpeedLimit, spinSpeedLimit);
                    }

                    turn *= spinFlip;

                    // don't drive at it until we're roughly pointed at it
                    if (scootAllowed && Math.abs(errorX) < aimedEnoughPx) {
                        double stillTooFar = snuggleRadiusPx - radius;
                        scoot = Range.clip(stillTooFar * kpScoot, 0, scootSpeedLimit);
                    }

                    heldSpin = turn;
                    heldScoot = scoot;
                }

            } else {
                lastPicX = -1;            // so a ball we find again counts as a new picture
                heldSpin = 0;
                heldScoot = 0;
                windup = 0;
                haveOldError = false;
            }

            boolean wheelsHot = started && (!needBumperHeld || gamepad1.right_bumper);

            if (wheelsHot) {
                // front left and back left are +turn, other 2 are -turn
                frontLeft.setPower(Range.clip(scoot + turn, -1, 1));
                frontRight.setPower(Range.clip(scoot - turn, -1, 1));
                backLeft.setPower(Range.clip(scoot + turn, -1, 1));
                backRight.setPower(Range.clip(scoot - turn, -1, 1));
            } else {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
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
                    telemetry.addData("Doing", "turn=%+.3f        scoot=%+.3f", turn, scoot);
                } else {
                    telemetry.addLine("no yellow out there");
                }

                telemetry.addLine();
                telemetry.addData("kP / kD", "%.5f / %.5f        (dpad)", kP, kD);
                telemetry.addData("Spin dir", "%s        (X flips it)", spinFlip > 0 ? "normal" : "FLIPPED");
                telemetry.addData("Scooting", "%s        (Y toggles)", scootAllowed ? "ON" : "off");
                telemetry.addData("Eyeball", eyeballFrozen ? "locked" : "AUTO - colour will wander");
                telemetry.update();
            }
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        portal.close();
    }

    // pin exposure so the picture stops changing every time we turn toward a window
    private void freezeTheEyeball(VisionPortal portal, long openForMs, int gainAmount) {
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
            ExposureControl squint = portal.getCameraControl(ExposureControl.class);
            if (squint != null) {
                if (squint.getMode() != ExposureControl.Mode.Manual) {
                    squint.setMode(ExposureControl.Mode.Manual);
                    sleep(50);
                }
                squint.setExposure(openForMs, TimeUnit.MILLISECONDS);
                sleep(20);
            }
            // scootAllowed = != ScootAllowed;
            // gamepad.wasY

            GainControl brightness = portal.getCameraControl(GainControl.class);
            if (brightness != null) {
                brightness.setGain(gainAmount);
                sleep(20);
            }
            eyeballFrozen = true;
        } catch (Exception uncooperativeCamera) {
            eyeballFrozen = false;
            telemetry.log().add("camera won't do manual exposure: " + uncooperativeCamera.getMessage());
        }
    }
}