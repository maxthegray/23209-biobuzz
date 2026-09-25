package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.graphics.Paint;
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
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * Same yellow-ball detection and PD movement as GoToBallPIDNew, without a gamepad:
 * wheels run after Start when a ball is seen; stay put when none is seen.
 *
 * Target selection (separate from color thresholding):
 * - Among blobs that look like one POLLEN (~2.8 in dia), pick largest circle radius (closest).
 * - Stick to that ball unless it vanishes or another is clearly closer (>15% larger radius).
 * - Blobs bigger than one pollen (merged pair / wrong object) are ignored.
 *
 * Magenta overlay marks the chosen target on the camera stream.
 *
 * Set scootAllowed true to also drive forward (same as pressing Y in GoToBallPIDNew).
 * Camera runs during INIT so you can aim it before anything moves.
 */

@TeleOp(name = "Auto Go to Closest Ball", group = "teaching")
public class AutoGoToClosestBall extends LinearOpMode {

    // same default as GoToBallPIDNew with Y off
    private static final boolean scootAllowed = true;

    private static final int eyeballWidth = 640;
    private static final int eyeballHeight = 480;

    private static final double zoominess = eyeballWidth / 320.0;

    private static final boolean wantFrozenEyeball = true;
    private static final long squintMs = 4;
    private static final int brightnessJuice = 10;

    private static final int smooshSize = 5;
    private static final int puffSize = 5;
    private static final int shrinkSize = 5;

    private static final Scalar yellowLow = new Scalar(40, 135, 0);
    private static final Scalar yellowHigh = new Scalar(255, 180, 100);

    private static double kP = 0.0040 / zoominess;
    private static double kD = 0.0006 / zoominess;
    private static double kI = 0.0;

    private static final double closeEnoughPx = 8 * zoominess;
    private static final double unstickPower = 0.10;
    private static final double spinSpeedLimit = 0.45;
    private static final double windupLeash = 0.15;

    private static final double snuggleRadiusPx = 55 * zoominess;
    private static final double kpScoot = 0.006 / zoominess;
    private static final double scootSpeedLimit = 0.35;
    private static final double aimedEnoughPx = 40 * zoominess;

    // BIOBUZZ POLLEN: ~2.8 in diameter (+/- 0.1 in per game manual / AndyMark am-5851).
    // NECTAR is ~3.6 in and not yellow; size gates here are for single pollen only.
    private static final double pollenDiameterIn = 2.8;
    private static final double pollenDiameterTolIn = 0.1;

    // snuggleRadiusPx is the fitted-circle size when one pollen is "close" in our tuning.
    private static final double maxPollenRadiusPx = snuggleRadiusPx
            * (1.0 + pollenDiameterTolIn / pollenDiameterIn) * 1.08;
    private static final double minPollenRadiusPx = 6 * zoominess;
    private static final double maxPollenAreaPx = Math.PI * maxPollenRadiusPx * maxPollenRadiusPx;
    private static final double minPollenAreaPx = Math.PI * minPollenRadiusPx * minPollenRadiusPx;
    // Two touching 2.8 in balls ≈ ~2× one ball's area in the mask.
    private static final double mergedPollenAreaPx = maxPollenAreaPx * 1.85;

    private static final double targetStickinessPx = 40 * zoominess;
    private static final double closerSwitchFraction = 0.15;

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

        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(new ColorRange(ColorSpace.YCrCb, yellowLow, yellowHigh))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(android.graphics.Color.rgb(255, 255, 0))
                .setBlurSize(smooshSize)
                .setDilateSize(puffSize)
                .setErodeSize(shrinkSize)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

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

        PollenTargetSelector targetSelector = new PollenTargetSelector();
        TargetHighlight highlight = new TargetHighlight(targetSelector);

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .addProcessor(highlight)
                .setCameraResolution(new Size(eyeballWidth, eyeballHeight))
                .setCamera(arducam)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);

        if (wantFrozenEyeball) {
            freezeTheEyeball(portal, squintMs, brightnessJuice);
        }

        double lastSeenX = 0;

        double windup = 0;
        double errorLastTime = 0;
        boolean haveOldError = false;

        double spinFlip = 1.0;
        boolean wasLive = false;

        ElapsedTime stopwatch = new ElapsedTime();
        ElapsedTime blabberTimer = new ElapsedTime();
        ElapsedTime sinceNewPic = new ElapsedTime();

        double lastPicX = -1, lastPicR = -1;
        double heldSpin = 0, heldScoot = 0;

        while (opModeIsActive() || opModeInInit()) {

            boolean started = opModeIsActive();

            if (started && !wasLive) {
                windup = 0;
                haveOldError = false;
                targetSelector.reset();
            }
            wasLive = started;

            double dt = stopwatch.seconds();
            stopwatch.reset();
            if (dt <= 0 || dt > 0.25) {
                dt = 0.02;
            }

            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

            ColorBlobLocatorProcessor.Blob b = targetSelector.pick(blobs);

            double turn = 0, scoot = 0, errorX = 0, radius = 0;

            if (b != null) {
                Circle ring = b.getCircle();
                lastSeenX = ring.getX();
                radius = ring.getRadius();

                errorX = lastSeenX - camCenterX;

                boolean freshPic = lastSeenX != lastPicX || radius != lastPicR;

                if (!freshPic) {
                    turn = heldSpin;
                    scoot = heldScoot;
                } else {
                    lastPicX = lastSeenX;
                    lastPicR = radius;

                    double picDt = sinceNewPic.seconds();
                    sinceNewPic.reset();
                    if (picDt <= 0 || picDt > 0.25) {
                        picDt = 0.033;
                    }

                    if (Math.abs(errorX) < closeEnoughPx) {
                        windup = 0;
                        turn = 0;
                        haveOldError = false;
                    } else {
                        windup += errorX * picDt;
                        double windupCap = windupLeash / Math.max(kI, 1e-9);
                        windup = Range.clip(windup, -windupCap, windupCap);

                        double errorSlope = haveOldError ? (errorX - errorLastTime) / picDt : 0;
                        errorLastTime = errorX;
                        haveOldError = true;

                        turn = (kP * errorX) + (kI * windup) + (kD * errorSlope);

                        if (Math.abs(turn) < unstickPower) {
                            turn = Math.signum(turn) * unstickPower;
                        }
                        turn = Range.clip(turn, -spinSpeedLimit, spinSpeedLimit);
                    }

                    turn *= spinFlip;

                    if (scootAllowed && Math.abs(errorX) < aimedEnoughPx) {
                        double stillTooFar = snuggleRadiusPx - radius;
                        scoot = Range.clip(stillTooFar * kpScoot, 0, scootSpeedLimit);
                    }

                    heldSpin = turn;
                    heldScoot = scoot;
                }

            } else {
                lastPicX = -1;
                heldSpin = 0;
                heldScoot = 0;
                windup = 0;
                haveOldError = false;
            }

            boolean wheelsHot = started && b != null;

            if (wheelsHot) {
                frontLeft.setPower(Range.clip(scoot + turn, -1, 1));
                frontRight.setPower(Range.clip(scoot - turn, -1, 1));
                backLeft.setPower(Range.clip(scoot + turn, -1, 1));
                backRight.setPower(Range.clip(scoot - turn, -1, 1));
            } else {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                if (!started) {
                    windup = 0;
                    haveOldError = false;
                }
            }

            if (blabberTimer.milliseconds() > 100) {
                blabberTimer.reset();

                telemetry.addData("State", !started ? "INIT - aim the camera"
                        : b != null ? "TRACKING" : "NO BALL (holding still)");
                telemetry.addData("Loop", "%.0f Hz", 1.0 / dt);
                telemetry.addData("Yellow blobs", "%d", blobs.size());

                if (b != null) {
                    telemetry.addData("Ball", "x=%.0f        off by %+.0f px        r=%.0f",
                            lastSeenX, errorX, radius);
                    telemetry.addData("Doing", "turn=%+.3f        scoot=%+.3f", turn, scoot);
                    telemetry.addData("Target lock", targetSelector.isLocked() ? "sticky" : "new pick");
                } else {
                    telemetry.addLine("no yellow out there");
                    if (!blobs.isEmpty()) {
                        telemetry.addLine("(blobs seen but none look like single pollen)");
                    }
                }

                telemetry.addLine();
                telemetry.addData("kP / kD", "%.5f / %.5f", kP, kD);
                telemetry.addData("Scooting", "%s", scootAllowed ? "ON" : "off");
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

    /**
     * Chooses one pollen blob: closest by fitted radius, with sticky lock and size filtering
     * based on ~2.8 in BIOBUZZ pollen (reject merged pairs that look like one huge blob).
     */
    static final class PollenTargetSelector {

        private boolean locked = false;
        private double lockX = 0;
        private double lockY = 0;

        private ColorBlobLocatorProcessor.Blob lastChosen = null;

        void reset() {
            locked = false;
            lastChosen = null;
        }

        boolean isLocked() {
            return locked;
        }

        ColorBlobLocatorProcessor.Blob getLastChosen() {
            return lastChosen;
        }

        static boolean looksLikeSinglePollen(ColorBlobLocatorProcessor.Blob blob) {
            if (blob == null) {
                return false;
            }
            double r = blob.getCircle().getRadius();
            double area = blob.getContourArea();
            if (r < minPollenRadiusPx || r > maxPollenRadiusPx) {
                return false;
            }
            if (area < minPollenAreaPx || area > mergedPollenAreaPx) {
                return false;
            }
            return true;
        }

        ColorBlobLocatorProcessor.Blob pick(List<ColorBlobLocatorProcessor.Blob> blobs) {
            if (blobs.isEmpty()) {
                locked = false;
                lastChosen = null;
                return null;
            }

            ColorBlobLocatorProcessor.Blob closest = null;
            double bestRadius = 0;
            for (ColorBlobLocatorProcessor.Blob blob : blobs) {
                if (!looksLikeSinglePollen(blob)) {
                    continue;
                }
                double r = blob.getCircle().getRadius();
                if (r > bestRadius) {
                    bestRadius = r;
                    closest = blob;
                }
            }

            if (locked) {
                ColorBlobLocatorProcessor.Blob nearLock = null;
                double bestDist = targetStickinessPx + 1;
                for (ColorBlobLocatorProcessor.Blob blob : blobs) {
                    if (!looksLikeSinglePollen(blob)) {
                        continue;
                    }
                    Circle c = blob.getCircle();
                    double dx = c.getX() - lockX;
                    double dy = c.getY() - lockY;
                    double dist = Math.hypot(dx, dy);
                    if (dist <= targetStickinessPx && dist < bestDist) {
                        bestDist = dist;
                        nearLock = blob;
                    }
                }
                if (nearLock != null && bestRadius <= nearLock.getCircle().getRadius() * (1 + closerSwitchFraction)) {
                    lastChosen = nearLock;
                    Circle ring = nearLock.getCircle();
                    lockX = ring.getX();
                    lockY = ring.getY();
                    return nearLock;
                }
            }

            if (closest == null) {
                locked = false;
                lastChosen = null;
                return null;
            }

            locked = true;
            lastChosen = closest;
            Circle ring = closest.getCircle();
            lockX = ring.getX();
            lockY = ring.getY();
            return closest;
        }
    }

    /** Draw-only: highlights the same blob as {@link PollenTargetSelector#pick}. */
    static final class TargetHighlight implements VisionProcessor {

        private final PollenTargetSelector targetSelector;
        private Paint highlightPaint;

        TargetHighlight(PollenTargetSelector targetSelector) {
            this.targetSelector = targetSelector;
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            highlightPaint = new Paint();
            highlightPaint.setColor(android.graphics.Color.rgb(255, 0, 255));
            highlightPaint.setStyle(Paint.Style.STROKE);
            highlightPaint.setAntiAlias(true);
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            ColorBlobLocatorProcessor.Blob chosen = targetSelector.getLastChosen();
            if (chosen == null) {
                return null;
            }
            Circle ring = chosen.getCircle();
            return new double[]{ring.getX(), ring.getY(), ring.getRadius()};
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            if (!(userContext instanceof double[]) || highlightPaint == null) {
                return;
            }
            double[] t = (double[]) userContext;
            highlightPaint.setStrokeWidth(scaleCanvasDensity * 6);

            float cx = (float) (t[0] * scaleBmpPxToCanvasPx);
            float cy = (float) (t[1] * scaleBmpPxToCanvasPx);
            float r = (float) (t[2] * scaleBmpPxToCanvasPx);

            canvas.drawCircle(cx, cy, r, highlightPaint);
            float arm = r + 12 * scaleBmpPxToCanvasPx;
            canvas.drawLine(cx - arm, cy, cx + arm, cy, highlightPaint);
            canvas.drawLine(cx, cy - arm, cx, cy + arm, highlightPaint);
        }
    }
}
