package org.firstinspires.ftc.teamcode.opmodes.vision;

import android.graphics.Canvas;
import android.graphics.Paint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
import org.firstinspires.ftc.teamcode.config.BallVisionConfig;
import org.firstinspires.ftc.teamcode.logging.FlightRecorder;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumMotors;
import org.opencv.core.Mat;

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
 * Set forwardAllowed true to also drive forward (same as pressing Y in GoToBallPIDNew).
 * Camera runs during INIT so you can aim it before anything moves.
 */

@TeleOp(name = "Auto Go to Closest Ball", group = "teaching")
public class AutoGoToClosestBall extends LinearOpMode {

    // same default as GoToBallPIDNew with Y off
    private static final boolean forwardAllowed = true;

    // camera, colour, PID and pollen-size numbers are in config/java (shared with "Go to Ball PID (new)")

    private boolean exposureLocked = false;

    @Override
    public void runOpMode() {

        MecanumMotors motors = new MecanumMotors(hardwareMap);   // the 4 drive motors, directions and brake already set

        final double camCenterX = BallVisionConfig.cameraWidth / 2.0;

        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(new ColorRange(ColorSpace.YCrCb, BallVisionConfig.yellowLow, BallVisionConfig.yellowHigh))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(android.graphics.Color.rgb(255, 255, 0))
                .setBlurSize(BallVisionConfig.blurSize)
                .setDilateSize(BallVisionConfig.dilateSize)
                .setErodeSize(BallVisionConfig.erodeSize)
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
                .setCameraResolution(new Size(BallVisionConfig.cameraWidth, BallVisionConfig.cameraHeight))
                .setCamera(arducam)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);

        if (BallVisionConfig.manualExposure) {
            lockExposure(portal, BallVisionConfig.exposureMs, BallVisionConfig.gain);
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
        FlightRecorder log = FlightRecorder.start(this);   // saves the run to a file on the hub, see logging/FlightRecorder.java

        double lastPicX = -1, lastPicR = -1;
        double heldSpin = 0, heldForward = 0;

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

            double turn = 0, forward = 0, errorX = 0, radius = 0;

            if (b != null) {
                Circle ring = b.getCircle();
                lastSeenX = ring.getX();
                radius = ring.getRadius();

                errorX = lastSeenX - camCenterX;

                boolean freshPic = lastSeenX != lastPicX || radius != lastPicR;

                if (!freshPic) {
                    turn = heldSpin;
                    forward = heldForward;
                } else {
                    lastPicX = lastSeenX;
                    lastPicR = radius;

                    double picDt = sinceNewPic.seconds();
                    sinceNewPic.reset();
                    if (picDt <= 0 || picDt > 0.25) {
                        picDt = 0.033;
                    }

                    if (Math.abs(errorX) < BallVisionConfig.closeEnoughPx) {
                        windup = 0;
                        turn = 0;
                        haveOldError = false;
                    } else {
                        windup += errorX * picDt;
                        double windupCap = BallVisionConfig.windupLeash / Math.max(BallVisionConfig.kI, 1e-9);
                        windup = Range.clip(windup, -windupCap, windupCap);

                        double errorSlope = haveOldError ? (errorX - errorLastTime) / picDt : 0;
                        errorLastTime = errorX;
                        haveOldError = true;

                        turn = (BallVisionConfig.kP * errorX) + (BallVisionConfig.kI * windup) + (BallVisionConfig.kD * errorSlope);

                        if (Math.abs(turn) < BallVisionConfig.unstickPower) {
                            turn = Math.signum(turn) * BallVisionConfig.unstickPower;
                        }
                        turn = Range.clip(turn, -BallVisionConfig.spinSpeedLimit, BallVisionConfig.spinSpeedLimit);
                    }

                    turn *= spinFlip;

                    if (forwardAllowed && Math.abs(errorX) < BallVisionConfig.aimedEnoughPx) {
                        double stillTooFar = BallVisionConfig.closeRadiusPx - radius;
                        forward = Range.clip(stillTooFar * BallVisionConfig.kpForward, 0, BallVisionConfig.forwardSpeedLimit);
                    }

                    heldSpin = turn;
                    heldForward = forward;
                }

            } else {
                lastPicX = -1;
                heldSpin = 0;
                heldForward = 0;
                windup = 0;
                haveOldError = false;
            }

            boolean wheelsHot = started && b != null;

            if (wheelsHot) {
                motors.frontLeft.setPower(Range.clip(forward + turn, -1, 1));
                motors.frontRight.setPower(Range.clip(forward - turn, -1, 1));
                motors.backLeft.setPower(Range.clip(forward + turn, -1, 1));
                motors.backRight.setPower(Range.clip(forward - turn, -1, 1));
            } else {
                motors.stop();
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
                    telemetry.addData("Doing", "turn=%+.3f        scoot=%+.3f", turn, forward);
                    telemetry.addData("Target lock", targetSelector.isLocked() ? "sticky" : "new pick");
                } else {
                    telemetry.addLine("no yellow out there");
                    if (!blobs.isEmpty()) {
                        telemetry.addLine("(blobs seen but none look like single pollen)");
                    }
                }

                telemetry.addLine();
                telemetry.addData("kP / kD", "%.5f / %.5f", BallVisionConfig.kP, BallVisionConfig.kD);
                telemetry.addData("Scooting", "%s", forwardAllowed ? "ON" : "off");
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
            if (r < BallVisionConfig.minPollenRadiusPx || r > BallVisionConfig.maxPollenRadiusPx) {
                return false;
            }
            if (area < BallVisionConfig.minPollenAreaPx || area > BallVisionConfig.mergedPollenAreaPx) {
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
                double bestDist = BallVisionConfig.targetStickinessPx + 1;
                for (ColorBlobLocatorProcessor.Blob blob : blobs) {
                    if (!looksLikeSinglePollen(blob)) {
                        continue;
                    }
                    Circle c = blob.getCircle();
                    double dx = c.getX() - lockX;
                    double dy = c.getY() - lockY;
                    double dist = Math.hypot(dx, dy);
                    if (dist <= BallVisionConfig.targetStickinessPx && dist < bestDist) {
                        bestDist = dist;
                        nearLock = blob;
                    }
                }
                if (nearLock != null && bestRadius <= nearLock.getCircle().getRadius() * (1 + BallVisionConfig.closerSwitchFraction)) {
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
