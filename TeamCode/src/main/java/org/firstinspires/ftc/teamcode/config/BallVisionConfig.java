package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.tuning.Tunables;
import org.opencv.core.Scalar;

// numbers for finding a yellow pollen ball with the webcam and turning / driving to it.
// shared by "Go to Ball PID (new)" and "Auto Go to Closest Ball": tune once, both use it.
// the old playful names are in the comments ("zoominess", "squint"...) so you can still find them.
@Configurable
public final class BallVisionConfig {

    // ---- camera ----
    // 640x480 because the SDK only has AprilTag calibration for a few sizes and 320x240 isn't one
    public static final int cameraWidth = 640;   // "eyeballWidth"
    public static final int cameraHeight = 480;   // "eyeballHeight"
    // all the pixel numbers here were picked at 320x240, this rescales them
    public static final double pixelScale = cameraWidth / 320.0;   // "zoominess"

    // global shutter cams are light hungry, start bright and walk it down
    public static final boolean manualExposure = true;   // "wantFrozenEyeball", lock exposure + gain so the colour doesn't wander
    public static final long exposureMs = 4;   // "squintMs"
    public static final int gain = 10;   // "brightnessJuice", high gain = grainy speckles that look yellow

    // ---- PID gains (tunable in Panels, http://192.168.43.1:8001, see tuning/Tunables.java) ----
    // if the driver hub says COPY INTO CODE for one of these, the new number replaces everything after the =
    // (the "/ pixelScale" too). the speed limits below are final so a typo in Panels can't make it spin flat out.
    // "Go to Ball PID (new)" can also change kP / kD with the d-pad, that shows up the same way
    public static double kP = 0.0040 / pixelScale;
    public static double kD = 0.0006 / pixelScale;
    // PD for now. I on a camera loop just makes it overshoot. unstickPower covers what I would do
    public static double kI = 0.0;
    public static double unstickPower = 0.10;   // below this the wheels just buzz
    public static double windupLeash = 0.15;   // cap on the I term
    public static double kpForward = 0.006 / pixelScale;   // "kpScoot", drive power per px the ball is too small

    // remembers the numbers above as "what the code says". has to stay below them
    static {
        Tunables.track(BallVisionConfig.class);
    }

    // ---- colour mask ----
    // sample had these at 15, way too big for a 3 inch ball, it glued them to the floor
    public static final int blurSize = 5;   // "smooshSize"
    public static final int dilateSize = 5;   // "puffSize"
    public static final int erodeSize = 5;   // "shrinkSize"

    // the built in YELLOW lets in anything slightly warm, this is pickier
    // numbers are brightness, redness, blueness (YCrCb). real yellow = bright, kinda red, not blue at all
    public static final Scalar yellowLow = new Scalar(40, 135, 0);   // 40 so the shady half of the ball still counts
    public static final Scalar yellowHigh = new Scalar(255, 180, 100);

    // ---- when to stop (safety limits, not tunable) ----
    public static final double closeEnoughPx = 8 * pixelScale;   // ball this close to the middle = stop turning
    public static final double spinSpeedLimit = 0.45;   // max turn power
    public static final double closeRadiusPx = 55 * pixelScale;   // "snuggleRadiusPx", bigger ball on screen = closer. stop driving at this size
    public static final double forwardSpeedLimit = 0.35;   // "scootSpeedLimit", max forward power
    public static final double aimedEnoughPx = 40 * pixelScale;   // only drive forward once the ball is this close to the middle

    // ---- which blob is a pollen ball ("Auto Go to Closest Ball") ----
    // BIOBUZZ POLLEN: ~2.8 in diameter (+/- 0.1 in per game manual / AndyMark am-5851).
    // NECTAR is ~3.6 in and not yellow; size gates here are for single pollen only.
    public static final double pollenDiameterIn = 2.8;
    public static final double pollenDiameterTolIn = 0.1;
    // closeRadiusPx is the fitted-circle size when one pollen is "close" in our tuning
    public static final double maxPollenRadiusPx = closeRadiusPx
            * (1.0 + pollenDiameterTolIn / pollenDiameterIn) * 1.08;
    public static final double minPollenRadiusPx = 6 * pixelScale;
    public static final double maxPollenAreaPx = Math.PI * maxPollenRadiusPx * maxPollenRadiusPx;
    public static final double minPollenAreaPx = Math.PI * minPollenRadiusPx * minPollenRadiusPx;
    // two touching 2.8 in balls ≈ ~2x one ball's area in the mask
    public static final double mergedPollenAreaPx = maxPollenAreaPx * 1.85;
    public static final double targetStickinessPx = 40 * pixelScale;   // keep chasing the same ball if it moved less than this
    public static final double closerSwitchFraction = 0.15;   // switch only if another ball looks this much bigger (closer)

    private BallVisionConfig() {
    }
}
