package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.tuning.Tunables;

// THE ONE PLACE for ultrasonic numbers. everything that uses the sensors reads them from here:
// WallApproach (FlowerApproachTest, DriveToWallTest, PathDemoAllCombos), UltrasonicAutoAlignment, UltrasonicReadout.
//
// 1. CALIBRATION:   corrected inches = scale * raw inches + offset
//    raw inches = the datasheet formula (sensors/UltrasonicSensor.java), no fix.
//    get new numbers from "Flower Approach Test (ultrasonic)": the results screen / website shows them after the
//    formula check. paste them below, then redeploy.
//    you can also try numbers live without redeploying: open Panels (http://192.168.43.1:8001) and change them there.
//    that is NOT saved. the driver hub shows COPY INTO CODE with the new numbers, type them in below to keep them.
//
// 2. everything below the calibration is measured or a safety limit: change it in the code, not in Panels.
@Configurable
public final class UltrasonicConfig {

    // ---- 1. calibration (tunable in Panels) ----
    public static double leftScale = 1.2649;
    public static double leftOffset = 1.718;
    // an older right side fit was scale 1.2237, offset 2.285
    public static double rightScale = 1.2649;
    public static double rightOffset = 1.718;

    // remembers the numbers above as "what the code says". has to stay below them
    static {
        Tunables.track(UltrasonicConfig.class);
    }

    // ---- 2. where the sensors sit on the robot (the flower dash website can override these for one run) ----
    public static final double sensorSpacingIn = 7.5;   // center to center, measured 2026-11-09
    public static final double sensorSetbackIn = 0.0;   // how far the sensor faces sit behind the bumper. measure this

    // ---- 3. where to stop in front of a flower ----
    public static final double flowerFromWallIn = 3.0;   // guess, measure on a real field
    public static final double flowerReachIn = 2.4;   // top ring sticks out this far
    public static final double frontMountsIn = 2.0;   // the sensor and limelight mounts stick out this far past the frame
    public static final double clearanceIn = 1.0;   // gap left between the mounts and the flower
    public static final double stopDistanceIn = flowerFromWallIn + flowerReachIn + frontMountsIn + clearanceIn;   // frame to wall

    // ---- 4. smoothing (see the comment at the top of sensors/UltrasonicSensor.java) ----
    public static final double defaultWindowSec = 0.2;   // average this many seconds of readings (2 sensor updates)
    public static final double outlierMm = 25.0;   // readings further than this from the window's median are dropped
    public static final double vccTauSec = 0.5;   // time constant of the supply voltage smoothing

    // ---- 5. is this reading sane? (safety limits, not tunable) ----
    public static final double minReadingIn = 1.0;   // under this is a loose signal wire (it reads as 0.8), not a real wall
    public static final double maxSkewIn = 2.5;   // 7.5 in apart, so 2.5 in is already ~18 deg crooked. more = a sensor is seeing the flower
    public static final double tooFarIn = 30.0;   // won't drive unless both sensors see something closer than this

    private UltrasonicConfig() {
    }

    public static double left(double rawIn) {
        return leftScale * rawIn + leftOffset;
    }

    public static double right(double rawIn) {
        return rightScale * rawIn + rightOffset;
    }

    // {leftScale, leftOffset, rightScale, rightOffset}
    public static double[] values() {
        return new double[]{leftScale, leftOffset, rightScale, rightOffset};
    }
}
