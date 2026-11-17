package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.tuning.Tunables;

// THE ONE PLACE to edit the ultrasonic correction. everything that uses corrected readings reads these:
// FlowerApproachTest (WallApproach), DriveToWallTest, UltrasonicAutoAlignment.
//
//   corrected inches = scale * raw inches + offset
//
// raw inches = the datasheet formula (Ultrasonic.HrlvShortRange), no fix.
// get new numbers from "Flower Approach Test (ultrasonic)": the results screen / website shows them after the
// formula check. paste them below, then redeploy.
//
// you can also try numbers live without redeploying: open Panels (http://192.168.43.1:8001) and change them there.
// that is NOT saved. the driver hub shows COPY INTO CODE with the new numbers, type them in below to keep them.
@Configurable
public final class UltrasonicCalibration {

    public static double leftScale = 1.2649;
    public static double leftOffset = 1.718;
    // an older right side fit was scale 1.2237, offset 2.285
    public static double rightScale = 1.2649;
    public static double rightOffset = 1.718;

    // remembers the numbers above as "what the code says". has to stay below them
    static {
        Tunables.track(UltrasonicCalibration.class);
    }

    private UltrasonicCalibration() {
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
