package org.firstinspires.ftc.teamcode.tests;

// THE ONE PLACE to edit the ultrasonic correction. everything that uses corrected readings reads these:
// FlowerApproachTest (WallApproach), DriveToWallTest, UltrasonicAutoAlignment.
//
//   corrected inches = scale * raw inches + offset
//
// raw inches = the datasheet formula (Ultrasonic.HrlvShortRange), no fix.
// get new numbers from "Flower Approach Test (ultrasonic)": the results screen / website shows them after the
// formula check. paste them below, then redeploy.
public final class UltrasonicCalibration {

    public static final double leftScale = 1.2649, leftOffset = 1.718;
//    public static final double rightScale = 1.2237, rightOffset = 2.285;
    public static final double rightScale = 1.2649, rightOffset = 1.718;

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
