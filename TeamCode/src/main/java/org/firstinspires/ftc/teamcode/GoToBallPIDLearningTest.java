package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Size;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

@TeleOp(name = "Go to Ball PID (Learning Test)", group = "teaching?")
public class GoToBallPIDLearningTest extends LinearOpMode{
//    private ColorBlobLocatorProcessor.Blob findClosestBall(List<ColorBlobLocatorProcessor.Blob> blobs){
//        ColorBlobLocatorProcessor.Blob closestBall = null;
//        double closestDistance = Double.MAX_VALUE;
//        for (ColorBlobLocatorProcessor.Blob b : blobs) {
//            Circle circleFit = b.getCircle();
//
//
//        }
//    }

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


        final double camCenterX = 160;
        final double camCenterY = 120;

        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image

                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))

                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

//        telemetry.setMsTransmissionInterval(100);   // Speed up telemetry updates for debugging.
        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {
            telemetry.addData("preview on/off", "... Camera Stream\n");
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    50, 1000, blobs);  // filter out very small blobs.

            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                    0.6, 1, blobs);     // filter out non-circular blobs.

            telemetry.addLine("Circularity Radius Center");

//            for (ColorBlobLocatorProcessor.Blob b : blobs) {
//                Circle circleFit = b.getCircle();
//                telemetry.addLine(String.format("%5.3f      %3d     (%3d,%3d)",
//                        b.getCircularity(), (int) circleFit.getRadius(), (int) circleFit.getX(), (int) circleFit.getY()));
//            }

            if (!blobs.isEmpty()){
                double maxRadius = 0.0;

                ColorBlobLocatorProcessor.Blob b = blobs.get(0);

                for(ColorBlobLocatorProcessor.Blob testBlob : blobs) {
                    Circle circleFit = testBlob.getCircle();
                    if(circleFit.getRadius() > maxRadius) {
                        maxRadius = circleFit.getRadius();
                        b = testBlob;
                    }
                }

                Circle circleFit = b.getCircle();
                double radius = circleFit.getRadius();
                double centerX = circleFit.getX();
                double centerY = circleFit.getY();

                double errorX = centerX - camCenterX;
//                double errorY = centerY - camCenterY;

                // Front left and back left are +turn; Other 2 are -turn
                double kP = 0.0025;
                double turn = -errorX * kP;

                frontLeft.setPower(turn);
                frontRight.setPower(-turn);
                backLeft.setPower(turn);
                backRight.setPower(-turn);
                telemetry.addLine("Ball found, moving.");
            } else {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);

                telemetry.addLine("No Ball found...");
            }

            telemetry.update();
            sleep(100); // Match the telemetry update interval.
        }
    }
}
