package org.firstinspires.ftc.teamcode.vision.ultrasonic;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.tests.Ultrasonic.HrlvShortRange;

/**
 * Hold TRIANGLE to drive forward toward the wall. Each side of the drivetrain is controlled by the
 * ultrasonic sensor on that side, so the robot also squares itself to the wall. A side stops once its
 * sensor reads TARGET_IN or closer. Releasing TRIANGLE stops the robot.
 *
 * Note: the HRLV-ShortRange's minimum reading is 20 mm (0.79 in), so 0.8 in is right at the bottom of
 * what the sensor can report. In practice a side stops when its sensor bottoms out.
 */
@TeleOp(name = "Drive to Wall w/ Ultrasonic", group = "Sensor")
public class DriveStraightUltrasonic extends LinearOpMode {
    private static final String LEFT_SIGNAL = "ultrasonic_left_signal";
    private static final String LEFT_VCC = "ultrasonic_left_vcc";
    private static final String RIGHT_SIGNAL = "ultrasonic_right_signal";
    private static final String RIGHT_VCC = "ultrasonic_right_vcc";

    private static final double TARGET_IN = 5;
    private static final double TARGET_MM = TARGET_IN * 25.4;

    // Power ramps down linearly as the robot gets closer: full MAX_POWER beyond SLOW_ZONE_MM from the target,
    // never below MIN_POWER (so the robot doesn't stall just short of the wall).
    private static final double MAX_POWER = 0.4;
    private static final double MIN_POWER = 0.12;
    private static final double SLOW_ZONE_MM = 300.0;

    @Override
    public void runOpMode() {
        HrlvShortRange left = new HrlvShortRange(
                hardwareMap.get(AnalogInput.class, LEFT_SIGNAL), hardwareMap.get(AnalogInput.class, LEFT_VCC));
        HrlvShortRange right = new HrlvShortRange(
                hardwareMap.get(AnalogInput.class, RIGHT_SIGNAL), hardwareMap.get(AnalogInput.class, RIGHT_VCC));

        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeftMotor");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addData("Status", "Initialized. Hold TRIANGLE to drive to the wall.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read every loop so the median filters stay fresh even while not driving
            double leftMm = left.getDistanceMm();
            double rightMm = right.getDistanceMm();

            double leftPower = 0;
            double rightPower = 0;
            if (gamepad1.triangle) {
                leftPower = powerFor(leftMm);
                rightPower = powerFor(rightMm);
            }

            frontLeft.setPower(leftPower);
            backLeft.setPower(leftPower);
            frontRight.setPower(rightPower);
            backRight.setPower(rightPower);

            boolean leftDone = leftMm <= TARGET_MM;
            boolean rightDone = rightMm <= TARGET_MM;

            telemetry.addData("Triangle", gamepad1.triangle ? "HELD" : "released");
            telemetry.addData("Left", "%.0f mm (%.2f in)  power %.2f", leftMm, leftMm / 25.4, leftPower);
            telemetry.addData("Right", "%.0f mm (%.2f in)  power %.2f", rightMm, rightMm / 25.4, rightPower);
            telemetry.addData("At wall", leftDone && rightDone ? "YES" : "no");
            telemetry.update();

            sleep(20);
        }

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    /** Forward power for one side, or 0 if that side has reached the target (or the sensor isn't reading). */
    private static double powerFor(double distanceMm) {
        if (Double.isNaN(distanceMm)) return 0;
        double remaining = distanceMm - TARGET_MM;
        if (remaining <= 0) return 0;
        return Range.clip(MAX_POWER * remaining / SLOW_ZONE_MM, MIN_POWER, MAX_POWER);
    }
}
