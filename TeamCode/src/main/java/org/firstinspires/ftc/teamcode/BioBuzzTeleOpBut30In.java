package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Starter Drive (30 in limit)", group = "teaching")
public class BioBuzzTeleOpBut30In extends LinearOpMode {

    // How far the robot is allowed to travel before the driver is locked out.
    private static final double DISTANCE_LIMIT_INCHES = 30.0;

    @Override
    public void runOpMode() {

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "back_left");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "back_right");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Zero the odometry so distance is measured from where the robot starts.
        // Robot must be still for this; it takes about a quarter second.
        pinpoint.resetPosAndIMU();
        sleep(300);

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        pinpoint.update();
        double lastX = pinpoint.getPosX(DistanceUnit.INCH);
        double lastY = pinpoint.getPosY(DistanceUnit.INCH);
        double distanceTraveled = 0;

        double scale = .5;
        while (opModeIsActive()) {
            // Add up how far the robot moved since the last loop.
            pinpoint.update();
            double x = pinpoint.getPosX(DistanceUnit.INCH);
            double y = pinpoint.getPosY(DistanceUnit.INCH);
            distanceTraveled += Math.hypot(x - lastX, y - lastY);
            lastX = x;
            lastY = y;

            if (distanceTraveled >= DISTANCE_LIMIT_INCHES) {
                break;
            }

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;


            double frontLeftPower = drive + strafe + turn;
            double frontRightPower = drive - strafe - turn;
            double backLeftPower = drive - strafe + turn;
            double backRightPower = drive + strafe - turn;

            frontLeft.setPower(frontLeftPower * scale);
            frontRight.setPower(frontRightPower * scale);
            backLeft.setPower(backLeftPower * scale);
            backRight.setPower(backRightPower * scale);

            telemetry.addData("Drive", "%.2f", drive);
            telemetry.addData("Strafe", "%.2f", strafe);
            telemetry.addData("Turn", "%.2f", turn);
            telemetry.addData("Distance", "%.1f / %.1f in", distanceTraveled, DISTANCE_LIMIT_INCHES);
            telemetry.update();
        }

        // Limit reached: stop the robot and ignore the gamepad for the rest of the OpMode.
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        while (opModeIsActive()) {
            telemetry.addLine("LOCKED - 30 inch limit reached");
            telemetry.addData("Distance", "%.1f in", distanceTraveled);
            telemetry.addLine("Stop the OpMode to drive again.");
            telemetry.update();
        }
    }
}
