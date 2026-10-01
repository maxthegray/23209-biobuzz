package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * One shared forward power for all wheels. Move the left stick to set it; hold a wheel
 * button to run that motor at the set power. Directions match {@link Constants#drivetrainConfig}.
 */
@TeleOp(name = "Drive Motor Speed Test", group = "test")
public class DriveMotorSpeedTest extends LinearOpMode {

    private static final double STICK_DEADZONE = 0.05;

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    /** Shared forward power (0–1) for every wheel test. */
    private double motorSpeed = 0.5;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx motor : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addLine("LEFT STICK: set motor speed (stays until you move stick again).");
        telemetry.addLine("Hold wheel button to run at that speed:");
        telemetry.addLine("D-pad / face: Up/Y=FL  Right/B=FR  Down/A=BL  Left/X=BR");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            updateMotorSpeedFromStick();

            boolean runFl = gamepad1.dpad_up || gamepad1.y;
            boolean runFr = gamepad1.dpad_right || gamepad1.b;
            boolean runBl = gamepad1.dpad_down || gamepad1.a;
            boolean runBr = gamepad1.dpad_left || gamepad1.x;

            frontLeft.setPower(runFl ? motorSpeed : 0);
            frontRight.setPower(runFr ? motorSpeed : 0);
            backLeft.setPower(runBl ? motorSpeed : 0);
            backRight.setPower(runBr ? motorSpeed : 0);

            telemetry.addData("Motor speed (power)", "%.3f", motorSpeed);
            telemetry.addLine("Velocity (ticks/s):");
            telemetry.addData("  FL", "%.1f", frontLeft.getVelocity());
            telemetry.addData("  FR", "%.1f", frontRight.getVelocity());
            telemetry.addData("  BL", "%.1f", backLeft.getVelocity());
            telemetry.addData("  BR", "%.1f", backRight.getVelocity());
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    /** Same forward convention as Starter Drive; value sticks when the stick returns to center. */
    private void updateMotorSpeedFromStick() {
        double stick = -gamepad1.left_stick_y;
        if (Math.abs(stick) > STICK_DEADZONE) {
            motorSpeed = Range.clip(stick, 0, 1);
        }
    }
}
