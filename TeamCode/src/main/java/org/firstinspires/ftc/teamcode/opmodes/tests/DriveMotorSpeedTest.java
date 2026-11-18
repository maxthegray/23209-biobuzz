package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumMotors;

/**
 * One shared forward power for all wheels. Move the left stick to set it; hold a wheel
 * button to run that motor at the set power. Directions come from mechanisms/MecanumMotors (same as pedro/Constants).
 */
@TeleOp(name = "Drive Motor Speed Test", group = "test")
public class DriveMotorSpeedTest extends LinearOpMode {

    private static final double STICK_DEADZONE = 0.05;


    /** Shared forward power (0–1) for every wheel test. */
    private double motorSpeed = 0.5;

    @Override
    public void runOpMode() {
        MecanumMotors motors = new MecanumMotors(hardwareMap);   // the 4 drive motors, directions and brake already set
        for (DcMotorEx motor : motors.all()) motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);   // so getVelocity() works

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

            motors.frontLeft.setPower(runFl ? motorSpeed : 0);
            motors.frontRight.setPower(runFr ? motorSpeed : 0);
            motors.backLeft.setPower(runBl ? motorSpeed : 0);
            motors.backRight.setPower(runBr ? motorSpeed : 0);

            telemetry.addData("Motor speed (power)", "%.3f", motorSpeed);
            telemetry.addLine("Velocity (ticks/s):");
            telemetry.addData("  FL", "%.1f", motors.frontLeft.getVelocity());
            telemetry.addData("  FR", "%.1f", motors.frontRight.getVelocity());
            telemetry.addData("  BL", "%.1f", motors.backLeft.getVelocity());
            telemetry.addData("  BR", "%.1f", motors.backRight.getVelocity());
            telemetry.update();
        }

        motors.stop();
    }

    /** Same forward convention as Starter Drive; value sticks when the stick returns to center. */
    private void updateMotorSpeedFromStick() {
        double stick = -gamepad1.left_stick_y;
        if (Math.abs(stick) > STICK_DEADZONE) {
            motorSpeed = Range.clip(stick, 0, 1);
        }
    }
}
