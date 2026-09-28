package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Shooter Velocity Test", group = "test")
public class ShooterVelocityTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        double target = 0;
        double step = 100;
        boolean running = false;
        boolean prevTriangle = false, prevCross = false, prevCircle = false, prevSquare = false, prevR1 = false;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.triangle && !prevTriangle) target += step;
            if (gamepad1.cross && !prevCross) target = Math.max(0, target - step);
            if (gamepad1.circle && !prevCircle) step *= 10;
            if (gamepad1.square && !prevSquare) step = Math.max(1, step / 10);
            if (gamepad1.right_bumper && !prevR1) running = !running;
            if (gamepad1.left_bumper) target = 0;

            prevTriangle = gamepad1.triangle;
            prevCross = gamepad1.cross;
            prevCircle = gamepad1.circle;
            prevSquare = gamepad1.square;
            prevR1 = gamepad1.right_bumper;

            shooter.setVelocity(running ? target : 0);

            telemetry.addData("running", running);
            telemetry.addData("target (ticks/s)", target);
            telemetry.addData("current (ticks/s)", "%.1f", shooter.getVelocity());
            telemetry.addData("step", step);
            telemetry.addData("power", "%.2f", shooter.getPower());
            telemetry.addData("encoder position", shooter.getCurrentPosition());
            telemetry.addLine("triangle/x: +/- step | cirlce/square: step x10 /10 | R1: toggle on/off | L1: zero");
            telemetry.update();
        }
    }
}
