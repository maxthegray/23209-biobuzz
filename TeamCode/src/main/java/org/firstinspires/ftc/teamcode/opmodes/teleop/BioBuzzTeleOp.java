package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.DriveConfig;
import org.firstinspires.ftc.teamcode.config.HardwareNames;
import org.firstinspires.ftc.teamcode.logging.FlightRecorder;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumMotors;

@TeleOp(name = "Starter Drive", group = "teaching")
public class BioBuzzTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, HardwareNames.PINPOINT);
        MecanumMotors motors = new MecanumMotors(hardwareMap);   // the 4 drive motors, directions and brake already set

        FlightRecorder log = FlightRecorder.start(this);   // saves the run to a file on the hub, see logging/FlightRecorder.java

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;


            double frontLeftPower = drive + strafe + turn;
            double frontRightPower = drive - strafe - turn;
            double backLeftPower = drive - strafe + turn;
            double backRightPower = drive + strafe - turn;

            motors.frontLeft.setPower(frontLeftPower * DriveConfig.speedScale());
            motors.frontRight.setPower(frontRightPower * DriveConfig.speedScale());
            motors.backLeft.setPower(backLeftPower * DriveConfig.speedScale());
            motors.backRight.setPower(backRightPower * DriveConfig.speedScale());

            telemetry.addData("Drive", "%.2f", drive);
            telemetry.addData("Strafe", "%.2f", strafe);
            telemetry.addData("Turn", "%.2f", turn);
            telemetry.update();
            log.update();
        }
    }
}
