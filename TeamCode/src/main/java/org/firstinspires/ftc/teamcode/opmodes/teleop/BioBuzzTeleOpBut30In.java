package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.DriveConfig;
import org.firstinspires.ftc.teamcode.config.HardwareNames;
import org.firstinspires.ftc.teamcode.logging.FlightRecorder;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumMotors;

@TeleOp(name = "Starter Drive (30 in limit)", group = "teaching")
public class BioBuzzTeleOpBut30In extends LinearOpMode {

    @Override
    public void runOpMode() {

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, HardwareNames.PINPOINT);
        MecanumMotors motors = new MecanumMotors(hardwareMap);   // the 4 drive motors, directions and brake already set

        // Zero the odometry so distance is measured from where the robot starts.
        // Robot must be still for this; it takes about a quarter second.
        pinpoint.resetPosAndIMU();
        sleep(300);

        FlightRecorder log = FlightRecorder.start(this);   // saves the run to a file on the hub, see logging/FlightRecorder.java

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        pinpoint.update();
        double lastX = pinpoint.getPosX(DistanceUnit.INCH);
        double lastY = pinpoint.getPosY(DistanceUnit.INCH);
        double distanceTraveled = 0;

        while (opModeIsActive()) {
            // Add up how far the robot moved since the last loop.
            pinpoint.update();
            double x = pinpoint.getPosX(DistanceUnit.INCH);
            double y = pinpoint.getPosY(DistanceUnit.INCH);
            distanceTraveled += Math.hypot(x - lastX, y - lastY);
            lastX = x;
            lastY = y;

            if (distanceTraveled >= DriveConfig.distanceLimitIn) {
                break;
            }

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
            telemetry.addData("Distance", "%.1f / %.1f in", distanceTraveled, DriveConfig.distanceLimitIn);
            telemetry.update();

            // pinpoint.update() above already read these, so this is no extra hub traffic. measured from where the
            // robot started, not field coordinates
            log.pose(x, y, pinpoint.getHeading(AngleUnit.RADIANS));
            log.put("Drive/distanceTraveledIn", distanceTraveled);
            log.update();
        }

        log.event("LOCKED at " + DriveConfig.distanceLimitIn + " in");
        // Limit reached: stop the robot and ignore the gamepad for the rest of the OpMode.
        motors.stop();

        while (opModeIsActive()) {
            telemetry.addLine("LOCKED - 30 inch limit reached");
            telemetry.addData("Distance", "%.1f in", distanceTraveled);
            telemetry.addLine("Stop the OpMode to drive again.");
            telemetry.update();
            log.update();
        }
    }
}
