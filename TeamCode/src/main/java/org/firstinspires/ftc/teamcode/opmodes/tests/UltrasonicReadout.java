package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.UltrasonicConfig;
import org.firstinspires.ftc.teamcode.logging.FlightRecorder;
import org.firstinspires.ftc.teamcode.sensors.BulkReads;
import org.firstinspires.ftc.teamcode.sensors.UltrasonicSensor;

import java.util.List;

// shows what both ultrasonic sensors read: the voltages, the raw datasheet distance and the corrected one.
// the robot never moves. good first check after touching the wiring
@TeleOp(name = "MaxBotix HRLV-ShortRange Test", group = "Sensor")
public class UltrasonicReadout extends LinearOpMode {

    @Override
    public void runOpMode() {
        UltrasonicSensor left = UltrasonicSensor.left(hardwareMap);
        UltrasonicSensor right = UltrasonicSensor.right(hardwareMap);
        List<LynxModule> hubs = BulkReads.manual(hardwareMap);
        FlightRecorder log = FlightRecorder.start(this);   // saves the run to a file on the hub

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            BulkReads.clear(hubs);
            double leftRawIn = left.getDistanceIn();
            double rightRawIn = right.getDistanceIn();
            double leftIn = UltrasonicConfig.left(leftRawIn);
            double rightIn = UltrasonicConfig.right(rightRawIn);

            telemetry.addData("Left signal / Vcc", "%.3f V / %.3f V", left.lastSignalVoltage(), left.lastVccVoltage());
            telemetry.addData("Left", "raw %.1f in -> %.1f in corrected", leftRawIn, leftIn);
            telemetry.addData("Right signal / Vcc", "%.3f V / %.3f V", right.lastSignalVoltage(), right.lastVccVoltage());
            telemetry.addData("Right", "raw %.1f in -> %.1f in corrected", rightRawIn, rightIn);
            telemetry.update();

            // same channel names WallApproach uses, so BuzzScope shows them the same way
            log.put("Ultrasonic/leftSignalVolts", left.lastSignalVoltage());
            log.put("Ultrasonic/leftVccVolts", left.lastVccVoltage());
            log.put("Ultrasonic/rightSignalVolts", right.lastSignalVoltage());
            log.put("Ultrasonic/rightVccVolts", right.lastVccVoltage());
            log.put("Ultrasonic/leftRawIn", leftRawIn);
            log.put("Ultrasonic/rightRawIn", rightRawIn);
            log.put("Ultrasonic/leftIn", leftIn);
            log.put("Ultrasonic/rightIn", rightIn);
            log.update();

            // more reads per sensor update = more readings for the filter to average
            sleep(20);
        }
    }
}
