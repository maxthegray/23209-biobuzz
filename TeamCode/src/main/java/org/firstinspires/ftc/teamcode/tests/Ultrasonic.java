package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.vision.ultrasonic.HrlvShortRange;

@TeleOp(name = "MaxBotix HRLV-ShortRange Test", group = "Sensor")
public class Ultrasonic extends LinearOpMode {

    // Names must match the analog devices in the robot configuration.
    // Each sensor uses BOTH channels of one hub analog port (blue = even channel, white = odd):
    //   sensor pin 3 (analog out) -> signal wire,  sensor pin 6 (V+) -> power wire AND the other signal wire,
    //   sensor pin 7 (GND) -> ground wire.
    private static final String LEFT_SIGNAL = "ultrasonic_left_signal";
    private static final String LEFT_VCC = "ultrasonic_left_vcc";
    private static final String RIGHT_SIGNAL = "ultrasonic_right_signal";
    private static final String RIGHT_VCC = "ultrasonic_right_vcc";

    @Override
    public void runOpMode() {
        HrlvShortRange left = new HrlvShortRange(
                hardwareMap.get(AnalogInput.class, LEFT_SIGNAL), hardwareMap.get(AnalogInput.class, LEFT_VCC));
        HrlvShortRange right = new HrlvShortRange(
                hardwareMap.get(AnalogInput.class, RIGHT_SIGNAL), hardwareMap.get(AnalogInput.class, RIGHT_VCC));

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double leftMm = left.getDistanceMm();
            double rightMm = right.getDistanceMm();

            telemetry.addData("Left signal / Vcc", "%.3f V / %.3f V", left.getSignalVoltage(), left.getVccVoltage());
            telemetry.addData("Left", "%.0f mm  (%.1f in)", leftMm, leftMm / 25.4);
            telemetry.addData("Right signal / Vcc", "%.3f V / %.3f V", right.getSignalVoltage(), right.getVccVoltage());
            telemetry.addData("Right", "%.0f mm  (%.1f in)", rightMm, rightMm / 25.4);
            telemetry.update();

            // Short poll interval keeps latency low; the median only needs a few samples
            sleep(50);
        }
    }
}
