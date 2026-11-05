package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import java.util.Arrays;

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

    /**
     * Reads one MaxBotix HRLV-ShortRange-EZ (MB16xx) using the ratiometric method.
     *
     * Datasheet: distance_mm = (Vobserved / (Vcc / 1024)) * 6 - 300
     * Vcc is measured on a second analog channel, so the result does not depend on what the
     * supply voltage actually is or on the ADC's scaling: only on the ratio Vout / Vcc.
     */
    static class HrlvShortRange {
        static final double MM_PER_BIT = 6.0;
        static final double OFFSET_MM = -300.0;
        // Datasheet: minimum reported distance is 20 mm, maximum is 5000 mm
        static final double MIN_MM = 20.0;
        static final double MAX_MM = 5000.0;
        static final int SAMPLES = 3;

        private final AnalogInput signal;
        private final AnalogInput vcc;
        private final double[] history = new double[SAMPLES];
        private int count = 0;
        private int next = 0;

        HrlvShortRange(AnalogInput signal, AnalogInput vcc) {
            this.signal = signal;
            this.vcc = vcc;
        }

        double getSignalVoltage() {
            return signal.getVoltage();
        }

        double getVccVoltage() {
            return vcc.getVoltage();
        }

        /**
         * Call once per loop (the sensor updates every 100 ms, so some samples repeat). Returns the median of the
         * last SAMPLES readings, which rejects the occasional spike the sensor outputs.
         * Returns NaN until the supply reads a sane voltage (sensor unpowered / miswired).
         */
        double getDistanceMm() {
            double supply = vcc.getVoltage();
            if (supply < 1.0) return Double.NaN;

            history[next] = ratioToMm(signal.getVoltage() / supply);
            next = (next + 1) % SAMPLES;
            count = Math.min(count + 1, SAMPLES);

            double[] sorted = Arrays.copyOf(history, count);
            Arrays.sort(sorted);
            return sorted[count / 2];
        }

        static double ratioToMm(double vOverVcc) {
            double mm = vOverVcc * 1024.0 * MM_PER_BIT + OFFSET_MM;
            return Math.max(MIN_MM, Math.min(MAX_MM, mm));
        }
    }

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
