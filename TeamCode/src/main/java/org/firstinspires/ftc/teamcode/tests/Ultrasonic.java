package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;

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
     * Puts every hub in MANUAL bulk caching. Call clearBulkCache(hubs) once at the top of every loop:
     * then the first read in a loop fetches all 4 analog channels in one transaction, so a sensor's signal and
     * Vcc come from the same instant (and the loop runs faster). Forgetting to clear = readings never change.
     */
    public static List<LynxModule> manualBulkReads(HardwareMap hardwareMap) {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        return hubs;
    }

    public static void clearBulkCache(List<LynxModule> hubs) {
        for (LynxModule hub : hubs) hub.clearBulkCache();
    }

    /**
     * Reads one MaxBotix HRLV-ShortRange-EZ (MB16xx) using the ratiometric method.
     *
     * Datasheet: distance_mm = (Vobserved / (Vcc / 1024)) * 6 - 300
     * Vcc is measured on a second analog channel, so the result does not depend on what the
     * supply voltage actually is or on the ADC's scaling: only on the ratio Vout / Vcc.
     *
     * Why the filter looks like this: in free-run the sensor only puts out a new value every 100 ms, and that
     * value has already been through the sensor's own 2 Hz filter. Anything that changes faster than that is
     * electrical/ADC noise on the hub side (1 mV of noise = about 2 mm). So:
     *   - Vcc is a steady 3.3 V, smooth it a lot instead of dividing by a fresh noisy sample every time
     *   - average every reading from the last windowSec (default 0.2 s = 2 sensor updates), after throwing
     *     out any reading far from the window's median so one spike can't pull the average
     * The window is in seconds, not calls, so reading more than once per loop doesn't change the smoothing.
     */
    public static class HrlvShortRange {
        static final double MM_PER_BIT = 6.0;
        static final double OFFSET_MM = -300.0;
        // Datasheet: minimum reported distance is 20 mm, maximum is 5000 mm
        static final double MIN_MM = 20.0;
        static final double MAX_MM = 5000.0;

        public static final double DEFAULT_WINDOW_SEC = 0.2;
        static final double VCC_TAU_SEC = 0.5;   // time constant of the Vcc smoothing
        static final double OUTLIER_MM = 25.0;   // readings further than this from the window's median are dropped
        static final int MAX_SAMPLES = 64;   // ring buffer size, plenty for 0.2 s even at a few hundred loops a second

        private final AnalogInput signal;
        private final AnalogInput vcc;
        private final ElapsedTime clock = new ElapsedTime();
        private final double[] times = new double[MAX_SAMPLES];
        private final double[] mms = new double[MAX_SAMPLES];
        private final double[] scratch = new double[MAX_SAMPLES];
        private int count = 0;
        private int next = 0;
        private double windowSec = DEFAULT_WINDOW_SEC;
        private double vccSmooth = Double.NaN, vccTime;
        private double lastSignal = Double.NaN, lastVcc = Double.NaN;

        public HrlvShortRange(AnalogInput signal, AnalogInput vcc) {
            this.signal = signal;
            this.vcc = vcc;
        }

        // how many seconds of readings get averaged. longer = smoother but slower to react
        public void setWindowSec(double sec) {
            windowSec = Math.max(0.0, sec);
        }

        public double windowSec() {
            return windowSec;
        }

        public double getSignalVoltage() {
            return signal.getVoltage();
        }

        public double getVccVoltage() {
            return vcc.getVoltage();
        }

        /**
         * Reads the hub once and returns the filtered distance (see the class comment).
         * Returns NaN while the supply reads under 1 V (sensor unpowered / miswired).
         */
        public double getDistanceMm() {
            double t = clock.seconds();
            double supply = vcc.getVoltage();
            lastVcc = supply;
            if (supply < 1.0) {
                // don't average across a wiring glitch
                count = 0;
                vccSmooth = Double.NaN;
                lastSignal = Double.NaN;
                return Double.NaN;
            }

            if (Double.isNaN(vccSmooth)) {
                vccSmooth = supply;
            } else {
                double a = 1 - Math.exp(-(t - vccTime) / VCC_TAU_SEC);
                vccSmooth += a * (supply - vccSmooth);
            }
            vccTime = t;

            lastSignal = signal.getVoltage();
            times[next] = t;
            mms[next] = ratioToMm(lastSignal / vccSmooth);
            next = (next + 1) % MAX_SAMPLES;
            count = Math.min(count + 1, MAX_SAMPLES);
            return filtered(t);
        }

        // mean of the readings in the window, ignoring ones far from the median
        private double filtered(double now) {
            int n = 0;
            for (int k = 0; k < count; k++) {
                int i = (next - 1 - k + MAX_SAMPLES) % MAX_SAMPLES;   // newest first
                if (k > 0 && now - times[i] > windowSec) break;
                scratch[n++] = mms[i];
            }
            double[] sorted = Arrays.copyOf(scratch, n);
            Arrays.sort(sorted);
            double median = n % 2 == 1 ? sorted[n / 2] : (sorted[n / 2 - 1] + sorted[n / 2]) / 2;

            double sum = 0;
            int used = 0;
            for (int i = 0; i < n; i++) {
                if (Math.abs(scratch[i] - median) <= OUTLIER_MM) {
                    sum += scratch[i];
                    used++;
                }
            }
            return used > 0 ? sum / used : median;
        }

        // what the last getDistanceMm() read off the hub, no new read
        public double lastSignalVoltage() {
            return lastSignal;
        }

        public double lastVccVoltage() {
            return lastVcc;
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

        List<LynxModule> hubs = manualBulkReads(hardwareMap);

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            clearBulkCache(hubs);
            double leftMm = left.getDistanceMm();
            double rightMm = right.getDistanceMm();

            telemetry.addData("Left signal / Vcc", "%.3f V / %.3f V", left.lastSignalVoltage(), left.lastVccVoltage());
            telemetry.addData("Left", "%.0f mm  (%.1f in)", leftMm, leftMm / 25.4);
            telemetry.addData("Right signal / Vcc", "%.3f V / %.3f V", right.lastSignalVoltage(), right.lastVccVoltage());
            telemetry.addData("Right", "%.0f mm  (%.1f in)", rightMm, rightMm / 25.4);
            telemetry.update();

            // more reads per sensor update = more readings for the filter to average
            sleep(20);
        }
    }
}
