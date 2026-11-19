package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.HardwareNames;
import org.firstinspires.ftc.teamcode.config.UltrasonicConfig;

import java.util.Arrays;

// one MaxBotix HRLV-ShortRange-EZ (MB16xx) ultrasonic sensor, read the ratiometric way.
// this gives the RAW distance from the datasheet formula. the calibration fix is applied on top
// (UltrasonicConfig.left / right), and the numbers you'd change are in config/UltrasonicConfig.java.
//
// datasheet: distance_mm = (Vobserved / (Vcc / 1024)) * 6 - 300
// Vcc is measured on a second analog channel, so the result only depends on the ratio Vout / Vcc,
// not on what the supply voltage actually is or on the ADC's scaling.
//
// why the filter looks like this: in free-run the sensor only puts out a new value every 100 ms, and that
// value has already been through the sensor's own 2 Hz filter. anything faster than that is electrical/ADC
// noise on the hub side (1 mV of noise = about 2 mm). so:
//   - Vcc is a steady 3.3 V, smooth it a lot instead of dividing by a fresh noisy sample every time
//   - average every reading from the last windowSec (default 0.2 s = 2 sensor updates), after throwing
//     out any reading far from the window's median so one spike can't pull the average
// the window is in seconds, not calls, so reading more than once per loop doesn't change the smoothing.
//
// wiring: sensor pin 3 (analog out) -> signal wire, pin 6 (V+) -> power wire AND the other signal wire,
// pin 7 (GND) -> ground. blue = even channel, white = odd
public class UltrasonicSensor {

    // from the datasheet, not something to tune
    static final double MM_PER_BIT = 6.0;
    static final double OFFSET_MM = -300.0;
    static final double MIN_MM = 20.0;   // closest it reports
    static final double MAX_MM = 5000.0;   // farthest it reports
    static final int MAX_SAMPLES = 64;   // ring buffer size, plenty for 0.2 s even at a few hundred loops a second

    private final AnalogInput signal;
    private final AnalogInput vcc;
    private final ElapsedTime clock = new ElapsedTime();
    private final double[] times = new double[MAX_SAMPLES];
    private final double[] mms = new double[MAX_SAMPLES];
    private final double[] scratch = new double[MAX_SAMPLES];
    private int count = 0;
    private int next = 0;
    private double windowSec = UltrasonicConfig.defaultWindowSec;
    private double vccSmooth = Double.NaN, vccTime;
    private double lastSignal = Double.NaN, lastVcc = Double.NaN;

    public UltrasonicSensor(AnalogInput signal, AnalogInput vcc) {
        this.signal = signal;
        this.vcc = vcc;
    }

    // the two sensors on the front of the robot, by their names in the robot configuration
    public static UltrasonicSensor left(HardwareMap hardwareMap) {
        return new UltrasonicSensor(hardwareMap.get(AnalogInput.class, HardwareNames.ULTRASONIC_LEFT_SIGNAL),
                hardwareMap.get(AnalogInput.class, HardwareNames.ULTRASONIC_LEFT_VCC));
    }

    public static UltrasonicSensor right(HardwareMap hardwareMap) {
        return new UltrasonicSensor(hardwareMap.get(AnalogInput.class, HardwareNames.ULTRASONIC_RIGHT_SIGNAL),
                hardwareMap.get(AnalogInput.class, HardwareNames.ULTRASONIC_RIGHT_VCC));
    }

    // same as left/right, but null when the robot config doesn't have both ports (a robot without the sensors)
    public static UltrasonicSensor tryLeft(HardwareMap hardwareMap) {
        return tryGet(hardwareMap, HardwareNames.ULTRASONIC_LEFT_SIGNAL, HardwareNames.ULTRASONIC_LEFT_VCC);
    }

    public static UltrasonicSensor tryRight(HardwareMap hardwareMap) {
        return tryGet(hardwareMap, HardwareNames.ULTRASONIC_RIGHT_SIGNAL, HardwareNames.ULTRASONIC_RIGHT_VCC);
    }

    private static UltrasonicSensor tryGet(HardwareMap hardwareMap, String signalName, String vccName) {
        AnalogInput signal = hardwareMap.tryGet(AnalogInput.class, signalName);
        AnalogInput vcc = hardwareMap.tryGet(AnalogInput.class, vccName);
        return signal == null || vcc == null ? null : new UltrasonicSensor(signal, vcc);
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

    // reads the hub once and returns the filtered raw distance in mm.
    // NaN while the supply reads under 1 V (sensor unpowered / miswired)
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
            double a = 1 - Math.exp(-(t - vccTime) / UltrasonicConfig.vccTauSec);
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

    // same reading in inches
    public double getDistanceIn() {
        return getDistanceMm() / 25.4;
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
            if (Math.abs(scratch[i] - median) <= UltrasonicConfig.outlierMm) {
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
