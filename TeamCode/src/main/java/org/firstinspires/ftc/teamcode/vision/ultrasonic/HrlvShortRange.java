package org.firstinspires.ftc.teamcode.vision.ultrasonic;

import com.qualcomm.robotcore.hardware.AnalogInput;

import java.util.Arrays;

/**
 * Reads one MaxBotix HRLV-ShortRange-EZ (MB16xx) using the ratiometric method.
 *
 * Datasheet: distance_mm = (Vobserved / (Vcc / 1024)) * 6 - 300
 * Vcc is measured on a second analog channel, so the result does not depend on what the
 * supply voltage actually is or on the ADC's scaling: only on the ratio Vout / Vcc.
 */
public class HrlvShortRange {
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

    public HrlvShortRange(AnalogInput signal, AnalogInput vcc) {
        this.signal = signal;
        this.vcc = vcc;
    }

    public double getSignalVoltage() {
        return signal.getVoltage();
    }

    public double getVccVoltage() {
        return vcc.getVoltage();
    }

    /**
     * Call once per loop (the sensor updates every 100 ms, so some samples repeat). Returns the median of the
     * last SAMPLES readings, which rejects the occasional spike the sensor outputs.
     * Returns NaN until the supply reads a sane voltage (sensor unpowered / miswired).
     */
    public double getDistanceMm() {
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
