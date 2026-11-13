package org.firstinspires.ftc.teamcode.tests.flowerdash;

import android.os.Environment;

import org.firstinspires.ftc.teamcode.tests.FlowerApproachTest.Fit;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

// everything the calibration test knows: stage, results, every sensor sample, drive trials.
// the opmode thread calls tick() and the setters, web threads only read (all synchronized)
public class CalibrationSession {

    public enum Stage { CALIBRATE, RESULTS, DRIVE }

    public enum Phase { IDLE, SETTLING, RECORDING, DRIVE }

    static final int MAX_SAMPLES = 200000;   // about an hour at 50 Hz
    static final double MIN_LOG_GAP = 0.015;   // s, the drive loop runs much faster than we need to log

    private static class Step {
        double tape;
        boolean recorded;
        double left, right, leftWobble, rightWobble;
        int n;
    }

    private static class Sample {
        double t, tape, l, r, v;
        int step, attempt;
        Phase phase;
        boolean counted, corrected;
    }

    private static class Trial {
        int n;
        double thinks, measured;
        String result;
        boolean fix;
    }

    private static final String RAW_HEADER = "t_s,attempt,step,phase,counted,corrected,tape_in,left_in,right_in,supply_v";
    private static final String SUMMARY_HEADER = "step,tape_in,left_mean_in,right_mean_in,left_wobble_in,right_wobble_in,left_error_in,right_error_in,n_samples";
    private static final String FIT_HEADER = "sensor,scale,offset,r2,rmse_in,max_error_before_in,max_error_after_in,points,sensor_spacing_in,sensor_setback_in";
    private static final String TRIAL_HEADER = "trial,robot_thinks_in,tape_measured_in,error_in,result,fix_applied";

    private final String id = Long.toString(System.currentTimeMillis(), 36);
    private final double settleSec, recordSec, goodEnough, jumpy, stopDistance, tooFar;
    private final List<Step> steps = new ArrayList<>();
    private final List<Sample> samples = new ArrayList<>();
    private final List<Trial> trials = new ArrayList<>();

    private Stage stage = Stage.CALIBRATE;
    private Phase phase = Phase.IDLE;
    private int step = 0, attempt = 1;
    private double now, phaseStart, lastLog = -1;
    private double liveL = Double.NaN, liveR = Double.NaN, liveV = Double.NaN;
    private double accL, accR, minL, maxL, minR, maxR;
    private int accN;
    private String message = "";
    private double spacing, setback;
    private double[] pasted = {1, 0, 1, 0};
    private boolean started, usingFix, estop;
    private double[] fixNow = {1, 0, 1, 0};

    private boolean driving;
    private String driveResult = "", driveNote = "";
    private double[] drive = new double[9];   // left, right, wall, toGo, angle, drivePower, strafePower, turnPower, bumper

    private BufferedWriter rawOut;
    private File dir;
    private String stamp;
    private int unflushed;

    public CalibrationSession(double[] tapes, double settleSec, double recordSec, double goodEnough, double jumpy,
                              double stopDistance, double tooFar, double spacing, double setback) {
        this.settleSec = settleSec;
        this.recordSec = recordSec;
        this.goodEnough = goodEnough;
        this.jumpy = jumpy;
        this.stopDistance = stopDistance;
        this.tooFar = tooFar;
        this.spacing = spacing;
        this.setback = setback;
        for (double t : tapes) {
            Step s = new Step();
            s.tape = t;
            steps.add(s);
        }
        try {
            dir = new File(Environment.getExternalStorageDirectory(), "FIRST/flower_cal");
            dir.mkdirs();
            stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            rawOut = new BufferedWriter(new FileWriter(new File(dir, "run_" + stamp + "_raw.csv")));
            rawOut.write(RAW_HEADER + "\n");
        } catch (Exception e) {
            rawOut = null;   // no disk, the browser still keeps its own copy
        }
    }

    // ---------- called every loop by the opmode ----------

    // l and r are raw readings while calibrating, corrected ones while driving
    public synchronized void tick(double t, double l, double r, double v, boolean driveStage) {
        now = t;
        liveL = l;
        liveR = r;
        liveV = v;
        boolean counted = false;

        if (driveStage) {
            phase = Phase.DRIVE;
        } else {
            if (phase == Phase.DRIVE) phase = Phase.IDLE;
            if (phase == Phase.SETTLING && t - phaseStart >= settleSec) {
                phase = Phase.RECORDING;
                phaseStart = t;
                accL = accR = 0;
                accN = 0;
                minL = minR = 999;
                maxL = maxR = -999;
            }
        }

        Phase logged = phase;
        if (phase == Phase.RECORDING) {
            if (Double.isNaN(l) || Double.isNaN(r)) {
                phase = Phase.IDLE;
                message = "A sensor isn't reading (supply under 1 V). Check the wiring, then record again";
                logged = Phase.IDLE;
            } else {
                accL += l;
                accR += r;
                minL = Math.min(minL, l);
                maxL = Math.max(maxL, l);
                minR = Math.min(minR, r);
                maxR = Math.max(maxR, r);
                accN++;
                counted = true;
                if (t - phaseStart >= recordSec) finishStep();
            }
        }

        if (t - lastLog >= MIN_LOG_GAP && samples.size() < MAX_SAMPLES) {
            lastLog = t;
            Sample s = new Sample();
            s.t = t;
            s.attempt = attempt;
            s.step = step;
            s.phase = logged;
            s.counted = counted;
            s.corrected = driveStage;
            s.tape = (stage == Stage.CALIBRATE && step < steps.size()) ? steps.get(step).tape : Double.NaN;
            s.l = l;
            s.r = r;
            s.v = v;
            samples.add(s);
            writeRaw(s);
        }
    }

    private void finishStep() {
        // the step we were recording is the one that's selected, step only moves after this
        Step s = steps.get(step);
        s.left = accL / accN;
        s.right = accR / accN;
        s.leftWobble = maxL - minL;
        s.rightWobble = maxR - minR;
        s.n = accN;
        s.recorded = true;
        phase = Phase.IDLE;
        message = "";
        saveSummary();
        if (step < steps.size() - 1) step++;
        else finishCalibration();
    }

    private void writeRaw(Sample s) {
        if (rawOut == null) return;
        try {
            rawOut.write(rawRow(s));
            rawOut.write('\n');
            if (++unflushed >= 50) {
                rawOut.flush();
                unflushed = 0;
            }
        } catch (Exception e) {
            rawOut = null;
        }
    }

    public synchronized void close() {
        try {
            if (rawOut != null) rawOut.close();
        } catch (Exception ignored) {
        }
        rawOut = null;
    }

    // ---------- commands, all on the opmode thread ----------

    // why a command can't run right now, or null if it can
    public synchronized String validate(String cmd) {
        boolean idle = phase == Phase.IDLE;
        switch (cmd) {
            case "RECORD":
            case "REDO":
            case "DONE":
            case "GOTO_STEP":
            case "SET_TAPES":
                if (stage != Stage.CALIBRATE) return cmd + " ignored: not in the formula check";
                if (!idle) return cmd + " ignored: still recording (Cancel first)";
                return null;
            case "CANCEL":
                return idle ? "Nothing to cancel" : null;
            case "RESTART":
                return stage == Stage.DRIVE ? "Go back to results first" : null;
            case "GO_DRIVE":
                return stage == Stage.RESULTS ? null : "GO_DRIVE ignored: not on the results screen";
            case "SKIP_DRIVE":
                if (stage == Stage.DRIVE) return "Already in the drive check";
                return idle ? null : "Still recording (Cancel first)";
            case "BACK":
            case "LOG_TRIAL":
                return stage == Stage.DRIVE ? null : cmd + " ignored: not in the drive check";
            case "DRIVE_HEARTBEAT":
                if (!started) return "Press START on the Driver Hub before driving";
                if (estop) return "E-stop is latched, press Reset";
                return stage == Stage.DRIVE ? null : "Not in the drive check";
            case "SET_GEOMETRY":
            case "ESTOP":
            case "RESET":
                return null;
            default:
                return "Unknown command " + cmd;
        }
    }

    public synchronized void startRecord() {
        phase = Phase.SETTLING;
        phaseStart = now;
        message = "";
    }

    public synchronized void cancel() {
        phase = Phase.IDLE;
    }

    public synchronized void redo() {
        if (step > 0) step--;
    }

    public synchronized void gotoStep(int i) {
        if (i >= 0 && i < steps.size()) step = i;
    }

    public synchronized void restart() {
        for (Step s : steps) s.recorded = false;
        step = 0;
        phase = Phase.IDLE;
        stage = Stage.CALIBRATE;
        attempt++;
    }

    public synchronized void finishCalibration() {
        stage = Stage.RESULTS;
        phase = Phase.IDLE;
        saveSummary();
    }

    public synchronized void enterDrive(boolean fix, double[] correction) {
        usingFix = fix;
        fixNow = correction;
        stage = Stage.DRIVE;
        phase = Phase.DRIVE;
        driving = false;
        driveResult = "";
        driveNote = "";
    }

    public synchronized void backToResults() {
        stage = Stage.RESULTS;
        phase = Phase.IDLE;
        driving = false;
    }

    public synchronized void setTapes(double[] tapes) {
        if (tapes.length == 0) return;
        List<Step> old = new ArrayList<>(steps);
        steps.clear();
        for (double t : tapes) {
            Step s = new Step();
            s.tape = t;
            for (Step o : old) {
                if (o.recorded && o.tape == t) s = o;
            }
            steps.add(s);
        }
        step = Math.min(step, steps.size() - 1);
    }

    public synchronized void setGeometry(double spacing, double setback) {
        this.spacing = spacing;
        this.setback = setback;
    }

    public synchronized void setStarted(boolean started) {
        this.started = started;
    }

    public synchronized void setEstop(boolean e) {
        estop = e;
        if (e) message = "E-STOP latched, press Reset to drive again";
    }

    public synchronized boolean isEstop() {
        return estop;
    }

    public synchronized Stage stage() {
        return stage;
    }

    public synchronized Phase phase() {
        return phase;
    }

    public synchronized int step() {
        return step;
    }

    public synchronized int stepCount() {
        return steps.size();
    }

    public synchronized String message() {
        return message;
    }

    public synchronized void setMessage(String m) {
        message = m;
    }

    public synchronized double progress() {
        if (phase == Phase.SETTLING) return Math.min(1, (now - phaseStart) / settleSec);
        if (phase == Phase.RECORDING) return Math.min(1, (now - phaseStart) / recordSec);
        return 0;
    }

    public synchronized double tapeNow() {
        return steps.get(step).tape;
    }

    public synchronized double maxWobble() {
        double worst = 0;
        for (Step s : steps) {
            if (s.recorded) worst = Math.max(worst, Math.max(s.leftWobble, s.rightWobble));
        }
        return worst;
    }

    // [left, right], from the recorded steps so far
    public synchronized Fit[] fits() {
        int n = steps.size();
        double[] tape = new double[n], l = new double[n], r = new double[n];
        boolean[] use = new boolean[n];
        for (int i = 0; i < n; i++) {
            Step s = steps.get(i);
            tape[i] = s.tape;
            l[i] = s.left;
            r[i] = s.right;
            use[i] = s.recorded;
        }
        return new Fit[]{Fit.of(tape, l, use), Fit.of(tape, r, use)};
    }

    public synchronized boolean fitsSane() {
        Fit[] f = fits();
        return f[0].sane() && f[1].sane();
    }

    // the fix that's pasted in the code right now, so the screens can show it working
    public synchronized void setPastedFix(double[] k) {
        pasted = k.clone();
    }

    // worst miss of the recorded raw readings once the given fix is applied, [left, right]
    public synchronized double[] worstWithFix(double[] k) {
        double worstL = 0, worstR = 0;
        for (Step s : steps) {
            if (!s.recorded) continue;
            worstL = Math.max(worstL, Math.abs(k[0] * s.left + k[1] - s.tape));
            worstR = Math.max(worstR, Math.abs(k[2] * s.right + k[3] - s.tape));
        }
        return new double[]{worstL, worstR};
    }

    // scale/offset pairs for WallApproach.useCorrection
    public synchronized double[] correction() {
        Fit[] f = fits();
        return new double[]{f[0].scale, f[0].offset, f[1].scale, f[1].offset};
    }

    public synchronized void setDrive(boolean driving, String result, String note, double[] values) {
        this.driving = driving;
        driveResult = result;
        driveNote = note;
        if (values != null) drive = values;
    }

    public synchronized void addTrial(double measured, double thinks, String result) {
        Trial t = new Trial();
        t.n = trials.size() + 1;
        t.measured = measured;
        t.thinks = thinks;
        t.result = result;
        t.fix = usingFix;
        trials.add(t);
        saveTrials();
    }

    // ---------- JSON for the page, built by hand since org.json throws on NaN ----------

    private static String n(double d) {
        if (Double.isNaN(d) || Double.isInfinite(d)) return "null";
        return String.format(Locale.US, "%.4f", d);
    }

    private static String q(String s) {
        return "\"" + s.replace("\\", "\\\\").replace("\"", "\\\"").replace("\n", " ") + "\"";
    }

    public synchronized String snapshotJson(int since) {
        if (since < 0 || since > samples.size()) since = 0;
        StringBuilder b = new StringBuilder(4096);
        b.append("{\"sessionId\":").append(q(id));
        b.append(",\"stage\":").append(q(stage.name()));
        b.append(",\"step\":").append(step);
        b.append(",\"phase\":").append(q(phase.name()));
        b.append(",\"progress\":").append(n(progress()));
        b.append(",\"attempt\":").append(attempt);
        b.append(",\"t\":").append(n(now));
        b.append(",\"started\":").append(started);
        b.append(",\"message\":").append(q(message));
        b.append(",\"live\":{\"left\":").append(n(liveL)).append(",\"right\":").append(n(liveR)).append(",\"supply\":").append(n(liveV)).append("}");

        b.append(",\"config\":{\"spacing\":").append(n(spacing)).append(",\"setback\":").append(n(setback));
        b.append(",\"stopDistance\":").append(n(stopDistance)).append(",\"tooFar\":").append(n(tooFar));
        b.append(",\"goodEnough\":").append(n(goodEnough)).append(",\"jumpy\":").append(n(jumpy));
        b.append(",\"settleSec\":").append(n(settleSec)).append(",\"recordSec\":").append(n(recordSec));
        b.append(",\"pasted\":[").append(n(pasted[0])).append(",").append(n(pasted[1])).append(",").append(n(pasted[2])).append(",").append(n(pasted[3])).append("]}");

        b.append(",\"steps\":[");
        for (int i = 0; i < steps.size(); i++) {
            Step s = steps.get(i);
            if (i > 0) b.append(',');
            b.append("{\"tape\":").append(n(s.tape)).append(",\"recorded\":").append(s.recorded);
            b.append(",\"left\":").append(n(s.left)).append(",\"right\":").append(n(s.right));
            b.append(",\"leftWobble\":").append(n(s.leftWobble)).append(",\"rightWobble\":").append(n(s.rightWobble));
            b.append(",\"n\":").append(s.n).append('}');
        }
        b.append(']');

        Fit[] f = fits();
        b.append(",\"fit\":{\"left\":").append(fitJson(f[0])).append(",\"right\":").append(fitJson(f[1]));
        b.append(",\"maxWobble\":").append(n(maxWobble())).append('}');

        b.append(",\"drive\":{\"active\":").append(driving).append(",\"usingFix\":").append(usingFix);
        b.append(",\"estop\":").append(estop).append(",\"result\":").append(q(driveResult)).append(",\"note\":").append(q(driveNote));
        String[] keys = {"left", "right", "wall", "toGo", "angle", "drivePower", "strafePower", "turnPower", "bumper"};
        for (int i = 0; i < keys.length; i++) b.append(",\"").append(keys[i]).append("\":").append(n(drive[i]));
        b.append('}');

        b.append(",\"trials\":[");
        for (int i = 0; i < trials.size(); i++) {
            Trial t = trials.get(i);
            if (i > 0) b.append(',');
            b.append("{\"n\":").append(t.n).append(",\"thinks\":").append(n(t.thinks)).append(",\"measured\":").append(n(t.measured));
            b.append(",\"result\":").append(q(t.result)).append(",\"fix\":").append(t.fix).append('}');
        }
        b.append(']');

        // sample = [index, t, step, attempt, phase, counted, corrected, tape, left, right, supply]
        b.append(",\"nextIndex\":").append(samples.size()).append(",\"samples\":[");
        for (int i = since; i < samples.size(); i++) {
            Sample s = samples.get(i);
            if (i > since) b.append(',');
            b.append('[').append(i).append(',').append(n(s.t)).append(',').append(s.step).append(',').append(s.attempt);
            b.append(',').append(s.phase.ordinal()).append(',').append(s.counted ? 1 : 0).append(',').append(s.corrected ? 1 : 0);
            b.append(',').append(n(s.tape)).append(',').append(n(s.l)).append(',').append(n(s.r)).append(',').append(n(s.v)).append(']');
        }
        b.append("]}");
        return b.toString();
    }

    private String fitJson(Fit f) {
        return "{\"points\":" + f.points + ",\"scale\":" + n(f.scale) + ",\"offset\":" + n(f.offset)
                + ",\"r2\":" + n(f.r2) + ",\"rmse\":" + n(f.rmse)
                + ",\"worstBefore\":" + n(f.worstBefore) + ",\"worstAfter\":" + n(f.worstAfter)
                + ",\"sane\":" + f.sane() + ",\"verdict\":" + q(f.points >= 3 ? f.verdict() : "need at least 3 distances") + "}";
    }

    // ---------- CSV ----------

    private static String c(double d) {
        return Double.isNaN(d) ? "" : String.format(Locale.US, "%.4f", d);
    }

    private static String rawRow(Sample s) {
        return c(s.t) + "," + s.attempt + "," + (s.step + 1) + "," + s.phase.name() + "," + (s.counted ? 1 : 0) + ","
                + (s.corrected ? 1 : 0) + "," + c(s.tape) + "," + c(s.l) + "," + c(s.r) + "," + c(s.v);
    }

    public synchronized String rawCsv() {
        StringBuilder b = new StringBuilder(RAW_HEADER).append('\n');
        for (Sample s : samples) b.append(rawRow(s)).append('\n');
        return b.toString();
    }

    public synchronized String summaryCsv() {
        StringBuilder b = new StringBuilder(SUMMARY_HEADER).append('\n');
        for (int i = 0; i < steps.size(); i++) {
            Step s = steps.get(i);
            if (!s.recorded) continue;
            b.append(i + 1).append(',').append(c(s.tape)).append(',').append(c(s.left)).append(',').append(c(s.right)).append(',')
                    .append(c(s.leftWobble)).append(',').append(c(s.rightWobble)).append(',')
                    .append(c(s.left - s.tape)).append(',').append(c(s.right - s.tape)).append(',').append(s.n).append('\n');
        }
        return b.toString();
    }

    public synchronized String fitCsv() {
        Fit[] f = fits();
        StringBuilder b = new StringBuilder(FIT_HEADER).append('\n');
        String[] names = {"left", "right"};
        for (int i = 0; i < 2; i++) {
            b.append(names[i]).append(',').append(c(f[i].scale)).append(',').append(c(f[i].offset)).append(',')
                    .append(c(f[i].r2)).append(',').append(c(f[i].rmse)).append(',').append(c(f[i].worstBefore)).append(',')
                    .append(c(f[i].worstAfter)).append(',').append(f[i].points).append(',')
                    .append(c(spacing)).append(',').append(c(setback)).append('\n');
        }
        return b.toString();
    }

    public synchronized String trialsCsv() {
        StringBuilder b = new StringBuilder(TRIAL_HEADER).append('\n');
        for (Trial t : trials) {
            b.append(t.n).append(',').append(c(t.thinks)).append(',').append(c(t.measured)).append(',')
                    .append(c(t.thinks - t.measured)).append(',').append(t.result).append(',').append(t.fix ? 1 : 0).append('\n');
        }
        return b.toString();
    }

    private void saveSummary() {
        save("summary", summaryCsv());
        save("fit", fitCsv());
    }

    private void saveTrials() {
        save("trials", trialsCsv());
    }

    private void save(String kind, String text) {
        if (dir == null || stamp == null) return;
        try (FileWriter w = new FileWriter(new File(dir, "run_" + stamp + "_" + kind + ".csv"))) {
            w.write(text);
        } catch (Exception ignored) {
        }
    }
}
