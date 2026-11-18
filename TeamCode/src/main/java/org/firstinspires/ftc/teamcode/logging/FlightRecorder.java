package org.firstinspires.ftc.teamcode.logging;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.config.HardwareNames;
import org.firstinspires.ftc.teamcode.tuning.Tunables;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import java.util.Map;

// saves what the robot did during an opmode to a file on the hub, so you can replay it later.
// one file per run in /sdcard/FIRST/logs, the newest 30 are kept. get them with `make pull-logs`,
// look at them with `make viewer` (BuzzScope) or AdvantageScope.
//
// how to use it in an opmode:
//
//     FlightRecorder log = FlightRecorder.start(this);            // or start(this, follower) to also save the pose
//     while (opModeIsActive()) {
//         ...
//         log.put("Ultrasonic/leftIn", left);                     // any extra numbers you want, optional
//         log.update();                                           // once per loop, after the puts
//     }
//
// saved on its own: battery volts, hub current, both gamepads, loop time, drive motor power + speed, and
// "start" / "stop". it closes itself when the opmode ends.
//
// it also watches the numbers you can tune live in Panels (see tuning/Tunables.java): while one is different
// from the code, the driver hub shows COPY INTO CODE with the file and the new value, and the log gets a TUNED event.
//
// it is built so it can't slow down or stop the robot:
//   - the loop only copies numbers into memory, a background thread writes the file
//   - battery volts / current are read on that background thread too, 4 times a second
//   - anything that goes wrong in here turns the recorder off and the opmode keeps running
// the one thing it does to the hubs: a hub that has bulk caching OFF is switched to AUTO, so reading the 4 motor
// speeds is one hub read instead of 4. AUTO never gives stale values. hubs already in MANUAL (see
// BulkReads.manual) are left alone.
//
// channel names are what the tools in tools/ look for, see docs/LOGGING.md before renaming one
public final class FlightRecorder {

    // false = start() gives back a recorder that does nothing. the one switch to turn logging off everywhere
    public static boolean ENABLED = true;

    static final String LOG_DIR = "/sdcard/FIRST/logs";
    static final int MAX_LOG_FILES = 30;

    // order is what the tools expect: left front, right front, left rear, right rear
    private static final String[] DRIVE_MOTORS = {HardwareNames.FRONT_LEFT_MOTOR, HardwareNames.FRONT_RIGHT_MOTOR,
            HardwareNames.BACK_LEFT_MOTOR, HardwareNames.BACK_RIGHT_MOTOR};

    private static final long MOTOR_INTERVAL_NS = 20_000_000L;   // motors 50 times a second at most
    private static final long SLOW_READ_NS = 500_000L;   // a read this slow went to the hub instead of the cache
    private static final long TUNED_INTERVAL_NS = 250_000_000L;   // look for Panels changes 4 times a second

    // for AdvantageScope's field: it wants meters from the field center, turned a quarter turn from pedro's axes
    // this season (red wall on the audience's left)
    private static final double FIELD_LENGTH_IN = 141.5;
    private static final int FIELD_QUARTER_TURNS = 1;

    private final LinearOpMode opMode;
    private final Follower follower;
    private volatile LogCore core;   // null = off
    private final String kind;   // "auto" or "teleop", so BuzzScope shows the right match clock

    private List<LynxModule> hubs = new ArrayList<>();
    private String[] hubAmps = {}, hubVolts = {};
    private int nextHub;
    private final String[] powerNames = new String[3];
    private final double[] powerValues = new double[3];

    private DcMotorEx[] motors = {};
    private String[] motorPowerName = {}, motorVelocityName = {};   // null for the drive motors, they go in arrays
    private final int[] driveIndex = {-1, -1, -1, -1};
    private boolean haveDrive;
    private double[] power = {}, velocity = {};
    private final double[] drivePower = new double[4], driveVelocity = new double[4];
    private int nextPower;
    private long lastMotorNs;

    private final double[] axes = new double[6];
    private final double[] poseValues = new double[3], velocityValues = new double[3];
    private final byte[] fieldPose = new byte[24];

    private long lastUpdateNs, windowMaxLoopNs;
    private volatile boolean restartLoopTimer;
    private boolean startLogged, stopLogged;
    private int ticksSinceStop;

    // the tuned numbers already written to the log, so each change is only logged once
    private final Map<String, Tunables.Change> tunedLogged = new HashMap<>();
    private long lastTunedNs;
    private String tunedText = "";
    private Telemetry.Item tunedItem;

    // the SDK tells us the exact moment START is pressed and when the opmode is over
    private final OpModeManagerNotifier.Notifications listener = new OpModeManagerNotifier.Notifications() {
        @Override
        public void onOpModePreInit(OpMode o) {
        }

        @Override
        public void onOpModePreStart(OpMode o) {
            markStart();
        }

        @Override
        public void onOpModePostStop(OpMode o) {
            markStop();
            close();
        }
    };

    public static FlightRecorder start(LinearOpMode opMode) {
        return start(opMode, null);
    }

    // with a follower the robot's position is saved too, so BuzzScope can draw where it drove
    public static FlightRecorder start(LinearOpMode opMode, Follower follower) {
        FlightRecorder recorder = new FlightRecorder(opMode, follower);
        if (ENABLED) {
            try {
                recorder.open();
            } catch (Throwable t) {
                recorder.fail(t);
            }
        }
        return recorder;
    }

    private FlightRecorder(LinearOpMode opMode, Follower follower) {
        this.opMode = opMode;
        this.follower = follower;
        kind = opMode.getClass().isAnnotationPresent(Autonomous.class) ? "auto" : "teleop";
    }

    // remember a number for this loop. it's written when update() is called
    public void put(String name, double value) {
        try {
            LogCore c = core;
            if (c != null) c.put(name, value);
        } catch (Throwable t) {
            fail(t);
        }
    }

    public void put(String name, boolean value) {
        try {
            LogCore c = core;
            if (c != null) c.put(name, value);
        } catch (Throwable t) {
            fail(t);
        }
    }

    // text, like a status. only written when it changes
    public void put(String name, String value) {
        try {
            LogCore c = core;
            if (c != null) c.put(name, value);
        } catch (Throwable t) {
            fail(t);
        }
    }

    public void put(String name, double[] values) {
        try {
            LogCore c = core;
            if (c != null && values != null) c.put(name, values);
        } catch (Throwable t) {
            fail(t);
        }
    }

    // for opmodes without a pedro follower that still know where the robot is. inches and radians
    public void pose(double xIn, double yIn, double headingRad) {
        try {
            LogCore c = core;
            if (c == null) return;
            poseValues[0] = xIn;
            poseValues[1] = yIn;
            poseValues[2] = headingRad;
            c.put("pose", poseValues);
            encodeFieldPose(xIn, yIn, headingRad);
            c.putRaw("Field/Robot", "struct:Pose2d", fieldPose);
        } catch (Throwable t) {
            fail(t);
        }
    }

    // something that happened, saved with the exact time. shows up in BuzzScope's event list
    public void event(String message) {
        try {
            LogCore c = core;
            if (c != null) c.event(message);
        } catch (Throwable t) {
            fail(t);
        }
    }

    // call once per loop, after the puts
    public void update() {
        LogCore c = core;
        if (c == null) return;
        try {
            long now = System.nanoTime();
            if (opMode.isStarted()) markStart();
            if (opMode.isStopRequested()) markStop();
            // the wait for START isn't a slow loop, so the first loop after it doesn't count
            if (restartLoopTimer) {
                restartLoopTimer = false;
                lastUpdateNs = 0;
                windowMaxLoopNs = 0;
            }
            long loopNs = lastUpdateNs == 0 ? 0 : now - lastUpdateNs;
            lastUpdateNs = now;
            windowMaxLoopNs = Math.max(windowMaxLoopNs, loopNs);
            if (now - lastTunedNs >= TUNED_INTERVAL_NS) {
                lastTunedNs = now;
                checkTuned(c);
            }

            if (!c.sampleDue()) {
                c.discard();
                return;
            }
            c.put("opMode/kind", kind);   // text is only written when it changes, so this goes in the file once
            c.put("loop/totalNanos", loopNs);
            c.put("loop/windowMaxTotalNanos", windowMaxLoopNs);   // the slowest loop since the last sample
            windowMaxLoopNs = 0;
            gamepad(c, opMode.gamepad1, "gamepad1/axes", "gamepad1/buttons");
            gamepad(c, opMode.gamepad2, "gamepad2/axes", "gamepad2/buttons");
            if (now - lastMotorNs >= MOTOR_INTERVAL_NS) {
                lastMotorNs = now;
                sampleMotors(c);
            }
            if (follower != null) samplePose(c);
            c.commit();
        } catch (Throwable t) {
            fail(t);
        }
    }

    // finishes the file. you don't need to call this, it happens by itself when the opmode ends
    public void close() {
        try {
            LogCore c = core;
            core = null;
            if (c != null) c.close();
        } catch (Throwable ignored) {
        }
    }

    private void open() throws IOException {
        File dir = new File(LOG_DIR);
        dir.mkdirs();
        prune(dir, MAX_LOG_FILES - 1);
        String name = opMode.getClass().getSimpleName();
        String stamp = new SimpleDateFormat("yyyyMMdd-HHmmss-SSS", Locale.US).format(new Date());
        File file = File.createTempFile(name + "-" + stamp + "-", ".wpilog", dir);
        final FileOutputStream out = new FileOutputStream(file);
        LogCore.Sink sink = new LogCore.Sink() {
            @Override
            public void write(byte[] data, int length) throws IOException {
                out.write(data, 0, length);
            }

            @Override
            public void sync() throws IOException {
                out.getFD().sync();
            }

            @Override
            public void close() throws IOException {
                out.close();
            }
        };

        findHubs();
        findMotors();
        LogCore c = new LogCore(System::nanoTime, sink, "23209-biobuzz", this::background);
        c.schema("Translation2d", "double x;double y;");
        c.schema("Rotation2d", "double value;");
        c.schema("Pose2d", "Translation2d translation;Rotation2d rotation;");
        c.event("init " + name);
        core = c;
        checkTuned(c);   // anything already changed in Panels before this opmode goes in the log right away

        try {
            OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getActivity()).registerListener(listener);
        } catch (Throwable t) {
            // no listener = update() and the background thread still notice start and stop, just a little later
            RobotLog.ww("FlightRecorder", t, "no opmode listener");
        }
    }

    // deletes the oldest logs so at most `keep` are left
    static void prune(File dir, int keep) {
        File[] logs = dir.listFiles((d, n) -> n.endsWith(".wpilog"));
        if (logs == null || logs.length <= keep) return;
        Arrays.sort(logs, (a, b) -> Long.compare(a.lastModified(), b.lastModified()));
        for (int i = 0; i < logs.length - keep; i++) logs[i].delete();
    }

    private void findHubs() {
        List<LynxModule> all = opMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : all) if (hub.isParent()) hubs.add(hub);   // control hub first
        for (LynxModule hub : all) if (!hub.isParent()) hubs.add(hub);
        hubAmps = new String[hubs.size()];
        hubVolts = new String[hubs.size()];
        for (int i = 0; i < hubs.size(); i++) {
            LynxModule hub = hubs.get(i);
            String label = hub.isParent() ? "control" : "expansion" + hub.getModuleAddress();
            hubAmps[i] = "hubs/" + label + "/currentAmps";
            hubVolts[i] = "hubs/" + label + "/volts";
            if (hub.getBulkCachingMode() == LynxModule.BulkCachingMode.OFF) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
        }
    }

    private void findMotors() {
        List<DcMotorEx> found = new ArrayList<>();
        List<String> names = new ArrayList<>();
        for (int i = 0; i < DRIVE_MOTORS.length; i++) {
            DcMotorEx m = opMode.hardwareMap.tryGet(DcMotorEx.class, DRIVE_MOTORS[i]);
            if (m == null) continue;
            driveIndex[i] = found.size();
            found.add(m);
            names.add(DRIVE_MOTORS[i]);
        }
        int driveCount = found.size();
        haveDrive = driveCount == 4;
        for (DcMotorEx m : opMode.hardwareMap.getAll(DcMotorEx.class)) {
            if (found.contains(m)) continue;
            String name = "motor" + found.size();
            for (String n : opMode.hardwareMap.getNamesOf(m)) {
                name = n;
                break;
            }
            found.add(m);
            names.add(name);
        }
        motors = found.toArray(new DcMotorEx[0]);
        power = new double[motors.length];
        velocity = new double[motors.length];
        motorPowerName = new String[motors.length];
        motorVelocityName = new String[motors.length];
        for (int i = 0; i < motors.length; i++) {
            // the 4 drive motors go in the Drive/motors arrays. everything else gets its own channels
            if (haveDrive && i < driveCount) continue;
            motorPowerName[i] = "Motors/" + names.get(i) + "/power";
            motorVelocityName[i] = "Motors/" + names.get(i) + "/velocityTps";
        }
    }

    private void gamepad(LogCore c, Gamepad g, String axesName, String buttonsName) {
        if (g == null) return;
        axes[0] = g.left_stick_x;
        axes[1] = -g.left_stick_y;   // + is up
        axes[2] = g.right_stick_x;
        axes[3] = -g.right_stick_y;
        axes[4] = g.left_trigger;
        axes[5] = g.right_trigger;
        c.put(axesName, axes);
        // one bit per button, in the order BuzzScope draws them
        long bits = 0;
        if (g.a) bits |= 1;
        if (g.b) bits |= 1 << 1;
        if (g.x) bits |= 1 << 2;
        if (g.y) bits |= 1 << 3;
        if (g.left_bumper) bits |= 1 << 4;
        if (g.right_bumper) bits |= 1 << 5;
        if (g.dpad_up) bits |= 1 << 6;
        if (g.dpad_down) bits |= 1 << 7;
        if (g.dpad_left) bits |= 1 << 8;
        if (g.dpad_right) bits |= 1 << 9;
        if (g.start) bits |= 1 << 10;
        if (g.back) bits |= 1 << 11;
        if (g.left_stick_button) bits |= 1 << 12;
        if (g.right_stick_button) bits |= 1 << 13;
        if (g.touchpad) bits |= 1 << 14;
        c.put(buttonsName, bits);
    }

    private void sampleMotors(LogCore c) {
        if (motors.length == 0) return;
        // speed comes out of the hub's bulk read, so all of these together are at most one hub read
        for (int i = 0; i < motors.length; i++) velocity[i] = motors[i].getVelocity();
        // power is remembered by the SDK for half a second after it's set, so reading it is free while the motor
        // is being driven. for a motor nobody has set in a while it's a real hub read (~2 ms), so after one of
        // those stop for this loop and carry on from the next motor next time
        for (int k = 0; k < motors.length; k++) {
            int i = (nextPower + k) % motors.length;
            long t0 = System.nanoTime();
            power[i] = motors[i].getPower();
            if (System.nanoTime() - t0 > SLOW_READ_NS) {
                nextPower = (i + 1) % motors.length;
                break;
            }
        }
        if (haveDrive) {
            for (int i = 0; i < 4; i++) {
                drivePower[i] = power[driveIndex[i]];
                driveVelocity[i] = velocity[driveIndex[i]];
            }
            c.put("Drive/motors/power", drivePower);
            c.put("Drive/motors/velocityTps", driveVelocity);
        }
        for (int i = 0; i < motors.length; i++) {
            if (motorPowerName[i] == null) continue;
            c.put(motorPowerName[i], power[i]);
            c.put(motorVelocityName[i], velocity[i]);
        }
    }

    private void samplePose(LogCore c) {
        Pose p = follower.pose();
        if (p != null) pose(p.x(), p.y(), p.heading());
        Velocity v = follower.velocity();
        if (v != null) {
            velocityValues[0] = v.vx;
            velocityValues[1] = v.vy;
            velocityValues[2] = Double.isNaN(v.omega) || Double.isInfinite(v.omega) ? 0 : v.omega;
            c.put("velocity", velocityValues);
        }
    }

    // pedro inches from the field corner -> meters from the field center, turned to match the FTC field drawing
    private void encodeFieldPose(double xIn, double yIn, double headingRad) {
        double x = finite(xIn) ? (xIn - FIELD_LENGTH_IN / 2) * 0.0254 : 0;
        double y = finite(yIn) ? (yIn - FIELD_LENGTH_IN / 2) * 0.0254 : 0;
        double fx, fy;
        switch (((FIELD_QUARTER_TURNS % 4) + 4) % 4) {
            case 0: fx = x; fy = y; break;
            case 1: fx = -y; fy = x; break;
            case 2: fx = -x; fy = -y; break;
            default: fx = y; fy = -x; break;
        }
        double heading = 0;
        if (finite(headingRad)) {
            heading = (headingRad + FIELD_QUARTER_TURNS * Math.PI / 2) % (2 * Math.PI);
            if (heading < 0) heading += 2 * Math.PI;
        }
        putDouble(fieldPose, 0, fx + 0.0);
        putDouble(fieldPose, 8, fy + 0.0);
        putDouble(fieldPose, 16, heading);
    }

    private static boolean finite(double v) {
        return !Double.isNaN(v) && !Double.isInfinite(v);
    }

    private static void putDouble(byte[] buffer, int offset, double value) {
        long bits = Double.doubleToRawLongBits(value);
        for (int i = 0; i < 8; i++) buffer[offset + i] = (byte) (bits >>> (8 * i));
    }

    // runs on the recorder's background thread about 4 times a second: battery, and a second way to notice
    // start/stop in case the loop isn't calling update()
    private void background() {
        LogCore c = core;
        if (c == null) return;
        if (opMode.isStarted()) markStart();
        if (opMode.isStopRequested()) {
            markStop();
            // give the loop a moment to log its last values before the file is finished
            if (++ticksSinceStop >= 2) close();
            return;
        }
        if (hubs.isEmpty()) return;
        int i = nextHub;
        nextHub = (nextHub + 1) % hubs.size();
        LynxModule hub = hubs.get(i);
        // each of these is its own ~2 ms hub read, which is why they're here and not in the robot loop
        double amps = hub.getCurrent(CurrentUnit.AMPS);
        double volts = hub.getInputVoltage(VoltageUnit.VOLTS);
        if (volts <= 0) return;   // a hub reads 0 V once the opmode has stopped, that's not the battery
        // volts and amps go in with the same timestamp, the battery math in BuzzScope pairs them up by time
        powerNames[0] = hubAmps[i];
        powerNames[1] = hubVolts[i];
        powerNames[2] = "battery";
        powerValues[0] = amps;
        powerValues[1] = volts;
        powerValues[2] = volts;
        c.record(powerNames, powerValues, i == 0 ? 3 : 2);   // "battery" = the control hub's volts
    }

    private synchronized void markStart() {
        if (startLogged) return;
        startLogged = true;
        restartLoopTimer = true;
        LogCore c = core;
        if (c != null) c.event("start");
    }

    private synchronized void markStop() {
        if (stopLogged) return;
        stopLogged = true;
        LogCore c = core;
        if (c == null) return;
        // the whole list once more at the end, so the last TUNED lines in a log are everything left to copy
        for (Tunables.Change change : Tunables.changes()) c.event(change.event());
        c.event("stop");
    }

    // looks for numbers changed in Panels. new or changed ones go in the log, and the driver hub gets one item
    // that stays on screen listing what to copy into which file. only called from the opmode's own thread
    private void checkTuned(LogCore c) {
        List<Tunables.Change> changes = Tunables.changes();
        if (changes.isEmpty() && tunedLogged.isEmpty()) return;

        StringBuilder text = new StringBuilder();
        Map<String, Tunables.Change> current = new HashMap<>();
        for (Tunables.Change change : changes) {
            String key = change.file + " " + change.owner + "." + change.field;
            current.put(key, change);
            Tunables.Change before = tunedLogged.get(key);
            if (before == null || !before.now.equals(change.now)) c.event(change.event());
            text.append("\n  ").append(change.file).append("   ").append(change.field).append(" = ").append(change.now)
                    .append("   (was ").append(change.code).append(")");
        }
        // put back to what the code says: say so in the log, so the tools don't keep asking for it to be copied
        for (Iterator<Map.Entry<String, Tunables.Change>> it = tunedLogged.entrySet().iterator(); it.hasNext(); ) {
            Map.Entry<String, Tunables.Change> e = it.next();
            if (current.containsKey(e.getKey())) continue;
            c.event("TUNED " + e.getKey() + " " + e.getValue().code + " -> " + e.getValue().code);
            it.remove();
        }
        tunedLogged.putAll(current);

        String now = text.toString();
        if (now.equals(tunedText)) return;
        tunedText = now;
        Telemetry telemetry = opMode.telemetry;
        if (telemetry == null) return;
        if (now.isEmpty()) {
            if (tunedItem != null) telemetry.removeItem(tunedItem);
            tunedItem = null;
        } else if (tunedItem == null) {
            // retained = it stays on the driver hub every update until it's removed, the opmode doesn't have to add it
            tunedItem = telemetry.addData("COPY INTO CODE", now).setRetained(true);
        } else {
            tunedItem.setValue(now);
        }
    }

    // logging must never stop the robot: write down why, turn the recorder off, carry on
    private void fail(Throwable t) {
        try {
            RobotLog.ee("FlightRecorder", t, "flight recorder turned off");
        } catch (Throwable ignored) {
        }
        try {
            close();
        } catch (Throwable ignored) {
        }
    }
}
