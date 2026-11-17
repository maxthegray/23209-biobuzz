package org.firstinspires.ftc.teamcode.opmodes.auto.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.logging.FlightRecorder;
import org.firstinspires.ftc.teamcode.mechanisms.PointDrive;
import org.firstinspires.ftc.teamcode.mechanisms.WallApproach;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.List;

// all 6 autos (red/blue, audience start tipped/not tipped, rear start). no shooter or intake yet so those are just waits
// poses are red side in pedro visualizer coords, blue gets mirrored

//@Disabled
@Autonomous(name = "Path Demo (all combos)", group = "teaching")
public class PathDemoAllCombos extends LinearOpMode {

    private static final double fieldSize = 141.5;
    private static final double hiveX = 58;   // middle of the red hive

    // the robot is 40 x 40 cm (15.75 in), pinpoint in the middle, so the center is 7.875 in from each edge
    private static final double halfRobot = 7.875;

    // the limelight and ultrasonic mounts stick out 2 in past the front of the frame, so the front needs that much more room than the sides and back
    private static final double frontMounts = 2.0;
    private static final double frontReach = halfRobot + frontMounts;

    private static final Pose startPose = pose(hiveX, halfRobot, 90);   // back flush against the wall
    private static final Pose audienceShootPose = pose(hiveX, 5 + halfRobot, 90);   // 5 in off the wall so it doesn't scrape
    private static final Pose rearShootPose = pose(hiveX, 116, -90);
    private static final Pose audienceStandoff = pose(hiveX, 21, 90);   // if it's still turning it waits out here, not by the wall

    private static final Pose gardenTurnSpot = new Pose(36, 30);   // spin around out here, nowhere near a wall
    private static final Pose gardenLineup = pose(6 + halfRobot, 27, -90);   // 6 in off the side wall so it can't slide into it
    private static final Pose gardenPickup = pose(3 + halfRobot, 4 + frontReach, -90);   // side 3 in off the side wall, front mounts 4 in off the audience wall
    private static final Pose gardenBackup = new Pose(24, 30);   // these two are pass-through spots on the way round to the rear, it doesn't stop at them
    private static final Pose swoopCorner = new Pose(22, 106);

    // flowers sit at 46.75 along their wall (manual). the sensors are only 7.5 in apart and a flower is ~6.3 in wide,
    // so being even an inch off center puts a sensor on the flower
    private static final double flowerCenter = 46.75;
    // a flower's front is 5.4 in off the wall. these are frame-to-wall distances, the mounts are 2 in closer than that
    private static final Pose sideFlowerLineup = pose(17 + halfRobot, flowerCenter, 180);   // frame 17 in off the wall
    private static final Pose sideFlowerPickup = pose(10.4 + halfRobot, flowerCenter, 180);   // frame 10.4 in off, so the mounts are 3 in from the flower. the ultrasonics do the last 2 in
    private static final Pose rearFlowerLineup = pose(flowerCenter, fieldSize - 17 - halfRobot, 90);
    private static final Pose rearFlowerPickup = pose(flowerCenter, fieldSize - 10.4 - halfRobot, 90);
    private static final Pose climbCorner1 = new Pose(24, 30);   // pass-through spots that take it round the left of the hive
    private static final Pose climbCorner2 = new Pose(20, 104);

    // rear start: back on the rear wall, side 3 in clear of the rear flower, facing the hive. shoots, then the flower is right there
    private static final Pose rearStartPose = pose(flowerCenter + 3.15 + 3 + halfRobot, fieldSize - halfRobot, -90);
    private static final Pose rearStartTurnSpot = new Pose(62, 112);   // turns around out here, clear of the wall and the flower
    private static final Pose sideCorner = new Pose(16, 112);   // pass-through spot between the rear shot and the side flower. high up so the front mounts clear the hive corner

    // private static final Pose climbControl2 = new Pose(18, 118);

    private static final Pose parkPose = pose(4 + frontReach, 108, -90);   // 4 in off the side wall even if the front is facing it
    private static final Pose parkWaypoint = new Pose(24, 40);   // straight line from down here would hit the hive or the side flower

    private static final double autoLength = 30;
    private static final double stopAt = 29.5;
    private static final double avgSpeed = 30;   // in/s, rough guess for how long parking takes
    private static final double parkBuffer = 1.5;
    private static final double passRadius = 14;   // in. this close to a pass-through spot counts, then it heads for the next one without slowing

    private static final long shotMs = 700;
    private static final long intakeMs = 1500;
    private static final long tipCheckMs = 1100;
    private static final int maxBalls = 4;

    private static final double nominalVoltage = 12.0;   // every path drives like it's on a 12 V battery
    private static final double lowBattery = 12.5;   // resting voltage under this, swap it before the run

    private static final double bumperToCenter = halfRobot;
    private static final boolean fixPoseAtWall = false;   // true = let the wall reading correct pedro's position after the adjust
    private static final double maxPoseFix = 4.0;   // ultrasonics disagree with pedro by more than this = don't trust them
    private static final double maxAdjust = 8.0;   // the adjust can't take it further than this from pedro's pickup spot (it slides up to ~6.5 in sideways to find the flower)

    private Follower follower;
    private FlightRecorder log;   // saves the run to a file on the hub, see logging/FlightRecorder.java
    private BatteryDrive drive;
    private WallApproach sonar;
    private final PointDrive pointDrive = new PointDrive();
    private boolean approaching = false;
    private String sonarResult = "-";
    private String stepNote = "";
    private final ElapsedTime timer = new ElapsedTime();
    private boolean isBlue = false;
    private boolean hiveTipped = true;
    private boolean rearStart = false;
    private boolean loose = false;
    private boolean parking = false;
    private int balls = 4;
    private String status = "Starting";
    private final List<String> stepLog = new ArrayList<>();
    private double lastStepTime = 0;
    private int timesStuck = 0;
    private Pose lastTarget;
    private String stopError = "-";

    @Override
    public void runOpMode() {
        // same as Constants.create() but with the battery scaled drive, so it covers every path, spin and hold
        drive = new BatteryDrive(hardwareMap);
        follower = new Follower(new PinpointLocalizer(hardwareMap, Constants.localizerConfig), drive, new Foresight(Constants.foresightConfig));
        try {
            sonar = new WallApproach(hardwareMap);
        } catch (IllegalArgumentException e) {
            sonar = null;   // not in the robot config, flowers just use pedro
        }
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        log = FlightRecorder.start(this, follower);

        while (opModeInInit()) {
            if (gamepad1.dpad_left) isBlue = false;
            if (gamepad1.dpad_right) isBlue = true;
            if (gamepad1.dpad_up) hiveTipped = true;
            if (gamepad1.dpad_down) hiveTipped = false;

            if (gamepad1.left_bumper) rearStart = false;
            if (gamepad1.right_bumper) rearStart = true;

            telemetry.addData("Alliance", isBlue ? "Blue" : "Red");
            telemetry.addData("Start", rearStart ? "Far wall (shoot, then the flower next to it)" : "Hive wall");
            telemetry.addData("Hive", rearStart ? "not used for this start" : hiveTipped ? "Tipped" : "Not tipped");
            telemetry.addLine("Dpad left/right = alliance, up/down = tipped, LB/RB = start wall");
            if (rearStart) telemetry.addLine(isBlue ? "Back against the audience wall, facing the blue hive, side 3 in from the flower"
                    : "Back against the rear wall, facing the red hive, side 3 in from the flower");
            else telemetry.addLine(isBlue ? "Back against the rear wall, centered on the blue hive"
                    : "Back against the audience wall, centered on the red hive");
            drive.readBattery();
            telemetry.addData("Battery", "%.1f V%s", drive.voltage, drive.voltage < lowBattery ? "  LOW, swap it" : "");
            if (sonar != null) sonar.showReadings(telemetry);
            else telemetry.addLine("Ultrasonics not in the config, flowers use pedro only");
            telemetry.update();
            log.update();
        }
        if (isStopRequested()) return;

        timer.reset();
        Pose start = mirrorIfBlue(rearStart ? rearStartPose : startPose);
        follower.setPose(start);
        lastTarget = start;

        if (!runPlan()) logStep("Out of time");
        park();
        logStep("Park");

        status = "Parked";
        waitMs((long) Math.max(0, (stopAt - timer.seconds()) * 1000));
        follower.stop();
        follower.update();
    }

    // false = ran out of time partway
    private boolean runPlan() {
        if (!step("Shoot preloads", shoot())) return false;
        if (!step("Tip check", checkTip())) return false;

        if (rearStart) {
            if (!step("Rear flower", pickUpFlower(rearStartTurnSpot, rearFlowerPickup, 12))) return false;   // 12 in off the wall before it turns
            if (!step("Back to rear shot", backToRearShot())) return false;
            if (!step("Shoot flower balls", shoot())) return false;
            if (!step("Side flower", pickUpFlower(sideFlowerLineup, sideFlowerPickup, 40, sideCorner))) return false;   // turns once it is past the hive
        } else if (hiveTipped) {
            if (!step("Garden lineup", lineUpForGarden())) return false;
            if (!step("Garden", intakeFromGarden())) return false;
            if (!step("Swoop to rear", swoopToRearShoot())) return false;
            if (!step("Shoot garden balls", shoot())) return false;
            if (!step("Rear flower", pickUpFlower(rearFlowerLineup, rearFlowerPickup, 0))) return false;
        } else {
            if (!step("Side flower", pickUpFlower(sideFlowerLineup, sideFlowerPickup, 12))) return false;   // 12 in along before it turns, so the corners clear the wall
            if (!step("Back to shoot", backToAudienceShot())) return false;
            if (!step("Shoot again", shoot())) return false;
            if (!step("Rear flower", pickUpFlower(rearFlowerLineup, rearFlowerPickup, 0, climbCorner1, climbCorner2))) return false;
        }

        // only skips the last shot if there isn't time for even one ball. otherwise it shoots what it can and leaves in time to park
        double driveTime = 2 + mirrorIfBlue(follower.pose()).distance(rearShootPose) / 35;
        double shotNeeds = driveTime + shotMs / 1000.0 + parkTimeFrom(rearShootPose);
        if (autoLength - timer.seconds() < shotNeeds) {
            logStep(String.format("No time for last shot (needs %.0f s)", shotNeeds));
            return true;
        }
        if (!step("Back to rear shot", backToRearShot())) return false;
        return step("Shoot last balls", shoot());
    }

    private boolean shoot() {
        // shooter code goes here once it's mounted
        while (balls > 0) {
            status = "Shooting (simulated), " + balls + " left";
            if (!waitMs(shotMs)) return false;
            balls--;
        }
        return true;
    }

    // will use the limelight later, for now this just uses what was picked in init
    private boolean checkTip() {
        status = "Tip check (simulated): " + (hiveTipped ? "tipped" : "not tipped");
        return waitMs(tipCheckMs);
    }

    // one move from wherever it is to the flower, turning on the way. it only goes past the lineup spot once it's facing the flower
    // then the ultrasonics adjust. if they give up it just stays where pedro put it
    private boolean pickUpFlower(Pose lineup, Pose pickup, double turnAfter, Pose... passThrough) {
        status = "To the flower";
        if (!glideTo(pickup, lineup, turnAfter, false, passThrough)) return false;
        if (!adjustAtFlower(pickup)) return false;
        status = "Intaking (simulated)";
        if (!waitMs(intakeMs)) return false;
        balls = maxBalls;
        return true;
    }

    // squares up so both sensors read the same, then creeps to the stop distance. false = out of time
    private boolean adjustAtFlower(Pose redPickup) {
        if (sonar == null) {
            sonarResult = "no sensors, pedro only";
            stepNote = " pedro";
            return true;
        }
        Pose pickup = mirrorIfBlue(redPickup);
        status = "Ultrasonic adjust";
        approaching = true;
        sonar.start();
        WallApproach.Result result = WallApproach.Result.RUNNING;
        boolean wandered = false;
        while (opModeIsActive() && result == WallApproach.Result.RUNNING && !outOfTime()) {
            result = sonar.update(follower, leftOf(pickup));
            follower.update();
            wandered = follower.pose().distance(pickup) > maxAdjust;   // bad readings would drive it off somewhere, pedro is the boss
            if (wandered) break;
            updateTelemetry();
        }
        approaching = false;
        double shift = sonar.flowerShift();
        String shiftNote = Math.abs(shift) < 0.1 ? "" : String.format(" slid %.1f %s", Math.abs(shift), shift > 0 ? "left" : "right");
        if (result == WallApproach.Result.DONE) {
            stepNote = " sonar" + shiftNote;
            fixPoseFromWall();
            return true;
        }
        follower.manual(0, 0, 0);
        sonarResult = wandered ? "wanted to go too far, back to pedro's spot" : "gave up, staying here";
        stepNote = " sonar failed" + shiftNote;
        if (wandered) return glideTo(redPickup, null, 0, false);
        return opModeIsActive() && !outOfTime();
    }

    // how far the pickup spot is to the robot's left. comes from pedro since the ultrasonics only see ahead
    private double leftOf(Pose spot) {
        Pose p = follower.pose();
        double dx = spot.x() - p.x(), dy = spot.y() - p.y();
        return -Math.sin(p.heading()) * dx + Math.cos(p.heading()) * dy;
    }

    // the wall doesn't move, so trust the ultrasonics over the pinpoint for how far we are from it
    private void fixPoseFromWall() {
        Pose p = follower.pose();
        double fromWall = sonar.bumperToWall() + bumperToCenter;
        double c = Math.cos(p.heading()), s = Math.sin(p.heading());
        Pose fixed;
        if (Math.abs(c) > Math.abs(s)) fixed = p.withX(c > 0 ? fieldSize - fromWall : fromWall);
        else fixed = p.withY(s > 0 ? fieldSize - fromWall : fromWall);

        double off = fixed.distance(p);
        if (!fixPoseAtWall) {
            sonarResult = String.format("done, pedro was off %.1f in (not fixed)", off);
        } else if (off > maxPoseFix) {
            sonarResult = String.format("done, but pedro was off %.1f in so left it", off);
        } else {
            follower.setPose(fixed);
            sonarResult = String.format("done, fixed pedro by %.1f in", off);
        }
    }

    // heads for the turn spot while it turns around, then on to the lineup without stopping
    private boolean lineUpForGarden() {
        status = "Lining up for the garden";
        return glideTo(gardenLineup, gardenTurnSpot, 12, true);
    }

    private boolean backToAudienceShot() {
        status = "Back to shoot";
        return glideTo(audienceShootPose, audienceStandoff, 6, false);   // backs off the flower 6 in before it turns
    }
    // private boolean backToAudienceShot() {
    //   status = "Back to rear shotting";
    //   if (!turnTo(audienceShootPose.heading())) return false;
    //   return driveTo(audienceShootPose);
    //   return driveTo(audienceShootPose);
    //   if (!turnTo(gardenLineup.heading())) return false;
    //   status = "returned to back";
    //   return driveTo != averageOfPos * x + averageOfPos * y + averageOfPos* z
    //   stableAverage = (!driveTo * x) + (averageOfPos * y) + (averageOfPos * z)
    //   stableAverage2 = (!driveTo * x) + (averageOfPos* y) + (averageOfPos * z)
    //   stableAverage3 =
    // }

    // from the rear flower it's right there. from the side flower it goes up the left side first
    private boolean backToRearShot() {
        status = "Back to the rear shot";
        boolean fromSide = mirrorIfBlue(follower.pose()).y() < sideCorner.y();
        if (fromSide) return glideTo(rearShootPose, null, 6, false, sideCorner);
        return glideTo(rearShootPose, null, 7, false);   // backs off the flower 7 in before it turns
    }

    private boolean intakeFromGarden() {
        status = "Driving in";
        if (!glideLoose(gardenPickup)) return false;
        status = "Intaking (simulated)";
        if (!waitMs(intakeMs)) return false;
        balls = maxBalls;
        return true;
    }

    // one move from the garden round the left of the hive to the rear shot
    private boolean swoopToRearShoot() {
        status = "Swooping to the rear";
        return glideTo(rearShootPose, null, 0, false, gardenBackup, swoopCorner);
    }

    // parks facing whichever way is closest, turning around first just burns time
    private void park() {
        parking = true;
        status = "Parking";
        Pose here = mirrorIfBlue(follower.pose());
        double facing = Math.round(here.heading() / (Math.PI / 2)) * (Math.PI / 2);
        if (here.y() < parkWaypoint.y()) glideTo(parkWaypoint.withHeading(facing), null, 0, true);
        glideLoose(parkPose.withHeading(facing));
    }

    // every move is one continuous motion: it drives and turns together and slows down as it gets close, see PointDrive
    // turnAfter = inches it drives before it starts turning, so the corners have room to swing
    // standoff = where it heads while it's still turning (null = straight to the target). it only goes on to the target once it's square
    //   a standoff right in front of the target (flower lineups) also has to be reached first, so the last bit is always straight in and slow
    // rough = just passing through, so close is good enough and it doesn't wait to stop
    // passThrough = spots to go by on the way, in order. it never slows for these
    // for spots right by a wall. within 1.5 in is fine there, pushing for the last half inch just grinds it into the wall
    private boolean glideLoose(Pose redTarget) {
        loose = true;
        boolean ok = glideTo(redTarget, null, 0, false);
        loose = false;
        return ok;
    }

    private boolean glideTo(Pose redTarget, Pose redStandoff, double turnAfter, boolean rough, Pose... passThrough) {
        Pose[] spots = new Pose[passThrough.length];
        for (int i = 0; i < spots.length; i++) spots[i] = mirrorIfBlue(passThrough[i]);
        Pose target = mirrorIfBlue(redTarget);
        Pose standoff = redStandoff == null ? null : mirrorIfBlue(redStandoff);
        lastTarget = target;

        Pose start = follower.pose();
        double dist = 0;
        Pose from = start;
        for (Pose spot : spots) {
            dist += from.distance(spot);
            from = spot;
        }
        dist += from.distance(target);
        double turn = Math.abs(angleTo(target.heading(), start.heading()));
        boolean bigTurn = turn > Math.toRadians(150);   // a 180 goes ccw through 90 first so it can't flip between left and right
        turnAfter = Math.min(turnAfter, dist / 2);
        double limit = 2 + dist / 12 + Math.toDegrees(turn) / 60;   // gives up after this so the auto never hangs

        pointDrive.start();
        ElapsedTime time = new ElapsedTime();
        int next = 0;
        boolean passedStandoff = standoff == null || standoff.distance(target) > 10;
        boolean there = false;
        while (opModeIsActive() && !there && !pointDrive.blocked() && time.seconds() < limit) {
            if (outOfTime()) return false;
            Pose p = follower.pose();
            while (next < spots.length && p.distance(spots[next]) < passRadius) next++;
            double turnLeft = Math.abs(angleTo(target.heading(), p.heading()));
            boolean finalHeading = p.distance(start) >= turnAfter && !(bigTurn && turnLeft > Math.toRadians(100));
            double facing = finalHeading ? target.heading() : p.distance(start) < turnAfter ? start.heading() : start.heading() + Math.PI / 2;
            if (next < spots.length) pointDrive.updateRough(follower, spots[next].withHeading(facing));
            else if (standoff != null && (turnLeft > Math.toRadians(15) || !passedStandoff)) {
                pointDrive.updateRough(follower, standoff.withHeading(facing));
                if (p.distance(standoff) < 6) passedStandoff = true;
            }
            else if (rough || !finalHeading) there = pointDrive.updateRough(follower, target.withHeading(facing)) && finalHeading;
            else there = loose ? pointDrive.updateLoose(follower, target) : pointDrive.update(follower, target);
            follower.update();
            updateTelemetry();
        }
        if (!there) timesStuck++;
        if (!rough || !there) {
            follower.stop();   // wheels off between moves no matter how it ended
            follower.update();
        }
        Pose p = follower.pose();
        stopError = String.format("%.1f in, %.0f deg", p.distance(target), Math.abs(Math.toDegrees(angleTo(target.heading(), p.heading()))));
        return opModeIsActive();
    }

    // cuts the wheels and waits. still calls update() so the pinpoint keeps tracking
    private boolean waitMs(long ms) {
        follower.stop();
        ElapsedTime wait = new ElapsedTime();
        while (opModeIsActive() && wait.milliseconds() < ms) {
            if (outOfTime()) return false;
            follower.update();
            drive.readBattery();   // wheels are off so drive() isn't running, keep the number fresh anyway
            updateTelemetry();
        }
        return opModeIsActive();
    }

    private boolean outOfTime() {
        double t = timer.seconds();
        boolean late = t > stopAt || (!parking && autoLength - t < timeToPark());
        if (late) {
            follower.stop();
            follower.update();
        }
        return late;
    }

    private double timeToPark() {
        return parkTimeFrom(mirrorIfBlue(follower.pose()));
    }

    private double parkTimeFrom(Pose here) {
        double speed = avgSpeed * Math.min(1, drive.voltage / nominalVoltage);   // a tired battery can't hit full speed even with the boost
        return Math.hypot(here.x() - parkPose.x(), here.y() - parkPose.y()) / speed + parkBuffer;
    }

    private boolean step(String name, boolean done) {
        if (done) logStep(name);
        return done;
    }

    private void logStep(String name) {
        double now = timer.seconds();
        // stuck = it hit something or ran out of time on a move in that step
        String line = String.format("%-20s %4.1f s (at %4.1f)", name, now - lastStepTime, now);
        if (timesStuck > 0) line += " stuck " + timesStuck;
        stepLog.add(line + stepNote);
        log.event("STEP " + name + stepNote);
        stepNote = "";
        lastStepTime = now;
        timesStuck = 0;
    }

    // blue is red rotated 180 around the field center, so doing this twice gets you back
    private Pose mirrorIfBlue(Pose p) {
        if (!isBlue) return p;
        double h = p.heading() + Math.PI;
        return new Pose(fieldSize - p.x(), fieldSize - p.y(), Math.atan2(Math.sin(h), Math.cos(h)));
    }

    private void updateTelemetry() {
        Pose p = follower.pose();
        telemetry.addData("Status", status);
        telemetry.addData("Plan", "%s, %s", isBlue ? "Blue" : "Red", hiveTipped ? "tipped" : "not tipped");
        telemetry.addData("Pose", "(%.1f, %.1f) %.0f deg", p.x(), p.y(), normalizeDeg(Math.toDegrees(p.heading())));
        telemetry.addData("Balls", balls);
        telemetry.addData("Last stop off by", stopError);   // what pedro thinks. if it looks way more off, it's the pinpoint or the start spot
        telemetry.addData("Battery", "%.1f V, power x%.2f", drive.voltage, drive.scale());
        telemetry.addData("Flower approach", sonarResult);
        if (approaching) sonar.addTelemetry(telemetry);
        telemetry.addData("Time", "%.1f s, park needs ~%.1f s", timer.seconds(), timeToPark());
        for (String line : stepLog) telemetry.addLine(line);
        telemetry.update();

        // every loop in this opmode ends by calling this, so this is the once-per-loop spot for the recorder
        log.put("Auto/status", status);
        log.put("Auto/balls", balls);
        log.put("Auto/batteryScale", drive.scale());
        log.put("Auto/approaching", approaching);
        if (approaching) sonar.log(log);
        log.update();
    }

    private static Pose pose(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }

    // shortest turn from current to target, -pi to pi
    private static double angleTo(double target, double current) {
        return Math.atan2(Math.sin(target - current), Math.cos(target - current));
    }

    private static double normalizeDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg <= -180) deg += 360;
        return deg;
    }

    // pedro's mecanum but every drive command gets scaled by battery voltage, so a fresh battery and a tired one drive the same
    private static class BatteryDrive extends Mecanum {
        private final VoltageSensor battery;
        private final ElapsedTime sinceRead = new ElapsedTime();
        double voltage;

        BatteryDrive(HardwareMap hardwareMap) {
            super(hardwareMap, Constants.drivetrainConfig);
            battery = hardwareMap.voltageSensor.iterator().next();
            voltage = battery.getVoltage();
            if (voltage < 6) voltage = nominalVoltage;
        }

        @Override
        public void drive(DrivePowers powers, boolean manual) {
            readBattery();
            double s = scale();
            super.drive(new DrivePowers(powers.forward() * s, powers.strafe() * s, powers.turn() * s), manual);
        }

        // the reading jumps around under load, so only read every 100 ms and smooth it
        void readBattery() {
            if (sinceRead.milliseconds() < 100) return;
            sinceRead.reset();
            double reading = battery.getVoltage();
            if (reading > 6) voltage += 0.2 * (reading - voltage);   // under 6 is a bad read, skip it
        }

        // capped so a bad battery can't make it go nuts either way
        double scale() {
            return Range.clip(nominalVoltage / voltage, 0.75, 1.15);
        }
    }
}
