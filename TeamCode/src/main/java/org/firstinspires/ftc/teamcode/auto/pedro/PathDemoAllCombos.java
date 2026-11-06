package org.firstinspires.ftc.teamcode.auto.pedro;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.paths.interpolator.PiecewiseInterpolator;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.tests.FlowerApproachTest;

import java.util.ArrayList;
import java.util.List;

// all 4 autos (red/blue, tipped/not tipped). no shooter or intake yet so those are just waits
// poses are red side in pedro visualizer coords, blue gets mirrored

//@Disabled
@Autonomous(name = "Path Demo (all combos)", group = "teaching")
public class PathDemoAllCombos extends LinearOpMode {

    private static final double fieldSize = 141.5;
    private static final double hiveX = 58;   // middle of the red hive

    private static final Pose startPose = pose(hiveX, 9, 90);
    private static final Pose audienceShootPose = pose(hiveX, 14, 90);   // 5 in off the wall so it doesn't scrape
    private static final Pose rearShootPose = pose(hiveX, 116, -90);

    private static final Pose gardenTurnSpot = new Pose(36, 30);   // spin around out here, nowhere near a wall
    private static final Pose gardenLineup = pose(15, 24, -90);   // a bit right of the pickup so it can't slide into the wall
    private static final Pose gardenPickup = pose(12, 12, -90);
    private static final Pose gardenBackup = pose(22, 24, -90);
    private static final Pose swoopControl = new Pose(22, 116);

    private static final Pose sideFlowerLineup = pose(24.4, 46.75, 180);
    private static final Pose sideFlowerPickup = pose(16.4, 46.75, 180);
    private static final Pose rearFlowerLineup = pose(48, 118, 90);
    private static final Pose rearFlowerPickup = pose(48, 125.1, 90);
    private static final Pose climbControl1 = new Pose(18, 14);
    private static final Pose climbControl2 = new Pose(18, 118);

    private static final Pose parkPose = pose(13, 108, -90);
    private static final Pose parkWaypoint = new Pose(24, 40);   // straight line from down here would hit the hive or the side flower

    private static final double autoLength = 30;
    private static final double stopAt = 29.5;
    private static final double avgSpeed = 30;   // in/s, rough guess for how long parking takes
    private static final double parkBuffer = 1.5;

    private static final long shotMs = 700;
    private static final long intakeMs = 1500;
    private static final long tipCheckMs = 1100;
    private static final int maxBalls = 4;

    private static final double nominalVoltage = 12.0;   // every path drives like it's on a 12 V battery
    private static final double lowBattery = 12.5;   // resting voltage under this, swap it before the run

    private static final double bumperToCenter = 9.0;   // half the robot length, measure it
    private static final boolean fixPoseAtWall = false;   // true = let the wall reading correct pedro's position after the adjust
    private static final double maxPoseFix = 4.0;   // ultrasonics disagree with pedro by more than this = don't trust them
    private static final double maxAdjust = 3.0;   // the adjust can't take it further than this from pedro's pickup spot

    private Follower follower;
    private BatteryDrive drive;
    private FlowerApproachTest.WallApproach sonar;
    private boolean approaching = false;
    private String sonarResult = "-";
    private String stepNote = "";
    private final ElapsedTime timer = new ElapsedTime();
    private boolean isBlue = false;
    private boolean hiveTipped = true;
    private boolean parking = false;
    private int balls = 4;
    private String status = "Starting";
    private final List<String> stepLog = new ArrayList<>();
    private double lastStepTime = 0;
    private double worstLag = 0;
    private int timesStuck = 0;
    private Pose lastTarget;
    private String stopError = "-";

    @Override
    public void runOpMode() {
        // same as Constants.create() but with the battery scaled drive, so it covers every path, spin and hold
        drive = new BatteryDrive(hardwareMap);
        follower = new Follower(new PinpointLocalizer(hardwareMap, Constants.localizerConfig), drive, new Foresight(Constants.foresightConfig));
        try {
            sonar = new FlowerApproachTest.WallApproach(hardwareMap);
        } catch (IllegalArgumentException e) {
            sonar = null;   // not in the robot config, flowers just use pedro
        }
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        while (opModeInInit()) {
            if (gamepad1.dpad_left) isBlue = false;
            if (gamepad1.dpad_right) isBlue = true;
            if (gamepad1.dpad_up) hiveTipped = true;
            if (gamepad1.dpad_down) hiveTipped = false;

            telemetry.addData("Alliance", isBlue ? "Blue" : "Red");
            telemetry.addData("Hive", hiveTipped ? "Tipped" : "Not tipped");
            telemetry.addLine("Dpad left/right = alliance, up/down = tipped");
            telemetry.addLine(isBlue ? "Back against the rear wall, centered on the blue hive"
                    : "Back against the audience wall, centered on the red hive");
            drive.readBattery();
            telemetry.addData("Battery", "%.1f V%s", drive.voltage, drive.voltage < lowBattery ? "  LOW, swap it" : "");
            if (sonar != null) sonar.showReadings(telemetry);
            else telemetry.addLine("Ultrasonics not in the config, flowers use pedro only");
            telemetry.update();
        }
        if (isStopRequested()) return;

        timer.reset();
        follower.setPose(mirrorIfBlue(startPose));
        lastTarget = mirrorIfBlue(startPose);

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

        if (hiveTipped) {
            if (!step("Garden lineup", lineUpForGarden())) return false;
            if (!step("Garden", intakeAndBackUp(gardenPickup, gardenBackup))) return false;
            if (!step("Swoop to rear", swoopToRearShoot())) return false;
            if (!step("Shoot garden balls", shoot())) return false;
            if (!step("Rear flower", pickUpFlower(rearFlowerLineup, rearFlowerPickup, rearFlowerLineup))) return false;
        } else {
            if (!step("Side flower", pickUpFlower(sideFlowerLineup, sideFlowerPickup, sideFlowerLineup))) return false;
            if (!step("Back to shoot", backToAudienceShot())) return false;
            if (!step("Shoot again", shoot())) return false;
            if (!step("Climb to rear flower", climbToRearFlower())) return false;
            if (!step("Rear flower", intakeFromFlower(rearFlowerPickup, rearFlowerLineup))) return false;
        }

        if (!step("Back to rear shot", backToRearShot())) return false;
        return step("Shoot flower balls", shoot());
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

    // drive over without turning, then spin to face it. turning on the way made it jerk
    private boolean pickUpFlower(Pose lineup, Pose pickup, Pose backup) {
        status = "Lining up";
        if (!slideTo(lineup)) return false;
        if (!turnTo(lineup.heading())) return false;
        return intakeFromFlower(pickup, backup);
    }

    // pedro drives in like normal, then the ultrasonics adjust. if they give up it just stays where pedro put it
    private boolean intakeFromFlower(Pose pickup, Pose backup) {
        status = "Driving in";
        if (!driveTo(pickup)) return false;
        if (!adjustAtFlower(pickup)) return false;
        status = "Intaking (simulated)";
        if (!waitMs(intakeMs)) return false;
        balls = maxBalls;
        status = "Backing up";
        return driveTo(backup);
    }

    // squares up so both sensors read the same, then creeps to the stop distance. false = out of time
    private boolean adjustAtFlower(Pose redPickup) {
        if (sonar == null) {
            sonarResult = "no sensors, pedro only";
            stepNote = " pedro";
            return true;
        }
        if (follower.holding() && !settle()) return false;   // pedro's path ends with it still rolling, stop first
        Pose pickup = mirrorIfBlue(redPickup);
        status = "Ultrasonic adjust";
        approaching = true;
        sonar.start();
        FlowerApproachTest.WallApproach.Result result = FlowerApproachTest.WallApproach.Result.RUNNING;
        boolean wandered = false;
        while (opModeIsActive() && result == FlowerApproachTest.WallApproach.Result.RUNNING && !outOfTime()) {
            result = sonar.update(follower, leftOf(pickup));
            follower.update();
            wandered = follower.pose().distance(pickup) > maxAdjust;   // bad readings would drive it off somewhere, pedro is the boss
            if (wandered) break;
            updateTelemetry();
        }
        approaching = false;
        if (result == FlowerApproachTest.WallApproach.Result.DONE) {
            stepNote = " sonar";
            fixPoseFromWall();
            return true;
        }
        follower.manual(0, 0, 0);
        sonarResult = wandered ? "wanted to go too far, back to pedro's spot" : "failed, back to pedro's spot";
        stepNote = " sonar failed";
        if (follower.pose().distance(pickup) > 1) return driveTo(redPickup);
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

    private boolean lineUpForGarden() {
        status = "To the turn spot";
        if (!slideTo(gardenTurnSpot)) return false;
        status = "Turning around";
        if (!turnTo(gardenLineup.heading())) return false;
        status = "Lining up";
        return driveTo(gardenLineup);
    }

    private boolean backToAudienceShot() {
        status = "Back to shoot";
        if (!turnTo(audienceShootPose.heading())) return false;   // turn here, there's no room for it by the wall
        return driveTo(audienceShootPose);
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

    private boolean backToRearShot() {
        status = "Back to the rear shot";
        if (!slideTo(rearShootPose)) return false;
        return turnTo(rearShootPose.heading());
    }

    private boolean intakeAndBackUp(Pose pickup, Pose backup) {
        status = "Driving in";
        if (!driveTo(pickup)) return false;
        status = "Intaking (simulated)";
        if (!waitMs(intakeMs)) return false;
        balls = maxBalls;
        status = "Backing up";
        return driveTo(backup);
    }

    private boolean swoopToRearShoot() {
        status = "Swooping to the rear";
        Pose target = mirrorIfBlue(rearShootPose);
        return follow(curve(follower.pose(), mirrorIfBlue(swoopControl), target).constant(target.heading()));
    }

    private boolean climbToRearFlower() {
        status = "Climbing to the rear flower";
        Pose target = mirrorIfBlue(rearFlowerLineup);
        Path path = curve(follower.pose(), mirrorIfBlue(climbControl1), mirrorIfBlue(climbControl2), target);
        return follow(path.constant(target.heading()));
    }

    private void park() {
        parking = true;
        status = "Parking";
        Pose here = mirrorIfBlue(follower.pose());
        if (here.y() < parkWaypoint.y()) driveTo(parkWaypoint.withHeading(here.heading()));
        driveTo(parkPose);
    }

    private boolean driveTo(Pose redTarget) {
        return driveTo(redTarget, 0, 1);
    }

    // drives there still facing the same way
    private boolean slideTo(Pose redTarget) {
        return driveTo(redTarget.withHeading(mirrorIfBlue(follower.pose()).heading()));
    }

    // spins in place. a 180 aims 90 deg ccw first so it can't flip between left and right
    private boolean turnTo(double redHeading) {
        Pose spot = follower.pose();
        double target = isBlue ? redHeading + Math.PI : redHeading;
        if (Math.abs(angleTo(target, spot.heading())) > Math.toRadians(150)
                && !spin(spot.withHeading(spot.heading() + Math.PI / 2), 30)) return false;
        return spin(spot.withHeading(target), 4);
    }

    private boolean spin(Pose target, double closeEnoughDeg) {
        lastTarget = target;
        follower.hold(target);
        ElapsedTime wait = new ElapsedTime();
        while (opModeIsActive() && wait.seconds() < 1.5) {
            if (outOfTime()) return false;
            follower.update();
            if (Math.abs(angleTo(target.heading(), follower.pose().heading())) < Math.toRadians(closeEnoughDeg)) return true;
            updateTelemetry();
        }
        return opModeIsActive();
    }

    // the turn happens between turnStart and turnEnd (fractions of the path)
    private boolean driveTo(Pose redTarget, double turnStart, double turnEnd) {
        Pose here = follower.pose();
        Pose target = mirrorIfBlue(redTarget);

        // a 180 can go either way depending on sensor noise, so those always go ccw through the halfway heading
        double turn = angleTo(target.heading(), here.heading());
        if (turn < Math.toRadians(-170)) turn += 2 * Math.PI;
        double halfway = here.heading() + turn / 2;

        PiecewiseInterpolator heading = Interpolator.piecewise();
        if (turnStart > 0) heading.until(turnStart, Interpolator.constant(here.heading()));
        heading.until((turnStart + turnEnd) / 2, Interpolator.linear(here.heading(), halfway));
        heading.until(turnEnd, Interpolator.linear(halfway, target.heading()));
        if (turnEnd < 1) heading.until(1, Interpolator.constant(target.heading()));

        return follow(line(here, target).heading(heading));
    }

    // no speed cap on purpose, capped paths only got ~0.2 power and the robot never started moving
    private boolean follow(Path path) {
        follower.follow(path);
        Pose end = path.endPose();
        lastTarget = end;
        double closest = Double.MAX_VALUE;
        ElapsedTime sinceCloser = new ElapsedTime();
        while (opModeIsActive()) {
            if (outOfTime()) return false;
            follower.update();
            if (!follower.following()) return true;   // pedro switches to holding when the path ends

            // pedro won't push harder if the robot stalls, so catch it here
            double left = distanceLeft(end);
            if (left < closest - 1) {
                closest = left;
                sinceCloser.reset();
            }
            if (sinceCloser.seconds() > 0.75) return getUnstuck(end);

            Object headingError = follower.debug().algorithm().get("headingError");
            if (headingError instanceof Double) worstLag = Math.max(worstLag, Math.abs(Math.toDegrees((Double) headingError)));
            updateTelemetry();
        }
        return false;
    }

    // hold pushes harder the further off it is, so it gets the robot moving again
    private boolean getUnstuck(Pose end) {
        timesStuck++;
        follower.hold(end);
        ElapsedTime wait = new ElapsedTime();
        while (opModeIsActive() && wait.seconds() < 1.5) {
            if (outOfTime()) return false;
            follower.update();
            if (distanceLeft(end) < 1.5) break;
            updateTelemetry();
        }
        return opModeIsActive();
    }

    // inches to go, plus 1 in for every ~6 deg of turning left
    private double distanceLeft(Pose end) {
        Pose p = follower.pose();
        return p.distance(end) + 10 * Math.abs(angleTo(end.heading(), p.heading()));
    }

    // cuts the wheels and waits. still calls update() so the pinpoint keeps tracking
    private boolean waitMs(long ms) {
        if (follower.holding() && !settle()) return false;
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

    // pedro ends paths a little early, so hold till it actually stops or it coasts past
    private boolean settle() {
        ElapsedTime wait = new ElapsedTime();
        while (opModeIsActive() && wait.seconds() < 1) {
            if (outOfTime()) return false;
            follower.update();
            Velocity v = follower.velocity();
            boolean stopped = Math.hypot(v.vx, v.vy) < 2 && Math.abs(v.omega) < 0.2;
            if (stopped && (distanceLeft(lastTarget) < 1.5 || wait.seconds() > 0.4)) break;
            updateTelemetry();
        }
        Pose p = follower.pose();
        stopError = String.format("%.1f in, %.0f deg", p.distance(lastTarget), Math.abs(Math.toDegrees(angleTo(lastTarget.heading(), p.heading()))));
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
        Pose here = mirrorIfBlue(follower.pose());
        double speed = avgSpeed * Math.min(1, drive.voltage / nominalVoltage);   // a tired battery can't hit full speed even with the boost
        return Math.hypot(here.x() - parkPose.x(), here.y() - parkPose.y()) / speed + parkBuffer;
    }

    private boolean step(String name, boolean done) {
        if (done) logStep(name);
        return done;
    }

    private void logStep(String name) {
        double now = timer.seconds();
        // lag = most the heading fell behind the path during that step, stuck = how many times it had to get unstuck
        String line = String.format("%-20s %4.1f s (at %4.1f) lag %3.0f", name, now - lastStepTime, now, worstLag);
        if (timesStuck > 0) line += " stuck " + timesStuck;
        stepLog.add(line + stepNote);
        stepNote = "";
        lastStepTime = now;
        worstLag = 0;
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
