package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.paths.interpolator.PiecewiseInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.List;

// all 4 autos (red/blue, tipped/not tipped). no shooter or intake yet so those are just waits
// poses are red side in pedro visualizer coords, blue gets mirrored
@Autonomous(name = "Path Demo (all combos)", group = "teaching")
public class PathDemoAllCombos extends LinearOpMode {

    private static final double fieldSize = 141.5;
    private static final double hiveX = 58;   // middle of the red hive

    private static final Pose startPose = pose(hiveX, 9, 90);
    private static final Pose audienceShootPose = pose(hiveX, 14, 90);   // 5 in off the wall so the turn doesn't scrape
    private static final Pose rearShootPose = pose(hiveX, 116, -90);

    private static final Pose gardenTurnSpot = pose(36, 30, -90);   // do the 180 out here, nowhere near a wall
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

    private Follower follower;
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

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
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
            telemetry.update();
        }
        if (isStopRequested()) return;

        timer.reset();
        follower.setPose(mirrorIfBlue(startPose));

        if (!runPlan()) logStep("Out of time");
        park();
        logStep("Park");

        status = "Parked";
        while (opModeIsActive() && timer.seconds() < stopAt) {
            follower.update();
            updateTelemetry();
        }
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
            if (!step("Rear flower", pickUp(rearFlowerLineup, rearFlowerPickup, rearFlowerLineup))) return false;
        } else {
            if (!step("Side flower", pickUp(sideFlowerLineup, sideFlowerPickup, sideFlowerLineup))) return false;
            if (!step("Back to shoot", driveTo(audienceShootPose, 0, 0.6))) return false;
            if (!step("Shoot again", shoot())) return false;
            if (!step("Climb to rear flower", climbToRearFlower())) return false;
            if (!step("Rear flower", intakeAndBackUp(rearFlowerPickup, rearFlowerLineup))) return false;
        }

        if (!step("Back to rear shot", driveTo(rearShootPose, 0, 0.6))) return false;
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

    private boolean pickUp(Pose lineup, Pose pickup, Pose backup) {
        status = "Lining up";
        if (!driveTo(lineup, 0.3, 0.9)) return false;   // wait for the corners to clear the wall, done turning before we get there
        return intakeAndBackUp(pickup, backup);
    }

    private boolean lineUpForGarden() {
        status = "Turning for the garden";
        if (!driveTo(gardenTurnSpot, 0.35, 0.9)) return false;   // go straight till the corners clear the start wall
        status = "Lining up";
        return driveTo(gardenLineup, 0, 0.5);   // close to the wall here so any leftover turn happens early
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

    // the turn happens between turnStart and turnEnd (fractions of the path)
    private boolean driveTo(Pose redTarget, double turnStart, double turnEnd) {
        Pose here = follower.pose();
        Pose target = mirrorIfBlue(redTarget);

        // a 180 can go either way depending on sensor noise, so those always go ccw through the halfway heading
        double turn = Math.atan2(Math.sin(target.heading() - here.heading()), Math.cos(target.heading() - here.heading()));
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
        double turnLeft = Math.atan2(Math.sin(end.heading() - p.heading()), Math.cos(end.heading() - p.heading()));
        return p.distance(end) + 10 * Math.abs(turnLeft);
    }

    // keeps updating pedro so it holds position, don't use sleep() here
    private boolean waitMs(long ms) {
        ElapsedTime wait = new ElapsedTime();
        while (opModeIsActive() && wait.milliseconds() < ms) {
            if (outOfTime()) return false;
            follower.update();
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
        Pose here = mirrorIfBlue(follower.pose());
        return Math.hypot(here.x() - parkPose.x(), here.y() - parkPose.y()) / avgSpeed + parkBuffer;
    }

    private boolean step(String name, boolean done) {
        if (done) logStep(name);
        return done;
    }

    private void logStep(String name) {
        double now = timer.seconds();
        // lag = most the heading fell behind the path during that step, stuck = how many times it had to get unstuck
        String line = String.format("%-20s %4.1f s (at %4.1f) lag %3.0f", name, now - lastStepTime, now, worstLag);
        stepLog.add(timesStuck > 0 ? line + " stuck " + timesStuck : line);
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
        telemetry.addData("Time", "%.1f s, park needs ~%.1f s", timer.seconds(), timeToPark());
        for (String line : stepLog) telemetry.addLine(line);
        telemetry.update();
    }

    private static Pose pose(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }

    private static double normalizeDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg <= -180) deg += 360;
        return deg;
    }
}
