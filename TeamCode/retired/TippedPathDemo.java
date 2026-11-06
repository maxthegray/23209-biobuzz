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
import org.firstinspires.ftc.teamcode.DriveStraightUltrasonic.Snuggler;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.List;

/*
 * RED auto, pretending the hive tips on the first volley. sensor bot = driving only, shooting and slurping are timed waits.
 *
 * every spot is in the pedro visualizer's numbers (visualizer.pedropathing.com), so you can type them straight in:
 *   (0,0) = corner where the red wall meets the audience wall, x -> blue wall, y -> rear wall
 *   heading 90 = facing the rear wall, -90 = facing the audience wall
 *
 * the run:
 *   1. start under our raised red cell, shoot 4 preloads at it
 *   2. tip check (pretend it tipped -> the rear red cell is up now)
 *   3. garden: line up over the 4 balls in our corner, drive into them, back out
 *   4. swoop up the red wall side of the hive to the rear side
 *   5. shoot the garden balls at the rear cell
 *   6. rear flower: line up, nose in, back out
 *   7. back to the rear launch spot, 8. shoot the flower balls
 *   9. park partly in our loading zone
 *   if the time left only just covers the drive to park, it drops everything and parks
 *
 * the DS shows how long each step took, so you can see where the 30 s went.
 */
@Autonomous(name = "Tipped Path Demo (red)", group = "teaching")
public class TippedPathDemo extends LinearOpMode {

    // ---------- spots ----------

    private static final double fieldSize = 141.5;   // real wall to wall, same as the visualizer
    private static final double halfBot = 9.0;   // 18 in robot
    private static final double hiveLaneX = fieldSize / 2 - 12.75;   // middle of the red hive = 58

    // back bumper on the audience wall, right under our raised (audience) cell, facing it
    private static final Pose startSpot = spot(hiveLaneX, halfBot, 90);

    // 4 pollen in a row against the audience wall in our corner (x ~1-11)
    private static final Pose gardenLineup = spot(11, 24, -90);   // right above the balls, facing them
    private static final Pose gardenGrab = spot(11, 3 + halfBot, -90);   // bumper 3 in off the wall = on the balls
    private static final Pose gardenBackOut = spot(22, 24, -90);   // back up and right, away from the red wall flower

    // after the tip our raised cell is the rear one (y ~85), shoot it from above
    private static final Pose launchSpotTwo = spot(hiveLaneX, 116, -90);
    private static final Pose swoopBend = new Pose(22, 116);   // curve pull point, keeps us between the red wall flower and the hive frame (x 46.5)

    // rear wall flower at x=48. grab = where the ultrasonics would stop us, plus 1 in since odometry isn't as exact
    private static final Pose rearFlowerLineup = spot(48, 118, 90);
    private static final Pose rearFlowerGrab = spot(48, fieldSize - Snuggler.stopAtIn - halfBot - 1, 90);

    // loading zone = red wall, x 0-11.5, y 94.5-117.7. ~7.5 in of robot inside it, 4 in off the wall
    private static final Pose parkSpot = spot(13, 108, -90);
    private static final Pose parkDetour = new Pose(24, 40);   // bailing from the bottom of the field, a straight line would hit the hive or the flower

    // ---------- timing ----------

    private static final double autoLength = 30.0;
    private static final double hardStopAt = 29.5;
    private static final double parkInPerSec = 30;   // rough guess at average speed incl. speeding up / slowing down
    private static final double parkCushionS = 1.5;   // spare time on top of that

    // ---------- driving ----------

    private static final double zoomSpeed = 0.8;   // fraction of top speed
    private static final double creepSpeed = 0.35;   // driving into balls / up to a flower

    // ---------- pretend mechanisms ----------

    private static final long launchMsPerBall = 700;   // guess
    private static final long slurpMs = 1500;   // guess
    private static final long tipCheckMs = 1100;   // same as the real check: 800 settle + 300 look
    private static final int maxBallsHeld = 4;   // G407

    private Follower follower;
    private final ElapsedTime autoClock = new ElapsedTime();
    private boolean parking = false;
    private int ballsOnBoard = 4;
    private String doing = "starting";
    private final List<String> laps = new ArrayList<>();
    private double lastLapAt = 0;

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        while (opModeInInit()) {
            telemetry.addLine("RED, pretending the hive tips. driving only");
            telemetry.addLine("back bumper on the audience wall");
            telemetry.addLine("robot's middle lined up with the red hive's middle, facing it");
            telemetry.update();
        }
        if (isStopRequested()) return;

        autoClock.reset();
        follower.setPose(startSpot);

        boolean onTime = pretendToLaunch() && lap("1. shoot 4 preloads");
        onTime = onTime && pretendTipCheck() && lap("2. tip check");
        onTime = onTime && visitGarden() && lap("3. garden");
        onTime = onTime && swoopToRearSide() && lap("4. swoop round the hive");
        onTime = onTime && pretendToLaunch() && lap("5. shoot garden balls");
        onTime = onTime && visitRearFlower() && lap("6. rear flower");
        onTime = onTime && backToLaunchSpotTwo() && lap("7. back to launch spot");
        onTime = onTime && pretendToLaunch() && lap("8. shoot flower balls");
        if (!onTime) lap("ran low on time, bailed");

        goPark();
        lap("9. park");

        // sit in the park spot till auto ends
        doing = "parked";
        while (opModeIsActive() && autoClock.seconds() < hardStopAt) {
            follower.update();
            blab();
        }
        follower.stop();
        follower.update();
    }

    // ---------- the steps ----------

    // no shooter yet: hold still as long as shooting would take
    private boolean pretendToLaunch() {
        // launcher + feeder go here once the shooter is mounted (see HypotheticalDemoAuton)
        while (ballsOnBoard > 0) {
            doing = "pretend launching, " + ballsOnBoard + " left";
            if (!chill(launchMsPerBall)) return false;
            ballsOnBoard--;
        }
        return true;
    }

    // the real one is in HypotheticalDemoAuton (limelight). here it always says tipped
    private boolean pretendTipCheck() {
        doing = "pretend tip check -> it TIPPED";
        return chill(tipCheckMs);
    }

    private boolean visitGarden() {
        doing = "garden: lining up";
        if (!driveTo(gardenLineup, zoomSpeed, 0.4, 1)) return false;   // turn late, we start against the wall
        doing = "garden: driving into the balls";
        // intake.setPower(1.0);
        if (!driveTo(gardenGrab, creepSpeed)) return false;
        if (!slurp()) return false;
        // intake.setPower(0);
        doing = "garden: backing out";
        return driveTo(gardenBackOut, zoomSpeed);
    }

    private boolean swoopToRearSide() {
        doing = "swooping round the hive";
        Pose here = follower.pose();
        return run(curve(here, swoopBend, launchSpotTwo).constant(launchSpotTwo.heading()), zoomSpeed);
    }

    private boolean visitRearFlower() {
        doing = "rear flower: lining up";
        if (!driveTo(rearFlowerLineup, zoomSpeed, 0.4, 1)) return false;   // turn late so our corner stays off the centre line
        doing = "rear flower: nosing in";
        // intake.setPower(1.0);
        if (!driveTo(rearFlowerGrab, creepSpeed)) return false;
        if (!slurp()) return false;
        // intake.setPower(0);
        doing = "rear flower: backing out";
        return driveTo(rearFlowerLineup, zoomSpeed);
    }

    private boolean backToLaunchSpotTwo() {
        doing = "back to the rear launch spot";
        return driveTo(launchSpotTwo, zoomSpeed, 0, 0.6);   // turn early, before we get near the centre line
    }

    // runs even after a bail. only the hard stop can cut it short
    private void goPark() {
        parking = true;
        doing = "parking";
        if (follower.pose().y() < parkDetour.y()) driveTo(parkDetour.withHeading(follower.pose().heading()), zoomSpeed);
        driveTo(parkSpot, zoomSpeed);
    }

    // pretend the intake is eating balls
    private boolean slurp() {
        doing = "pretend slurping";
        if (!chill(slurpMs)) return false;
        ballsOnBoard = maxBallsHeld;
        return true;
    }

    // ---------- driving ----------

    private boolean driveTo(Pose goal, double speedCap) {
        return driveTo(goal, speedCap, 0, 1);
    }

    // straight line to the spot. the turn happens between turnFrom and turnTo (0 = start of the path, 1 = end)
    private boolean driveTo(Pose goal, double speedCap, double turnFrom, double turnTo) {
        Pose here = follower.pose();
        PiecewiseInterpolator turnPlan = Interpolator.piecewise();
        if (turnFrom > 0) turnPlan.until(turnFrom, Interpolator.constant(here.heading()));
        turnPlan.until(turnTo, Interpolator.linear(here.heading(), goal.heading()));
        if (turnTo < 1) turnPlan.until(1, Interpolator.constant(goal.heading()));
        return run(line(here, goal).heading(turnPlan), speedCap);
    }

    // let pedro drive the path. false = out of time, go park
    private boolean run(Path path, double speedCap) {
        follower.follow(path.with(Constants.foresightConfig.maxPathSpeed.at(speedCap)));
        while (opModeIsActive()) {
            if (outOfTime()) return false;
            follower.update();
            if (!follower.following()) return true;   // pedro switches to holding the moment the path is done
            blab();
        }
        return false;
    }

    // hold still (pedro keeps holding) instead of sleep(). false = out of time
    private boolean chill(long ms) {
        ElapsedTime waiting = new ElapsedTime();
        while (opModeIsActive() && waiting.milliseconds() < ms) {
            if (outOfTime()) return false;
            follower.update();
            blab();
        }
        return opModeIsActive();
    }

    // ---------- the clock ----------

    // true = drop everything and park. while parking only the hard stop counts
    private boolean outOfTime() {
        double t = autoClock.seconds();
        boolean late = t > hardStopAt || (!parking && autoLength - t < secondsToPark());
        if (late) {
            follower.stop();
            follower.update();
        }
        return late;
    }

    private double secondsToPark() {
        Pose here = follower.pose();
        return Math.hypot(here.x() - parkSpot.x(), here.y() - parkSpot.y()) / parkInPerSec + parkCushionS;
    }

    // write down how long that step took
    private boolean lap(String step) {
        double now = autoClock.seconds();
        laps.add(String.format("%-24s %4.1f s  (at %4.1f)", step, now - lastLapAt, now));
        lastLapAt = now;
        return true;
    }

    // ---------- helpers ----------

    private void blab() {
        Pose at = follower.pose();
        telemetry.addData("Doing", doing);
        telemetry.addData("At", "(%.1f, %.1f) %.0f deg", at.x(), at.y(), wrapDegrees(Math.toDegrees(at.heading())));
        telemetry.addData("Balls", "%d (pretend)", ballsOnBoard);
        telemetry.addData("Clock", "%.1f / %.0f s, parking needs ~%.1f s", autoClock.seconds(), autoLength, secondsToPark());
        for (String lap : laps) telemetry.addLine(lap);
        telemetry.update();
    }

    private static Pose spot(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }

    private static double wrapDegrees(double deg) {
        while (deg > 180) deg -= 360;
        while (deg <= -180) deg += 360;
        return deg;
    }
}
