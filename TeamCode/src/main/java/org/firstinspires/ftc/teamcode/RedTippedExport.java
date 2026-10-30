package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.interpolator.Interpolator;
import org.firstinspires.ftc.teamcode.pedro.Constants;

// straight from the visualizer export of red_tipped.pp, paths only (no waits for shooting/intake). full speed
@Autonomous(name = "Red Tipped (visualizer export)", group = "teaching")
public class RedTippedExport extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose redtippedstart = poseFactory.of(58, 9, 90);
    private final Pose point11turnforgarden = poseFactory.of(36, 30, -90);
    private final Pose point11turnforgardenSegment1Heading = poseFactory.of(36, 30, 90);
    private final Pose point11turnforgardenSegment2Start = poseFactory.of(36, 30, 90);
    private final Pose point11turnforgardenSegment2End = poseFactory.of(36, 30, 180);
    private final Pose point11turnforgardenSegment3Start = poseFactory.of(36, 30, 180);
    private final Pose point11turnforgardenSegment3End = poseFactory.of(36, 30, 270);
    private final Pose point11turnforgardenSegment4Heading = poseFactory.of(36, 30, -90);
    private final Pose point22togarden = poseFactory.of(15, 24, -90);
    private final Pose point33grabgarden = poseFactory.of(12, 12, -90);
    private final Pose point44backoffgarden = poseFactory.of(22, 24, -90);
    private final Pose point55swooproundthehive = poseFactory.of(58, 116, -90);
    private final Pose point55swooproundthehiveControl1 = poseFactory.of(22, 116, 0);
    private final Pose point66torearflower = poseFactory.of(48, 118, 90);
    private final Pose point66torearflowerSegment1Heading = poseFactory.of(48, 118, -90);
    private final Pose point66torearflowerSegment2Start = poseFactory.of(48, 118, -90);
    private final Pose point66torearflowerSegment2End = poseFactory.of(48, 118, 0);
    private final Pose point66torearflowerSegment3Start = poseFactory.of(48, 118, 0);
    private final Pose point66torearflowerSegment3End = poseFactory.of(48, 118, 90);
    private final Pose point66torearflowerSegment4Heading = poseFactory.of(48, 118, 90);
    private final Pose point77grabrearflower = poseFactory.of(48, 125.1, 90);
    private final Pose point88backoutofflower = poseFactory.of(48, 118, 90);
    private final Pose point99backtolaunchspot = poseFactory.of(58, 116, -90);
    private final Pose point99backtolaunchspotSegment1Start = poseFactory.of(58, 116, 90);
    private final Pose point99backtolaunchspotSegment1End = poseFactory.of(58, 116, 180);
    private final Pose point99backtolaunchspotSegment2Start = poseFactory.of(58, 116, 180);
    private final Pose point99backtolaunchspotSegment2End = poseFactory.of(58, 116, 270);
    private final Pose point99backtolaunchspotSegment3Heading = poseFactory.of(58, 116, -90);
    private final Pose point1010park = poseFactory.of(13, 108, -90);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
            follow(follower, path11turnforgarden()),
            follow(follower, path22togarden()),
            follow(follower, path33grabgarden()),
            follow(follower, path44backoffgarden()),
            follow(follower, path55swooproundthehive()),
            follow(follower, path66torearflower()),
            follow(follower, path77grabrearflower()),
            follow(follower, path88backoutofflower()),
            follow(follower, path99backtolaunchspot()),
            follow(follower, path1010park())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(redtippedstart);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path11turnforgarden() {
        return Paths.line(redtippedstart, point11turnforgarden).heading(Interpolator.piecewise().until(0.35, Interpolator.constant(point11turnforgardenSegment1Heading)).until(0.625, Interpolator.linear(point11turnforgardenSegment2Start, point11turnforgardenSegment2End)).until(0.9, Interpolator.linear(point11turnforgardenSegment3Start, point11turnforgardenSegment3End)).until(1, Interpolator.constant(point11turnforgardenSegment4Heading)));
    }

    public Path path22togarden() {
        return Paths.line(point11turnforgarden, point22togarden).constant(point22togarden);
    }

    public Path path33grabgarden() {
        return Paths.line(point22togarden, point33grabgarden).constant(point33grabgarden);
    }

    public Path path44backoffgarden() {
        return Paths.line(point33grabgarden, point44backoffgarden).constant(point44backoffgarden);
    }

    public Path path55swooproundthehive() {
        return Paths.curve(point44backoffgarden, point55swooproundthehiveControl1, point55swooproundthehive).constant(point55swooproundthehive);
    }

    public Path path66torearflower() {
        return Paths.line(point55swooproundthehive, point66torearflower).heading(Interpolator.piecewise().until(0.3, Interpolator.constant(point66torearflowerSegment1Heading)).until(0.6, Interpolator.linear(point66torearflowerSegment2Start, point66torearflowerSegment2End)).until(0.9, Interpolator.linear(point66torearflowerSegment3Start, point66torearflowerSegment3End)).until(1, Interpolator.constant(point66torearflowerSegment4Heading)));
    }

    public Path path77grabrearflower() {
        return Paths.line(point66torearflower, point77grabrearflower).constant(point77grabrearflower);
    }

    public Path path88backoutofflower() {
        return Paths.line(point77grabrearflower, point88backoutofflower).constant(point88backoutofflower);
    }

    public Path path99backtolaunchspot() {
        return Paths.line(point88backoutofflower, point99backtolaunchspot).heading(Interpolator.piecewise().until(0.3, Interpolator.linear(point99backtolaunchspotSegment1Start, point99backtolaunchspotSegment1End)).until(0.6, Interpolator.linear(point99backtolaunchspotSegment2Start, point99backtolaunchspotSegment2End)).until(1, Interpolator.constant(point99backtolaunchspotSegment3Heading)));
    }

    public Path path1010park() {
        return Paths.line(point99backtolaunchspot, point1010park).constant(point1010park);
    }
}
