package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;

// checks the two pod offsets in Constants.java. if they're wrong the robot thinks it moved every time it turns,
// so it ends up inches off after a turn even though driving straight looks fine
// it spins in place 3 times: measure, try a fix, check the fix. then it shows the two numbers to type in
@TeleOp(name = "Pod Offset Check", group = "test")
public class PodOffsetCheck extends LinearOpMode {

    static final double spinPower = 0.35;
    static final double goodEnough = 0.3;   // in. the position wobbling less than this during a spin is fine

    private Follower follower;
    private String status = "";

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
        double xOffset = Constants.localizerConfig.xPodOffset.get();
        double yOffset = Constants.localizerConfig.yPodOffset.get();

        telemetry.addLine("Put the robot in open floor, 2 ft clear all round");
        telemetry.addLine("After start: HOLD A to spin, let go to pause");
        telemetry.addData("xPodOffset now", "%.2f", xOffset);
        telemetry.addData("yPodOffset now", "%.2f", yOffset);
        telemetry.update();
        waitForStart();
        follower.setPose(new Pose(0, 0, 0));

        status = "Spin 1 of 3: measuring";
        double[] before = spin();
        if (before == null) return;
        double wobbleBefore = Math.hypot(before[0], before[1]);

        double newX = xOffset, newY = yOffset;
        double[] after = before;
        if (wobbleBefore > goodEnough) {
            // try moving each offset by what it measured, then see how much that actually changed things
            pinpoint.setOffsets(xOffset + before[1], yOffset + before[0], DistanceUnit.INCH);
            sleep(300);
            status = "Spin 2 of 3: trying a fix";
            double[] trial = spin();
            if (trial == null) return;

            newX = xOffset + before[1] / gain(before[1], trial[1]);
            newY = yOffset + before[0] / gain(before[0], trial[0]);
            pinpoint.setOffsets(newX, newY, DistanceUnit.INCH);
            sleep(300);
            status = "Spin 3 of 3: checking the fix";
            after = spin();
            if (after == null) return;
        }
        double wobbleAfter = Math.hypot(after[0], after[1]);

        while (opModeIsActive()) {
            telemetry.addData("Wobble before", "%.2f in", wobbleBefore);
            if (wobbleBefore <= goodEnough) {
                telemetry.addLine("Offsets are fine, nothing to change");
            } else {
                telemetry.addData("Wobble with the fix", "%.2f in", wobbleAfter);
                telemetry.addLine(wobbleAfter < 0.5 * wobbleBefore ? "FIX WORKED. put these in pedro/Constants.java:" : "Fix did NOT settle. run it again on flat floor, don't use these yet:");
                telemetry.addData("c.xPodOffset.set", "%.3f  (was %.3f)", newX, xOffset);
                telemetry.addData("c.yPodOffset.set", "%.3f  (was %.3f)", newY, yOffset);
            }
            telemetry.update();
        }
    }

    // how much of the wobble the trial took out. 1 = all of it, -1 = it doubled (the offset counts the other way)
    private static double gain(double before, double trial) {
        if (Math.abs(before) < 0.15) return 1;
        double gain = (before - trial) / before;
        return Math.copySign(Math.max(Math.abs(gain), 0.5), gain);
    }

    // one full turn in place. returns how far forward and left of the robot's center the pinpoint is really tracking, null = stopped
    private double[] spin() {
        double turned = 0, sumForward = 0, sumLeft = 0;
        double last = follower.pose().heading();
        while (opModeIsActive() && Math.abs(turned) < 2 * Math.PI) {
            follower.manual(0, 0, gamepad1.a ? spinPower : 0);
            follower.update();
            Pose p = follower.pose();
            double h = p.heading();
            double step = Math.atan2(Math.sin(h - last), Math.cos(h - last));
            last = h;
            // a point off-center goes round in a circle as the robot spins. averaging over a whole turn picks out where it is
            sumForward += (p.x() * Math.cos(h) + p.y() * Math.sin(h)) * step;
            sumLeft += (-p.x() * Math.sin(h) + p.y() * Math.cos(h)) * step;
            turned += step;

            telemetry.addLine(status);
            telemetry.addLine(gamepad1.a ? "Spinning" : "HOLD A to spin");
            telemetry.addData("Turned", "%.0f of 360 deg", Math.abs(Math.toDegrees(turned)));
            telemetry.update();
        }
        follower.manual(0, 0, 0);
        follower.update();
        if (!opModeIsActive()) return null;
        return new double[]{sumForward / turned, sumLeft / turned};   // dividing by the signed turn makes it work spinning either way
    }
}
