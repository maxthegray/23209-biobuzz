package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.groups.Groups.sequential;



@Autonomous(name = "Other Auto Test", group="testing")
public class OtherAutoTest extends OpMode {
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose sTART = poseFactory.of(56, 8, 90);
    private final Pose shootA = poseFactory.of(56, 8, 90);
    private final Pose collectA = poseFactory.of(27.1061, 36.847, -179.0553);
    private final Pose collectAControl1 = poseFactory.of(121.5833, 38.553, 0);
    private final Pose collectASegment1Start = poseFactory.of(27.1061, 36.847, 90);
    private final Pose collectASegment1End = poseFactory.of(27.1061, 36.847, 180);
    private final Pose switchABStart = poseFactory.of(27.1061, 36.847, 180);
    private final Pose switchAB = poseFactory.of(56, 135, 270);
    private final Pose switchABControl1 = poseFactory.of(19.4629, 99.0932, 0);
    private final Pose shootB = poseFactory.of(56, 135, 270);
    private final Pose collectB = poseFactory.of(26.8242, 105.6061, 178.8323);
    private final Pose collectBControl1 = poseFactory.of(119.2265, 103.5712, 0);
    private final Pose collectBSegment1Start = poseFactory.of(26.8242, 105.6061, 270);
    private final Pose collectBSegment1End = poseFactory.of(26.8242, 105.6061, 180);
    private final Pose switchBAStart = poseFactory.of(26.8242, 105.6061, 180);
    private final Pose switchBA = poseFactory.of(56.2114, 7.7477, 90);
    private final Pose switchBAControl1 = poseFactory.of(17.8348, 36.1182, 0);



    // shootA is the start pose and shootB is the end of switchAB, so those need no path of their own
    private Path collectAPath() {
        return curve(shootA, collectAControl1, collectA).linear(collectASegment1Start, collectASegment1End);
    }

    private Path switchABPath() {
        return curve(switchABStart, switchABControl1, switchAB).linear(switchABStart, switchAB);
    }

    private Path collectBPath() {
        return curve(shootB, collectBControl1, collectB).linear(collectBSegment1Start, collectBSegment1End);
    }

    private Path switchBAPath() {
        return curve(switchBAStart, switchBAControl1, switchBA).linear(switchBAStart, switchBA);
    }

    public Command autoRoutine() {
        return sequential(
                follow(follower, collectAPath()),
                follow(follower, switchABPath()),
                follow(follower, collectBPath()),
                follow(follower, switchBAPath())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(sTART);
        follower.update();
    }


    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
    }
}