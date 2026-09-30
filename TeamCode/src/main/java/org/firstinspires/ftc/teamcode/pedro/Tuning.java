package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

// everything autotune can run. open http://192.168.43.1:10158 on the robot wifi and go top to bottom
// autotune registers @Tuner methods in alphabetical order by Java method name, not source order
public class Tuning {

    // 1. spins each wheel, you say if it went the right way
    @Tuner(name = "1. Mecanum wheel directions")
    public static Procedure tuner01Mecanum() {
        return new MecanumTuner();
    }

    // 2. check driving works with the motor settings
    @Tuner(name = "2. Driving test (no odometry)")
    public static Procedure tuner02Driving() {
        return new Tests(hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig), null, null);
    }

    // 3. push forward, push left, spin 180, it works out the pod setup
    @Tuner(name = "3. Pinpoint pod setup")
    public static Procedure tuner03Pinpoint() {
        return new PinpointTuner();
    }

    // 4. forward should make x go up, strafing left should make y go up
    @Tuner(name = "4. Localization sanity check")
    public static Procedure tuner04Localization() {
        return new Tests(hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig), null);
    }

    // 5. the big one, speeds, braking and path following gains. needs lots of space
    @Tuner(name = "5. Foresight autotune (needs space)")
    public static Procedure tuner05Foresight() {
        return new ForesightTuner(hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig));
    }

    // 6. hold, line, curve etc with everything tuned
    @Tuner(name = "6. Full follower tests")
    public static Procedure tuner06FullTests() {
        return new Tests(hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                () -> new Foresight(Constants.foresightConfig));
    }
}
