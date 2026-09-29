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
public class Tuning {

    // 1. spins each wheel, you say if it went the right way
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    // 2. check driving works with the motor settings
    @Tuner
    public static Procedure drivingTest() {
        return new Tests(hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig), null, null);
    }

    // 3. push forward, push left, spin 180, it works out the pod setup
    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    // 4. forward should make x go up, strafing left should make y go up
    @Tuner
    public static Procedure localizationTest() {
        return new Tests(hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig), null);
    }

    // 5. the big one, speeds, braking and path following gains. needs lots of space
    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig));
    }

    // 6. hold, line, curve etc with everything tuned
    @Tuner
    public static Procedure fullTests() {
        return new Tests(hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                () -> new Foresight(Constants.foresightConfig));
    }
}
