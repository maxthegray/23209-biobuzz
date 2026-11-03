package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// sensor bot settings for pedro pathing v3. all three blocks came from autotune
public class Constants {

    // step 1: MecanumTuner checks these directions, names match the robot config
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeftMotor");
        c.frontRightName.set("frontRightMotor");
        c.backLeftName.set("backLeftMotor");
        c.backRightName.set("backRightMotor");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // step 2: PinpointTuner fills in the offsets (inches) and pod directions
    // known bug (pedro issue #185): it can flip the strafe pod sign, double check it by hand
    
    // public static PinpointConfig localizerConfig = new PinpointConfig(
    //         c -> {
    //             c.name.set("pinpoint");
    //             c.xPodOffset.set(0.0);   // not measured yet
    //             c.yPodOffset.set(0.0);   // not measured yet
    //             c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    //             c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    //         }
    // );

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // Old bad value: -3.220380347544753
        c.xPodOffset.set(-2.81496062992);
        c.yPodOffset.set(0.0);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });


    // step 3: ForesightTuner results (tuned on our sensor bot 2026-10-21). rerun it if the drivetrain changes
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3597373564612878);
                Controller secondaryTranslationalForward = Controller.proportional(0.13291334085324785);
                Controller primaryTranslationalLateral = Controller.proportional(0.5502504565055769);
                Controller secondaryTranslationalLateral = Controller.proportional(0.2033028407158245);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.007815890238465663));
                c.brake.set(Controller.proportionalFeedforward(0.006643506702695813));

                c.headingFeedback.set(Controller.proportional(6.850489029145952));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.104897891962599, 0.0053908964799081385));

                c.linearBrakeCoefficients.set(Matrix.diag(0.21298353512080692, 0.11080383152120533));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0018440578423433744, 0.0020768012656047856));

                c.maxAchievableForwardVelocity.set(78.75846835654511);
                c.maxAchievableStrafeVelocity.set(71.43099689300077);
                c.naturalForwardDeceleration.set(40.49524293548839);
                c.naturalStrafeDeceleration.set(63.69204320827517);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new VoltageMecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
