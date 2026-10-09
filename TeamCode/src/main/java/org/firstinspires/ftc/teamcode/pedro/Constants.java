package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// sensor bot settings for pedro pathing v3. every number here gets replaced by autotune
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
        c.xPodOffset.set(-3.220380347544753);
        c.yPodOffset.set(0.717855813935047);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });


    // step 3: ForesightTuner replaces ALL of these. they're the example robot from the pedro docs, not ours
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.4032646077894567);
                Controller secondaryTranslationalForward = Controller.proportional(0.14899549714943022);
                Controller primaryTranslationalLateral = Controller.proportional(0.6688022239397529);
                Controller secondaryTranslationalLateral = Controller.proportional(0.2471045510211852);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.008880530746244928));
                c.brake.set(Controller.proportionalFeedforward(0.007548451134308189));

                c.headingFeedback.set(Controller.proportional(7.723889152957562));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.07454281466576992, 0.008199354911390587));

                c.linearBrakeCoefficients.set(Matrix.diag(0.17765986717257456, 0.06951998514125576));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001974124447727619, 0.002754865299021996));

                c.maxAchievableForwardVelocity.set(64.38594567284912);
                c.maxAchievableStrafeVelocity.set(54.797120608684416);
                c.naturalForwardDeceleration.set(43.81864385893574);
                c.naturalStrafeDeceleration.set(75.9104997710097);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
