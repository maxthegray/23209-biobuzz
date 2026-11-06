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

public class Constants {
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

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-2.81496062992);
        c.yPodOffset.set(0.0);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3502332739941906);
                Controller secondaryTranslationalForward = Controller.proportional(0.12940183633541608);
                Controller primaryTranslationalLateral = Controller.proportional(0.5307440417581804);
                Controller secondaryTranslationalLateral = Controller.proportional(0.19609574168766297);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.007774116807357989));
                c.brake.set(Controller.proportionalFeedforward(0.00660799928625429));

                c.headingFeedback.set(Controller.proportional(6.760110883968154));
                c.maxBrakingPower.set(0.4);   // not from autotune. default 0.2 let it slide past spots
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.11055094467026527, 0.0049049414706489845));

                c.linearBrakeCoefficients.set(Matrix.diag(0.19836083101102445, 0.12202614310476821));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0016636021665278112, 0.0019522058020384728));

                c.maxAchievableForwardVelocity.set(85.57516484479729);
                c.maxAchievableStrafeVelocity.set(74.43293909632106);
                c.naturalForwardDeceleration.set(37.83372230212618);
                c.naturalStrafeDeceleration.set(64.83052595678117);
            }
    );

    public static Follower create(HardwareMap h){
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}