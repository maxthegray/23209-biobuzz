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
        c.xPodOffset.set(-3.220380347544753);
        c.yPodOffset.set(0.0);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });


    // step 3: ForesightTuner results (tuned on our sensor bot 2026-10-21). rerun it if the drivetrain changes
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3859305130404934);
                Controller secondaryTranslationalForward = Controller.proportional(0.14259101231523041);
                Controller primaryTranslationalLateral = Controller.proportional(0.649893858954874);
                Controller secondaryTranslationalLateral = Controller.proportional(0.24011841540008994);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.008317315818631975));
                c.brake.set(Controller.proportionalFeedforward(0.007069718445837179));

                // plain autotune again, the 5 deg split made it jump between strengths mid turn
                c.headingFeedback.set(Controller.proportional(7.0912011875081555));
                c.cosineScale.set(false);   // on made it stop and go whenever a turn fell behind
                c.maxBrakingPower.set(0.4);   // default 0.2 let it slide past spots
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.10744791340779461, 0.006264631361127194));

                c.linearBrakeCoefficients.set(Matrix.diag(0.18552051491361324, 0.10012107419859242));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002071194869299692, 0.0022879444427973015));

                c.maxAchievableForwardVelocity.set(68.3251044222146);
                c.maxAchievableStrafeVelocity.set(61.181111299438065);
                c.naturalForwardDeceleration.set(41.270915401949004);
                c.naturalStrafeDeceleration.set(64.36119081305444);
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
