package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.HardwareNames;

// the 4 drive motors, set up the same way everywhere: left side reversed so +power = forward, brake when 0.
// only the setup lives here. the mecanum math (drive + strafe + turn) stays in each opmode so you can read it there.
// pedro/Constants.java sets the same directions for Pedro's own follower
public class MecanumMotors {

    public final DcMotorEx frontLeft, frontRight, backLeft, backRight;

    public MecanumMotors(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, HardwareNames.FRONT_LEFT_MOTOR);
        frontRight = hardwareMap.get(DcMotorEx.class, HardwareNames.FRONT_RIGHT_MOTOR);
        backLeft = hardwareMap.get(DcMotorEx.class, HardwareNames.BACK_LEFT_MOTOR);
        backRight = hardwareMap.get(DcMotorEx.class, HardwareNames.BACK_RIGHT_MOTOR);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotorEx motor : all()) motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public DcMotorEx[] all() {
        return new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight};
    }

    public void stop() {
        for (DcMotorEx motor : all()) motor.setPower(0);
    }
}
