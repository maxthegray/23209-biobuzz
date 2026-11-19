package org.firstinspires.ftc.teamcode.config;

// every name from the robot configuration on the driver hub, in one place.
// if you rename something in the configuration, change it here too, or the opmode crashes at init
// with "unable to find a hardware device with name ...".
// pedro/Constants.java has its own copy of the drive motor and pinpoint names (pedro's file, leave it alone)
public final class HardwareNames {

    // drive motors
    public static final String FRONT_LEFT_MOTOR = "frontLeftMotor";
    public static final String FRONT_RIGHT_MOTOR = "frontRightMotor";
    public static final String BACK_LEFT_MOTOR = "backLeftMotor";
    public static final String BACK_RIGHT_MOTOR = "backRightMotor";

    // other motors
    public static final String SHOOTER = "shooter";

    // sensors and cameras
    public static final String PINPOINT = "pinpoint";
    public static final String LIMELIGHT = "limelight";
    public static final String WEBCAM = "Webcam 1";

    // each ultrasonic sensor uses both channels of one analog port: one for the reading, one to measure its supply
    public static final String ULTRASONIC_LEFT_SIGNAL = "ultrasonic_left_signal";
    public static final String ULTRASONIC_LEFT_VCC = "ultrasonic_left_vcc";
    public static final String ULTRASONIC_RIGHT_SIGNAL = "ultrasonic_right_signal";
    public static final String ULTRASONIC_RIGHT_VCC = "ultrasonic_right_vcc";

    private HardwareNames() {
    }
}
