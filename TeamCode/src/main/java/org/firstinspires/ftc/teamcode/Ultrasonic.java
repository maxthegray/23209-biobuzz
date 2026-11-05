package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name = "MaxBotix MB1614 Test", group = "Sensor")
public class Ultrasonic extends LinearOpMode {

    // Declare the analog input sensor
    private AnalogInput ultrasonicSensor;

    // The REV Hub outputs a 3.3V supply to the sensor pins
    private final double SUPPLY_VOLTAGE = 3.3;

    @Override
    public void runOpMode() {
        // Initialize hardware using the exact name from your robot configuration
        ultrasonicSensor = hardwareMap.get(AnalogInput.class, "maxbotix_ultrasonic");

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read raw voltage (returns a value between 0.0 and 3.3 volts)
            double rawVoltage = ultrasonicSensor.getVoltage();

            // Formula: Distance (mm) = (Voltage / Supply Voltage) * 5120
            double distanceMm = (rawVoltage / SUPPLY_VOLTAGE) * 5120.0;
            double distanceCm = distanceMm / 10.0;
            double distanceInches = distanceCm / 2.54;

            // Output data to Driver Station Telemetry
            telemetry.addData("Raw Voltage", "%.3f V", rawVoltage);
            telemetry.addData("Distance (mm)", "%.1f mm", distanceMm);
            telemetry.addData("Distance (cm)", "%.1f cm", distanceCm);
            telemetry.addData("Distance (in)", "%.1f in", distanceInches);
            telemetry.update();

            sleep(50);
        }
    }
}