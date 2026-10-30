package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Map;

// pedro's mecanum but powers get scaled by battery voltage, so a fresh battery and a tired one drive the same
public class VoltageMecanum extends Mecanum {

    public static double nominalVoltage = 12.0;

    private final VoltageSensor battery;
    private final ElapsedTime sinceRead = new ElapsedTime();
    private double voltage;

    public VoltageMecanum(HardwareMap hardwareMap, MecanumConfig config) {
        super(hardwareMap, config);
        battery = hardwareMap.voltageSensor.iterator().next();
        voltage = battery.getVoltage();
        if (voltage < 6) voltage = nominalVoltage;
    }

    @Override
    public void drive(DrivePowers powers, boolean manual) {
        // the reading jumps around under load, so only read every 100 ms and smooth it
        if (sinceRead.milliseconds() > 100) {
            sinceRead.reset();
            double reading = battery.getVoltage();
            if (reading > 6) voltage += 0.2 * (reading - voltage);
        }
        double scale = Range.clip(nominalVoltage / voltage, 0.75, 1.15);
        super.drive(new DrivePowers(powers.forward() * scale, powers.strafe() * scale, powers.turn() * scale), manual);
    }

    public double voltage() {
        return voltage;
    }

    @Override
    public Map<String, Object> debug() {
        Map<String, Object> map = super.debug();
        map.put("voltage", voltage);
        return map;
    }
}
