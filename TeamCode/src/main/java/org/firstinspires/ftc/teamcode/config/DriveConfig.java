package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.tuning.Tunables;

// numbers for the Starter Drive teleops. the drive motor setup itself is in mechanisms/MecanumMotors.java
@Configurable
public final class DriveConfig {

    // the public static ones can be changed live in Panels (http://192.168.43.1:8001), see tuning/Tunables.java
    public static double teleopSpeedScale = 0.5;   // stick power is multiplied by this. 1 = full speed

    // remembers the numbers above as "what the code says". has to stay below them
    static {
        Tunables.track(DriveConfig.class);
    }

    // safety limits, not tunable. the code clips teleopSpeedScale to this whatever Panels says
    public static final double maxSpeedScale = 1.0;

    // "Starter Drive (30 in limit)": how far the robot may travel before the driver is locked out. a safety limit, not tunable
    public static final double distanceLimitIn = 30.0;

    private DriveConfig() {
    }

    // teleopSpeedScale, kept between 0 and maxSpeedScale
    public static double speedScale() {
        return Math.max(0, Math.min(maxSpeedScale, teleopSpeedScale));
    }
}
