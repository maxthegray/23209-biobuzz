package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

// switches for showing sensors live while the robot runs. flip them in Panels (http://192.168.43.1:8001).
// these are on/off switches, not tuning, so they are not tracked: flipping one never asks you to COPY INTO CODE.
// they go back to what's written here after a redeploy or restart.
@Configurable
public final class LiveView {

    // true = the driver hub shows both ultrasonic distances, and Panels gets them for its Graph tab.
    // off by default so the driver hub stays clean in matches. needs the flight recorder on (it does the reading)
    public static boolean ultrasonic = false;

    private LiveView() {
    }
}
