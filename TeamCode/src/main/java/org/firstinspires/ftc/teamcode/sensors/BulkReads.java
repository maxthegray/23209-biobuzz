package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

// makes the hub read every sensor in one go, once per loop.
//
//     List<LynxModule> hubs = BulkReads.manual(hardwareMap);   // once, after hardware setup
//     while (...) {
//         BulkReads.clear(hubs);                               // FIRST thing in every loop
//         ...
//     }
//
// the first read after clear() fetches all the analog channels in one transaction, so an ultrasonic sensor's
// signal and Vcc come from the same instant (and the loop runs faster).
// forgetting clear() = readings never change
public final class BulkReads {

    public static List<LynxModule> manual(HardwareMap hardwareMap) {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        return hubs;
    }

    public static void clear(List<LynxModule> hubs) {
        for (LynxModule hub : hubs) hub.clearBulkCache();
    }

    private BulkReads() {
    }
}
