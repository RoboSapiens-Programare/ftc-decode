package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public boolean initialize;
    public Revolver revolver;

    public Robot(HardwareMap hwMap) {
//        lp = new LaunchPad (hardwareMap);
        initialize = true;
        revolver = new Revolver(hwMap);
        initialize = false;
    }

    public boolean isInitialize() {
        return initialize;
    }
}
