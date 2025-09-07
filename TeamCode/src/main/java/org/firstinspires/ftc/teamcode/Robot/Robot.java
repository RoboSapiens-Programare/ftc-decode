package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public LaunchPad lp;
    public Robot(HardwareMap hardwareMap) {
        lp = new LaunchPad (hardwareMap);
    }
}
