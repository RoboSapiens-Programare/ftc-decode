package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Revolver;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

public class Robot {
    public boolean initialize;
    public Revolver revolver;
    public Turret turret;

    public Robot(HardwareMap hwMap) {
        initialize = true;
        revolver = new Revolver(hwMap);
        turret = new Turret(hwMap);
        initialize = false;
    }

    public boolean isInitialize() {
        return initialize;
    }
}
