package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Revolver;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

public class Robot {
    public boolean initialize;
    public Spindexer spindexer;
    public Revolver revolver;
    public Intake intake;
    public Turret turret;
    public Drive drive;

    public Robot(HardwareMap hwMap) {
        initialize = true;

        drive = new Drive(hwMap);

        spindexer = new Spindexer(hwMap);
        revolver = new Revolver(hwMap);

        turret = new Turret(hwMap);
        intake = new Intake(hwMap, revolver);

        initialize = false;
    }

    public boolean isInitialize() {
        return initialize;
    }
}
