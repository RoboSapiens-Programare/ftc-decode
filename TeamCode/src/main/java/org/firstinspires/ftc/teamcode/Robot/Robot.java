package org.firstinspires.ftc.teamcode.Robot;

import com.bylazar.configurables.PanelsConfigurables;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Revolver;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Robot {
    public boolean initialize;
    public Revolver revolver;
    public Intake intake;
    public Turret turret;
    public Drive drive;
    public static Follower follower;
    public static PoseHistory poseHistory;

    public Robot(HardwareMap hwMap) {
        initialize = true;

        drive = new Drive(hwMap);
        revolver = new Revolver(hwMap);

        turret = new Turret(hwMap);
        intake = new Intake(hwMap, revolver);

        if (follower == null) {
            follower = Constants.createFollower(hwMap);
            PanelsConfigurables.INSTANCE.refreshClass(this);
        } else {
            follower = Constants.createFollower(hwMap);
        }

        poseHistory = follower.getPoseHistory();

        initialize = false;
    }

    public boolean isInitialize() {
        return initialize;
    }
}
