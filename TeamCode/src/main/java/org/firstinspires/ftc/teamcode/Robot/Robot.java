package org.firstinspires.ftc.teamcode.Robot;

import com.bylazar.configurables.PanelsConfigurables;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.FixedTurret;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.PoseHistory;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Robot {
    public boolean initialize;
    public Spindexer spindexer;
    public Intake intake;
    public FixedTurret turret;
    public static Follower follower;
    public static PoseHistory poseHistory;

    public enum Alliance {
        RED,
        BLUE
    };

    public static Alliance alliance = Alliance.RED;

    public Robot(HardwareMap hwMap) {
        initialize = true;

        spindexer = new Spindexer(hwMap);

        turret = new FixedTurret(hwMap);
        intake = new Intake(hwMap, spindexer);

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