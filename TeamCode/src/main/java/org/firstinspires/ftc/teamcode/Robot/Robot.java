package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Robot {
    public boolean initialize;

    public Shooter shooter;
    public Intake intake;
    public static Follower follower;
    public static PoseHistory poseHistory;

    public enum Alliance {
        RED,
        BLUE
    };

    public static Alliance alliance = Alliance.BLUE;
    public static Pose transitionPose = new Pose(63, 9, Math.PI / 2);

    public Robot(HardwareMap hwMap) {
        initialize = true;

        shooter = new Shooter(hwMap);

        intake = new Intake(hwMap);

        follower = Constants.createFollower(hwMap);

        poseHistory = follower.getPoseHistory();

        initialize = false;
    }

    public boolean isInitialize() {
        return initialize;
    }
}
