package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.usb.RobotArmingStateNotifier;
//import com.seattlesolvers.solverslib.photon.PhotonCore;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

public class Robot {
    public boolean initialize;

    public Shooter shooter;
    public Intake intake;
    public static Follower follower;

    private final List<LynxModule> allHubs;
    public static PoseHistory poseHistory;

    public enum Alliance {
        RED,
        BLUE
    };

    public static Alliance alliance = Alliance.BLUE;
    public static Pose transitionPose = new Pose(63, 9, Math.PI / 2);

    public Robot(HardwareMap hwMap) {
        initialize = true;

        // Setup in your Robot class if you have one, or in init at start of opMode
        // Don't do manual or auto bulk caching elsewhere - do it here.
//        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.experimental.setMaximumParallelCommands(8); // Can be adjusted based on user preference - but raising this number further can cause issues
//        PhotonCore.PARALLELIZE_SERVOS = false; // Set to false if using REV Servo Hub
//        PhotonCore.enable();

        allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        shooter = new Shooter(hwMap);

        intake = new Intake(hwMap);

        follower = Constants.createFollower(hwMap);

        poseHistory = follower.getPoseHistory();

        initialize = false;
    }

    public void resetCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public boolean isInitialize() {
        return initialize;
    }
}
