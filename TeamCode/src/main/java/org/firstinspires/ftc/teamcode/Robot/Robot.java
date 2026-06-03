package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
// import com.seattlesolvers.solverslib.photon.PhotonCore;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;
import java.util.List;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Robot {
    public boolean initialize;

    public Shooter shooter;
    public Intake intake;
    private final CachingServo headlight;
    private final ElapsedTime headlightTimer = new ElapsedTime();

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
        //        PhotonCore.experimental.setMaximumParallelCommands(8); // Can be adjusted based on
        // user preference - but raising this number further can cause issues
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

        headlight = new CachingServo(hwMap.get(Servo.class, "headlight"));

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

    public void updateHeadlight() {
        if (shooter.shooting) {
            if (shooter.velocityReached() && shooter.isAimed()) {
                headlight.setPosition(1);
            } else if (intake.isEmpty()) {
                if (headlightTimer.milliseconds() > 10) {
                    if (headlight.getPosition() > 0) {
                        headlight.setPosition(0);
                    } else {
                        headlight.setPosition(1);
                    }

                    headlightTimer.reset();
                }
            } else {
                if (headlightTimer.milliseconds() > 50) {
                    if (headlight.getPosition() > 0) {
                        headlight.setPosition(0);
                    } else {
                        headlight.setPosition(0.75);
                    }

                    headlightTimer.reset();
                }
            }
        } else {
            if (intake.isFull()) {
                if (headlightTimer.milliseconds() > 200) {
                    if (headlight.getPosition() > 0) {
                        headlight.setPosition(0);
                    } else {
                        headlight.setPosition(0.5);
                    }

                    headlightTimer.reset();
                }
            } else {
                headlight.setPosition(0.277);
            }
        }
    }

    public void update() {
        updateHeadlight();

        intake.update();
        shooter.update();
        follower.update();
    }

    public void init() {
        intake.init();
        shooter.init();
    }
}
