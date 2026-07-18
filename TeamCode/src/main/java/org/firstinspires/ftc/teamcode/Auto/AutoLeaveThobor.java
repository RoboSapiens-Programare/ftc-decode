package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "Auto Leave BLUE", group = "Autonomous - Thobor")
@Config
public class AutoLeaveThobor extends OpMode {
    private int pathState; // Current autonomous path state (state machine)

    // EDIT: HABAR NU AM CUM SE ACCESEAZA LA VOI
    // E FTFTFTFT IMPORTANT
    private Follower follower = Robot.follower;


    private final Pose startingPose = new Pose(29.000 - (31-28)/(2*2.54), 126.000+(41-30)/(2*2.54), Math.toRadians(90));

    private final Pose[] LEAVE_POINTS = {
            // inside close shoot zone
            new Pose(40.000, 130.000, Math.toRadians(180)),
            new Pose(59.000, 105.000, Math.toRadians(180)),
            new Pose(60.000, 130.000, Math.toRadians(180)),

            // mid field
            new Pose(20.000, 95.000, Math.toRadians(180)),
            new Pose(20.000, 70.000, Math.toRadians(180)),
            new Pose(35.000, 75.000, Math.toRadians(180)),

            // far field
            new Pose(48.000, 72.000, Math.toRadians(180)),
            new Pose(25.000, 45.000, Math.toRadians(180)),
            new Pose(53.000, 35.000, Math.toRadians(180)),
            new Pose(35.000, 15.000, Math.toRadians(180)),
            new Pose(52.000, 22.000, Math.toRadians(180)),
    };

    @Override
    public void init() {
        Robot.follower.setStartingPose(startingPose);
    }

    @Override
    public void loop() {
        follower.update();

        switch (pathState) {
            case 0:
                follower.followPath(generateEmergencyLeavePath());
                pathState = 1;
                break;
            case 1:
                // PATH FINISHED
                if (!follower.isBusy()) {
                    pathState = 2;
                }
                break;

            case 2:
                requestOpModeStop();
                pathState = -1;
                break;
        }
    }

    @Override
    public void stop() {
    }

    private PathChain generateEmergencyLeavePath() {
        // Grab the absolute latest position tracking from Pedro Pathing
        Pose currentPose = Robot.follower.getPose();

        Pose closestTarget = LEAVE_POINTS[0];
        double shortestDistance = Double.MAX_VALUE;

        // Loop through all points to find the closest one geometrically
        for (Pose target : LEAVE_POINTS) {
            // Calculate stand
            //                                                                    ard Euclidean
            // distance: sqrt((x2-x1)^2 + (y2-y1)^2)
            double distance =
                    Math.hypot(
                            target.getX() - currentPose.getX(), target.getY() - currentPose.getY());

            if (distance < shortestDistance) {
                shortestDistance = distance;
                closestTarget = target;
            }
        }

        // Smoothly interpolate from wherever the robot physically is right now
        // to the closest safe zone heading
        return Robot.follower
                .pathBuilder()
                .addPath(new BezierLine(currentPose, closestTarget))
                .setLinearHeadingInterpolation(currentPose.getHeading(), closestTarget.getHeading())
                .build();
    }
}
