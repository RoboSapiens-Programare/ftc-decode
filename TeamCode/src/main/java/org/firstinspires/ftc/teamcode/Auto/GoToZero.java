package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "Go To Zero", group = "Autonomous")
@Config
public class GoToZero extends OpMode {
    private int loopCount = 0;
    private int pathState; // Current autonomous path state (state machine)
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        robot.shooter.init();

        Robot.follower.setStartingPose(Robot.transitionPose);
    }

    @Override
    public void loop() {
        robot.resetCache();

        robot.update();

        autonomousPathUpdate();
    }

    @Override
    public void stop() {
        Robot.transitionPose = Robot.follower.getPose();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                PathChain p =
                        Robot.follower
                                .pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                Robot.follower.getPose(),
                                                new Pose(50, Robot.follower.getPose().getY())))
                                .setConstantHeadingInterpolation(Robot.follower.getHeading())
                                .build();

                Robot.follower.followPath(p, true);
                setPathState(1);

                break;
            case 1:
                if (!Robot.follower.isBusy()) {
                    setPathState(2);
                }

                break;
            case 2:
                PathChain p2 =
                        Robot.follower
                                .pathBuilder()
                                .addPath(
                                        new BezierLine(Robot.follower.getPose(), new Pose(32, 115)))
                                .setLinearHeadingInterpolation(
                                        Robot.follower.getHeading(), Math.PI / 2)
                                .build();

                Robot.follower.followPath(p2, true);
                setPathState(3);
                break;

            case 3:
                if (!Robot.follower.isBusy()) {
                    PathChain p3 =
                            Robot.follower
                                    .pathBuilder()
                                    .addPath(
                                            new BezierLine(
                                                    Robot.follower.getPose(), new Pose(32, 132)))
                                    .setConstantHeadingInterpolation(Math.PI / 2)
                                    .build();

                    Robot.follower.followPath(p3, true);
                    Robot.follower.setMaxPower(0.3);
                    setPathState(4);
                }
                break;

            case 4:
                if (!Robot.follower.isBusy()) {
                    requestOpModeStop();
                    pathState = -1;
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
    }
}
