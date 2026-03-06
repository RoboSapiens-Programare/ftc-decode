package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.uV;

@Autonomous(name = "Auto Far 9 blue", group = "0. Auto")
public class AutoFar9Blu extends OpMode {


    private int pathState; // Current autonomous path state (state machine)
    private ElapsedTime pathTimer; // Timer for path state machine
    private boolean singleton;
    private Paths paths; // Paths defined in the Paths class

    private static Robot robot;

    private boolean pathingOnly=false;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime autoTimer = new ElapsedTime();

    Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        Robot.alliance = Robot.Alliance.BLUE;

        Robot.follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));

        pathTimer = new ElapsedTime();
        paths = new Paths(Robot.follower); // Build paths
    }

    @Override
    public void init_loop() {

        autoTimer.reset();

        dashboardTelemetry.update();
    }


    @Override
    public void loop() {

        Robot.follower.update(); // Update Pedro Pathing
        robot.intake.update();
//        robot.shooter.update();
        pathState = autonomousPathUpdate(); // Update autonomous state machine


        // Log values to Panels and Driver Station
        dashboardTelemetry.addData("Path State", pathState);
        dashboardTelemetry.addData("X", Robot.follower.getPose().getX());
        dashboardTelemetry.addData("Y", Robot.follower.getPose().getY());
        dashboardTelemetry.addData("Heading", Robot.follower.getPose().getHeading());

        telemetry.addData("velo", robot.shooter.turretMotorRight.getVelocity());

        telemetry.addData("empty", robot.intake.isEmpty());

        robot.shooter.turretMotorLeft.setPower(0.8);
        robot.shooter.turretMotorRight.setPower(0.8);

        robot.shooter.lobServo.setPosition(uV.angleFar);

        dashboardTelemetry.update();
    }

    @Override
    public void stop() {
        Robot.transitionPose = Robot.follower.getPose();
    }

    public static class Paths {
        public PathChain grab1;
        public PathChain shoot1;
        public PathChain grab2;
        public PathChain shoot2;
        public PathChain grab3;
        public PathChain shoot3;

        public Paths(Follower follower) {
            grab1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.503, 8.336),
                                    new Pose(64.448, 39.776),
                                    new Pose(26.797, 35.664)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            shoot1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(26.797, 35.664),
                                    new Pose(60.287, 12.965)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();

            grab2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(60.287, 12.965),
                                    new Pose(67.717, 62.573),
                                    new Pose(26.503, 59.930)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            shoot2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(26.503, 59.930),
                                    new Pose(60.497, 12.608)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();

            grab3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(60.497, 12.608),
                                    new Pose(78.829, 91.077),
                                    new Pose(31.678, 84.161)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            shoot3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(31.678, 84.161),
                                    new Pose(49.196, 90.448)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // shoot preload
                if (!pathingOnly)
                {
                    robot.shooter.openGate();
//                    robot.shooter.update();
//                    robot.shooter.track();
                }
                if (!Robot.follower.isBusy() /* && robot.shooter.isShootReady() */ && !pathingOnly)
                {
                    if (robot.shooter.turretMotorRight.getVelocity() > 1400)
                    {
                        robot.intake.shoot();
                    }
                    if (robot.intake.isEmpty())
                    {
                        if (timer.seconds()>1)
                        {
                            Robot.follower.followPath(paths.grab1);
                            setPathState(1);
                        }
                    } else {
                        timer.reset();
                    }
                } else if (pathingOnly && !Robot.follower.isBusy()) {
                    Robot.follower.followPath(paths.grab1);
                    setPathState(1);
                }
                break;
            case 1:
                // go to grab 1
                if (!pathingOnly)
                {
                    robot.intake.pullBalls();
                    robot.shooter.closeGate();
                }
                if (!Robot.follower.isBusy())
                {
                    // rest e redundant teoretic aici da cred ca e mai eficient + justifica pathingonly
                    robot.intake.rest();
                    Robot.follower.followPath(paths.shoot1);
                    setPathState(2);
                } else if (pathingOnly && !Robot.follower.isBusy()) {
                    Robot.follower.followPath(paths.shoot1);
                    setPathState(2);
                }
                break;
            case 2:
                // shoot 1
                if (!pathingOnly)
                {
                    robot.shooter.openGate();
//                    robot.shooter.update();
//                    robot.shooter.track();
                }
                if (!Robot.follower.isBusy() /*&& robot.shooter.isShootReady()*/ && !pathingOnly)
                {
                    if (robot.shooter.turretMotorRight.getVelocity()>1400)
                    {
                        robot.intake.shoot();
                    }
                    if (robot.intake.isEmpty())
                    {
                        if (timer.seconds()>1)
                        {
                            Robot.follower.followPath(paths.grab2);
                            setPathState(3);
                        }
                    } else {
                        timer.reset();
                    }
                } else if (pathingOnly && !Robot.follower.isBusy()) {
                    Robot.follower.followPath(paths.grab2);
                    setPathState(3);
                }
                break;
            case 3:
                // go to grab 2
                if (!pathingOnly)
                {
                    robot.intake.pullBalls();
                    robot.shooter.closeGate();
                }
                if (!Robot.follower.isBusy())
                {
                    robot.intake.rest();
                    Robot.follower.followPath(paths.shoot2);
                    setPathState(4);
                } else if (pathingOnly && !Robot.follower.isBusy()) {
                    Robot.follower.followPath(paths.shoot2);
                    setPathState(4);
                }
                break;
            case 4:
                // shoot 2
                if (!pathingOnly)
                {
                    robot.shooter.openGate();
//                    robot.shooter.update();
//                    robot.shooter.track();
                }
                if (!Robot.follower.isBusy() /*&& robot.shooter.isShootReady()*/ && !pathingOnly)
                {
                    if (robot.shooter.turretMotorRight.getVelocity()>1400)
                    {
                        robot.intake.shoot();
                    }
                    if (robot.intake.isEmpty())
                    {
                        if (timer.seconds()>1)
                        {
                            Robot.follower.followPath(paths.grab3);
                            setPathState(5);
                        }
                    } else {
                        timer.reset();
                    }
                } else if (pathingOnly && !Robot.follower.isBusy()) {
                    Robot.follower.followPath(paths.grab3);
                    setPathState(5);
                }
                break;
            case 5:
                // go to grab 3
                if (!pathingOnly)
                {
                    robot.intake.pullBalls();
                    robot.shooter.closeGate();
                }
                if (!Robot.follower.isBusy())
                {
                    robot.intake.rest();
                    Robot.follower.followPath(paths.shoot3);
                    setPathState(6);
                } else if (pathingOnly && !Robot.follower.isBusy()) {
                    Robot.follower.followPath(paths.shoot3);
                    setPathState(6);
                }
                break;
            case 6:
                // shoot 3
                if (!pathingOnly)
                {
                    robot.shooter.openGate();
//                    robot.shooter.update();
//                    robot.shooter.track();
                }
                if (!Robot.follower.isBusy() /* && robot.shooter.isShootReady() */ && !pathingOnly)
                {
                    if (robot.shooter.turretMotorRight.getVelocity()>1400)
                    {
                        robot.intake.shoot();
                    }
                    if (robot.intake.isEmpty())
                    {
                        setPathState(-1);
                    }
                } else if (pathingOnly && !Robot.follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
        return pathState;
    }

    public void setPathState(int pState) {
        timer.reset();
        singleton = true;
        pathState = pState;
        pathTimer.reset();
    }
}
