package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.uV;

@Autonomous(name = "Auto C 12x 'spikes + preload'", group = "0. Auto")
public class AutoClose12Blu extends OpMode {


    private int pathState; // Current autonomous path state (state machine)
    int loopCount=0;
    private ElapsedTime pathTimer; // Timer for path state machine
    private boolean singleton;
    private Paths paths; // Paths defined in the Paths class

    private static Robot robot;

    private boolean pathingOnly=false;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime timer2 = new ElapsedTime();
    private final ElapsedTime autoTimer = new ElapsedTime();

    private boolean singletonRest = true;

    Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        Robot.alliance = Robot.Alliance.BLUE;

        Robot.follower.setStartingPose(new Pose(20.9, 123.1, Math.toRadians(144)));

        pathTimer = new ElapsedTime();
        paths = new Paths(Robot.follower); //Build Paths
        Robot.follower.setMaxPower(0.9);
    }

    @Override
    public void init_loop() {

        autoTimer.reset();

        dashboardTelemetry.update();
    }

    @Override
    public void start()
    {
        Robot.follower.followPath(paths.shootPreload);
        robot.shooter.openGate();
        robot.shooter.LL_TURRET_OFFSET_DEG = -1;
    }


    @Override
    public void loop() {

        Robot.follower.update(); // Update Pedro Pathing
        robot.intake.update();
//        robot.shooter.update();
        pathState = autonomousPathUpdate(); // Update autonomous state machine



        loopCount++;
        if (loopCount % 5 == 0) {
            dashboardTelemetry.addData("Path State", pathState);
            dashboardTelemetry.addData("Distance (in)", robot.shooter.llDistance);
            dashboardTelemetry.addData("Flywheel RPM", -robot.shooter.turretMotorLeft.getVelocity());
            dashboardTelemetry.update();
            dashboardTelemetry.addData("Track State", robot.shooter.trackState);
            dashboardTelemetry.addData("Turret Output", robot.shooter.turretOutput);
            dashboardTelemetry.addData("Turret Error", Math.toDegrees(robot.shooter.turretErrorRad));
            dashboardTelemetry.addData("Target RPM", robot.shooter.targetVelocity);
            dashboardTelemetry.addData("Actual RPM", -robot.shooter.turretMotorLeft.getVelocity());
            dashboardTelemetry.addData("Distance (in)", robot.shooter.llDistance);
            dashboardTelemetry.addData("Velocity OK", robot.shooter.velocityReached());
            dashboardTelemetry.addData("Aimed", robot.shooter.isAimed());
            dashboardTelemetry.addData("encoder pos", robot.shooter.turretEncoder.getCurrentPosition());
            dashboardTelemetry.addData("heading", robot.follower.getHeading());
            dashboardTelemetry.update();
        }



        dashboardTelemetry.update();
    }

    @Override
    public void stop() {
        Robot.transitionPose = Robot.follower.getPose();
        robot.shooter.LL_TURRET_OFFSET_DEG = 0;
    }

    public static class Paths {
        public PathChain shootPreload;
        public PathChain grab1;
        public PathChain shoot1;
        public PathChain grab2;
        public PathChain shoot2;
        public PathChain grab3;
        public PathChain shoot3;

        public Paths(Follower follower) {
            shootPreload = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(20.923, 123.133),
                                    new Pose(61.189, 88.368)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                    .build();

            grab1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(61.189, 88.568),
                                    new Pose(24.217, 85.508)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            shoot1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(24.217, 85.508),
                                    new Pose(62.161, 95.923)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            grab2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(62.161, 95.923),
                                    new Pose(64.112, 55.692),
                                    new Pose(24.266, 59.713)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(24.266, 59.713),
                                    new Pose(55.531, 89.867)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            grab3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(62.531, 82.867),
                                    new Pose(66.881, 31.563),
                                    new Pose(24.818, 35.580)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(24.818, 35.580),
                                    new Pose(55.559, 89.916)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // shoot preload
                if (!pathingOnly)
                {
                    robot.shooter.update();
                    robot.shooter.shooting = true;
                    if (singleton)
                    {
                        robot.intake.rest();
                        robot.shooter.openGate();
                        singleton = false;
                    }
                }
                if (!Robot.follower.isBusy() && robot.shooter.velocityReached() && !pathingOnly)
                {
                    if (!Robot.follower.isBusy() && timer.seconds()>1 && robot.shooter.velocityReached() && robot.shooter.isAimed())
                    {
                        robot.intake.shoot();
                    }
                    else {
                        robot.intake.rest();
                    }
                    if (robot.intake.isEmpty())
                    {
                        if (timer2.seconds()>0.4)
                        {
                            Robot.follower.followPath(paths.grab1);
                            robot.shooter.shooting = false;
                            setPathState(1);
                        }
                    } else {
                        timer2.reset();
                    }
                } else {
                    timer.reset();
                }
                break;
            case 1:
                // go to grab 1
                if (!pathingOnly)
                {
                    robot.intake.pullBallsHard();
                    robot.shooter.closeGate();
                }
                if (!Robot.follower.isBusy())
                {
                    Robot.follower.followPath(paths.shoot1);
                    setPathState(2);
                }
                break;
            case 2:
                if (!pathingOnly)
                {

                    robot.shooter.update();
                    robot.shooter.shooting = true;
                }
                if (!Robot.follower.isBusy() && robot.shooter.velocityReached() && !pathingOnly)
                {
                    if (singleton)
                    {
                        robot.intake.rest();
                        robot.shooter.openGate();
                        singleton = false;
                    }
                    if (!Robot.follower.isBusy() && timer.seconds()>1 && robot.shooter.velocityReached() && robot.shooter.isAimed())
                    {
                        robot.intake.shoot();
                    }
                    else {
                        robot.intake.rest();
                    }
                    if (robot.intake.isEmpty())
                    {
                        if (timer2.seconds()>0.4)
                        {
                            Robot.follower.followPath(paths.grab2);
                            robot.shooter.shooting = false;
                            setPathState(3);
                        }
                    } else {
                        timer2.reset();
                    }
                } else {
                    timer.reset();
                }
                break;
            case 3:
                // go to grab 2
                if (!pathingOnly)
                {
                    robot.intake.pullBallsHard();
                    robot.shooter.closeGate();
                }
                if (!Robot.follower.isBusy())
                {
                    Robot.follower.followPath(paths.shoot2);
                    setPathState(4);
                }
                break;
            case 4:
                if (!pathingOnly)
                {
                    robot.shooter.update();
                    robot.shooter.shooting = true;
                }
                if (!Robot.follower.isBusy() && robot.shooter.velocityReached() && !pathingOnly)
                {
                    if (singleton)
                    {
                        robot.intake.rest();
                        robot.shooter.openGate();
                        singleton = false;
                    }
                    if (!Robot.follower.isBusy() && timer.seconds()>1 && robot.shooter.velocityReached() && robot.shooter.isAimed())
                    {
                        robot.intake.shoot();
                    }
                    else {
                        robot.intake.rest();
                    }
                    if (robot.intake.isEmpty())
                    {
                        if (timer2.seconds()>0.4)
                        {
                            Robot.follower.followPath(paths.grab3);
                            robot.shooter.shooting = false;
                            setPathState(5);
                        }
                    } else {
                        timer2.reset();
                    }
                } else {
                    timer.reset();
                }
                break;
            case 5:
                // go to grab 3
                if (!pathingOnly)
                {
                    robot.intake.pullBallsHard();
                    robot.shooter.closeGate();
                }
                if (!Robot.follower.isBusy())
                {
                    Robot.follower.followPath(paths.shoot3);
                    setPathState(6);
                }
                break;
            case 6:
                if (!pathingOnly)
                {

                    robot.shooter.update();
                    robot.shooter.shooting = true;
                }
                if (!Robot.follower.isBusy() && robot.shooter.velocityReached() && !pathingOnly)
                {
                    if (singleton)
                    {
                        robot.intake.rest();
                        robot.shooter.openGate();
                        singleton = false;
                    }
                    if (!Robot.follower.isBusy() && timer.seconds()>1 && robot.shooter.velocityReached() && robot.shooter.isAimed())
                    {
                        robot.intake.shoot();
                    }
                    else {
                        robot.intake.rest();
                    }
                    if (robot.intake.isEmpty())
                    {
                        if (timer2.seconds()>1)
                        {
//                            Robot.follower.followPath(paths.grab1);
                            robot.shooter.shooting = false;
                            setPathState(-1);
                        }
                    } else {
                        timer2.reset();
                    }
                } else {
                    timer.reset();
                }
                break;
        }
        return pathState;
    }

    public void setPathState(int pState) {
        timer.reset();
        timer2.reset();
        singletonRest = true;
        singleton = true;
        pathState = pState;
        pathTimer.reset();
    }
}
