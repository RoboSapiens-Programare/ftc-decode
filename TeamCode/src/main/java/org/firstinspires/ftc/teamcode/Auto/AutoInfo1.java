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

@Autonomous(name = "Auto informatie 1", group = "0. Auto")
public class AutoInfo1 extends OpMode {


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

    private byte gateLoops = 0;
    private final byte targetGateLoops = 1;

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
        robot.shooter.update();
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
        public PathChain grab2;
        public PathChain openGate2;
        public PathChain shoot2;
        public PathChain gate1;
        public PathChain gate1p;
        public PathChain shoot3;
        public PathChain grab3;
        public PathChain shoot4;

        public Paths(Follower follower) {
            shootPreload = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(20.923, 123.133),
                                    new Pose(56.490, 93.538)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(144))
                    .build();

            grab2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.490, 93.538),
                                    new Pose(77.371, 54.378),
                                    new Pose(21.245, 57.699)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            openGate2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(21.245, 57.699),
                                    new Pose(47.173, 62.168),
                                    new Pose(18.752, 73.028)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(18.752, 73.028),
                                    new Pose(54.673, 57.524),
                                    new Pose(55.364, 94.902)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                    .build();

            gate1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(55.364, 94.902),
                                    new Pose(47.734, 62.636),
                                    new Pose(18.580, 66.594)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(160))
                    .build();

            gate1p = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(18.580, 66.594),
                                    new Pose(14.722, 60.617),
                                    new Pose(11.521, 58.668)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(110))
                    .build();

            shoot3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(11.521, 58.668),
                                    new Pose(48.192, 62.346),
                                    new Pose(55.483, 94.545)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(144))
                    .build();

            grab3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(55.483, 94.545),
                                    new Pose(67.185, 82.510),
                                    new Pose(26.552, 84.084)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                    .build();

            shoot4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(26.552, 84.084),
                                    new Pose(47.517, 102.175)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // shoot preload
                if (!pathingOnly)
                {
                    if (singleton)
                    {
                        robot.intake.rest();
                        robot.shooter.openGate();
                        singleton = false;
                    }
                }
                robot.shooter.update();
                robot.shooter.shooting = true;
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
                            Robot.follower.followPath(paths.grab2);
                            robot.shooter.shooting = false;
                            setPathState(101);
                        }
                    } else {
                        timer2.reset();
                    }
                } else {
                    timer.reset();
                }
                break;
            case 101:
                if (!pathingOnly)
                {
                    robot.shooter.shooting = false;
                    robot.intake.pullBallsHard();
                    robot.shooter.closeGate();
                }
                if (!Robot.follower.isBusy())
                {
                    Robot.follower.followPath(paths.openGate2);
                    Robot.follower.setMaxPower(0.8);
                    setPathState(1);
                }
                break;
            case 1:
                // go to grab 1
                if (!pathingOnly)
                {
                    robot.intake.rest();
                }
                if (!Robot.follower.isBusy())
                {
                    Robot.follower.followPath(paths.shoot2);
                    Robot.follower.setMaxPower(0.9);
                    setPathState(2);
                }
                break;
            case 2:
                if (singleton)
                {
                    robot.intake.rest();
                    robot.shooter.openGate();
                    singleton = false;
                }
                robot.shooter.update();
                robot.shooter.shooting = true;
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
                            Robot.follower.followPath(paths.gate1);
                            Robot.follower.setMaxPower(1);
                            robot.shooter.shooting = false;
                            setPathState(301);
                        }
                    } else {
                        timer2.reset();
                    }
                }
                else {
                    timer.reset();
                }
                break;
            case 301:
                if (!pathingOnly)
                {
                    robot.intake.pullBallsHard();
                    robot.shooter.shooting = false;
                    robot.shooter.closeGate();
                }
                if (!Robot.follower.isBusy())
                {
                    Robot.follower.followPath(paths.gate1p);
                    Robot.follower.setMaxPower(1);
                        setPathState(3);
                }
                break;
            case 3:
                // go to grab 2
                if (!pathingOnly)
                {
                    robot.intake.pullBallsHard();
                    robot.shooter.shooting = false;
                    robot.shooter.closeGate();
                }
                if (!Robot.follower.isBusy() && autoTimer.seconds() > 2)
                {
                    Robot.follower.followPath(paths.shoot3);
                    Robot.follower.setMaxPower(0.9);
                    if (gateLoops<targetGateLoops)
                    {
                        gateLoops++;
                        setPathState(2);
                    } else {
                        setPathState(4);
                    }

                }
                break;
            case 4:
                if (singleton)
                {
                    robot.intake.rest();
                    robot.shooter.openGate();
                    singleton = false;
                }
                robot.shooter.update();
                robot.shooter.shooting = true;
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
                            Robot.follower.followPath(paths.grab3);
//                            robot.shooter.shooting = false;
                            setPathState(5);
                        }
                    } else {
                        timer2.reset();
                    }
                }
                else {
                    timer.reset();
                }
                break;
            case 5:
                // get spike 3
                if (!pathingOnly)
                {
                    robot.intake.pullBallsHard();
                    robot.shooter.shooting = false;
                    robot.shooter.closeGate();
                }
                if (!Robot.follower.isBusy())
                {
                    Robot.follower.followPath(paths.shoot4);
                        setPathState(6);
                }
                break;
            case 6:
                if (singleton)
                {
                    robot.intake.rest();
                    robot.shooter.openGate();
                    singleton = false;
                }
                robot.shooter.update();
                robot.shooter.shooting = true;
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
                            robot.shooter.shooting = false;
                            setPathState(-1);
                        }
                    } else {
                        timer2.reset();
                    }
                }
                else {
                    timer.reset();
                }
                break;
        }
        return pathState;
    }

    public void setPathState(int pState) {
        timer.reset();
        timer2.reset();
        autoTimer.reset();
        singletonRest = true;
        singleton = true;
        pathState = pState;
        pathTimer.reset();
    }
}
