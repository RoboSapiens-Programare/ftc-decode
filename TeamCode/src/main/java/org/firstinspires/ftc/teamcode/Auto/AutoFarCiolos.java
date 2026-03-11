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

@Autonomous(name = "Auto F 'Athena' XL", group = "0. Auto")
public class AutoFarCiolos extends OpMode {


    private int pathState; // Current autonomous path state (state machine)
    int loopCount=0;
    private ElapsedTime pathTimer; // Timer for path state machine
    private boolean singleton = true;
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
//        robot.shooter.LL_TURRET_OFFSET_DEG = -4;

        pathTimer = new ElapsedTime();
        paths = new Paths(Robot.follower); //Build Paths
        Robot.follower.setStartingPose(new Pose(56, 8, Math.toRadians(180)));
        Robot.transitionPose = new Pose(56, 8, Math.toRadians(180));
        Robot.follower.setMaxPower(1);
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
    }

    public static class Paths {
        public PathChain grabHuman;
        public PathChain shootHuman;
        public PathChain grab1;
        public PathChain shoot1;
        public PathChain grabHuman2;
        public PathChain shootHuman2;
        public PathChain grabHuman3;
        public PathChain shootHuman3;

        public Paths(Follower follower) {
            grabHuman = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000),
                                    new Pose(14.209, 7.637)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shootHuman = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(14.209, 7.637),
                                    new Pose(55.804, 8.077)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            grab1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(55.804, 8.077),
                                    new Pose(72.234, 43.056),
                                    new Pose(11.993, 39.224)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(11.993, 39.224),
                                    new Pose(56.496, 8.112)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            grabHuman2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(56.496, 8.112),
                                    new Pose(13.909, 7.937)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shootHuman2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(13.909, 7.937),
                                    new Pose(56.238, 8.126)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            grabHuman3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(56.238, 8.126),
                                    new Pose(13.986, 7.748)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shootHuman3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(13.986, 7.748),
                                    new Pose(55.839, 8.028)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (singleton)
                {
                    robot.shooter.openGate();
                    robot.shooter.shooting = true;
                    robot.shooter.turretLocked = true;
                    singleton = false;
                }
                if (timer2.seconds()>1 && robot.shooter.velocityReached())
                {
                    robot.intake.shoot();
                }
                    if (robot.intake.isEmpty())
                    {
                        if (timer.seconds()>1)
                        {
                            Robot.follower.followPath(paths.grabHuman);
                            setPathState(1);
                        }
                    } else {
                        timer.reset();
                    }

                if (Robot.follower.isBusy())
                {
                    timer2.reset();
                }
                break;
            case 1:
                // go to grab 1
                if (!pathingOnly)
                {
                    robot.intake.pullBallsHard();
                    robot.shooter.closeGate();
                    robot.shooter.shooting = false;
                }
                if (!Robot.follower.isBusy())
                {
                    if (autoTimer.seconds() > 1.5)
                    {
                        Robot.follower.followPath(paths.shootHuman);
                        setPathState(2);
                    }
                } else {
                    autoTimer.reset();
                }
                break;
            case 2:
                // shoot 1
                if (!pathingOnly)
                {
                    robot.shooter.update();
                    robot.shooter.shooting = true;
                }
                if (!Robot.follower.isBusy() /*&& robot.shooter.velocityReached()*/ && !pathingOnly)
                {
                    robot.shooter.openGate();
                    if (singleton)
                    {
                        robot.intake.rest();
                        timer.reset();
                        singleton = false;
                    }
                    if (Robot.follower.isBusy())
                    {
                        timer.reset();
                    }
                    if (!Robot.follower.isBusy() && timer.seconds()>1  /*&& robot.shooter.velocityReached()*/)
                    {
                        robot.intake.shoot();
                    }
                    if (robot.intake.isEmpty())
                    {
                        if (timer2.seconds()>1)
                        {
                            Robot.follower.followPath(paths.grab1);
//                            robot.shooter.turretLocked = false;
                            robot.shooter.shooting = false;
                            setPathState(3);
                        }
                    } else {
                        timer2.reset();
                    }
                } else if (pathingOnly && !Robot.follower.isBusy()) {
                    Robot.follower.followPath(paths.grab1);
                    setPathState(3);
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
                    robot.shooter.shooting=false;
                }
                if (!Robot.follower.isBusy())
                {
//                    robot.intake.rest();
                    Robot.follower.followPath(paths.shoot1);
                    setPathState(4);
                }
                break;
            case 4:
                // shoot 2
                if (!pathingOnly)
                {

                    robot.shooter.update();
                    robot.shooter.shooting = true;
                }
                if (!Robot.follower.isBusy() /*&& robot.shooter.velocityReached()*/ && !pathingOnly)
                {
                    robot.shooter.openGate();
                    if (singleton)
                    {
                        robot.intake.rest();
                        timer.reset();
                        singleton = false;
                    }
                    if (Robot.follower.isBusy())
                    {
                        timer.reset();
                    }
                    if (!Robot.follower.isBusy() && timer.seconds()>1 /*&& robot.shooter.velocityReached()*/)
                    {
                        robot.intake.shoot();
                    }
                    if (robot.intake.isEmpty())
                    {
                        if (timer2.seconds()>1)
                        {
                            Robot.follower.followPath(paths.grabHuman2);
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
                // go to grab 2
                if (!pathingOnly)
                {
                    robot.intake.pullBallsHard();
                    robot.shooter.closeGate();
                    robot.shooter.shooting=false;
                }

                if (!Robot.follower.isBusy())
                {
                    if (singleton)
                    {
                        autoTimer.reset();
                        singleton = false;
                    }
                    if (autoTimer.seconds() > 1.5)
                    {
                        Robot.follower.followPath(paths.shootHuman);
                        setPathState(6);
                    }
                }
                break;
            case 6:
                // shoot 2
                if (!pathingOnly)
                {
                    robot.shooter.update();
                    robot.shooter.shooting = true;
                }
                if (!Robot.follower.isBusy() /*&& robot.shooter.velocityReached()*/ && !pathingOnly)
                {
                    robot.shooter.openGate();
                    if (singleton)
                    {
                        robot.intake.rest();
                        timer.reset();
                        singleton = false;
                    }
                    if (Robot.follower.isBusy())
                    {
                        timer.reset();
                    }
                    if (!Robot.follower.isBusy() && timer.seconds()>1 /*&& robot.shooter.velocityReached()*/)
                    {
                        robot.intake.shoot();
                    }
                    if (robot.intake.isEmpty())
                    {
                        if (timer2.seconds()>1)
                        {
                            Robot.follower.followPath(paths.grabHuman2);
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
            case 7:
                // go to grab 2
                if (!pathingOnly)
                {
                    robot.intake.pullBallsHard();
                    robot.shooter.closeGate();
                    robot.shooter.shooting=false;
                }

                if (!Robot.follower.isBusy())
                {
                    if (singleton)
                    {
                        autoTimer.reset();
                        singleton = false;
                    }
                    if (autoTimer.seconds() > 1.5)
                    {
                        Robot.follower.followPath(paths.shootHuman2);
                        setPathState(8);
                    }
                }
                break;
            case 8:
                // shoot 2
                if (!pathingOnly)
                {
                    robot.shooter.update();
                    robot.shooter.shooting = true;
                }
                if (!Robot.follower.isBusy() /*&& robot.shooter.velocityReached()*/ && !pathingOnly)
                {
                    robot.shooter.openGate();
                    if (singleton)
                    {
                        robot.intake.rest();
                        timer.reset();
                        singleton = false;
                    }
                    if (Robot.follower.isBusy())
                    {
                        timer.reset();
                    }
                    if (!Robot.follower.isBusy() && timer.seconds()>1 /*&& robot.shooter.velocityReached()*/)
                    {
                        robot.intake.shoot();
                    }
                    if (robot.intake.isEmpty())
                    {
                        if (timer2.seconds()>1)
                        {
                            Robot.follower.followPath(paths.grabHuman3);
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
