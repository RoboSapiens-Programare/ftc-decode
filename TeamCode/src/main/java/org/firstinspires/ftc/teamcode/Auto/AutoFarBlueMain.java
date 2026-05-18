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

@Autonomous(name = "Auto F 'Hades' B", group = "0. Auto")
public class AutoFarBlueMain extends OpMode {


    private int pathState; // Current autonomous path state (state machine)
    int loopCount=0;
    private ElapsedTime pathTimer; // Timer for path state machine
    private boolean singleton = true;
    private Paths paths; // Paths defined in `the Paths class

    private static Robot robot;

    private boolean pathingOnly=false;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime timer2 = new ElapsedTime();
    private final ElapsedTime autoTimer = new ElapsedTime();

    int gateLoops=0;
    int desiredGateLoops = 2;

    private boolean singletonRest = true;

    Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        Robot.alliance = Robot.Alliance.BLUE;
//        robot.shooter.LL_TURRET_OFFSET_DEG = -4;

        pathTimer = new ElapsedTime();
        paths = new Paths(Robot.follower); //Build Paths
        Robot.follower.setStartingPose(new Pose(56, 6, Math.toRadians(180)));
        Robot.transitionPose = new Pose(56, 6, Math.toRadians(180));
        Robot.follower.setMaxPower(1);

        // TODO: tune
        robot.shooter.goToAngle(-Math.toRadians(95));
    }

    @Override
    public void init_loop() {

        autoTimer.reset();

        dashboardTelemetry.update();
        robot.shooter.update();
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
            dashboardTelemetry.addData("0. POSE", Robot.follower.getPose());
            dashboardTelemetry.update();
        }



        dashboardTelemetry.update();
    }

    @Override
    public void stop() {
        Robot.transitionPose = Robot.follower.getPose();
        Robot.alliance = Robot.Alliance.BLUE;

//        robot.shooter.goToAngle(0);
        robot.shooter.stopOverride();
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
                            new BezierCurve(
                                    new Pose(56.000, 6.000),
                                    new Pose(35.014, 17.944),
                                    new Pose(11.524, 10.657)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shootHuman = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(11.524, 10.657),
                                    new Pose(34.699, 17.843),
                                    new Pose(56.000, 6.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            grab1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.000, 6.000),
                                    new Pose(72.234, 47.587),
                                    new Pose(11.490, 42.749)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(11.490, 42.749),
                                    new Pose(56.000, 6.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            grabHuman2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.000, 6.000),
                                    new Pose(35.091, 17.745),
                                    new Pose(11.224, 10.622)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shootHuman2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(11.224, 10.622),
                                    new Pose(34.920, 18.087),
                                    new Pose(56.000, 6.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            grabHuman3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.000, 6.000),
                                    new Pose(34.965, 18.161),
                                    new Pose(11.133, 10.601)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shootHuman3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(11.133, 10.601),
                                    new Pose(34.934, 18.455),
                                    new Pose(56.000, 6.000)
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
//                    robot.shooter.turretLocked = true;
                    singleton = false;
                }
                if (timer2.seconds()>1 && robot.shooter.velocityReached())
                {
                    robot.intake.shoot();
                }
                else {
                    robot.intake.rest();
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
                if (!Robot.follower.isBusy() && robot.shooter.velocityReached() && !pathingOnly)
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
                    if (!Robot.follower.isBusy() && timer.seconds()>1  && robot.shooter.velocityReached())
                    {
                        robot.intake.shoot();
                    } else {
                        robot.intake.rest();
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
                if (!Robot.follower.isBusy() && robot.shooter.velocityReached() && !pathingOnly)
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
                    if (!Robot.follower.isBusy() && timer.seconds()>1 && robot.shooter.velocityReached())
                    {
                        robot.intake.shoot();
                    } else {
                        robot.intake.rest();
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
                if (!Robot.follower.isBusy() && robot.shooter.velocityReached() && !pathingOnly)
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
                    if (!Robot.follower.isBusy() && timer.seconds()>1 && robot.shooter.velocityReached())
                    {
                        robot.intake.shoot();
                    } else {
                        robot.intake.rest();
                    }
                    if (robot.intake.isEmpty())
                    {
                        if (timer2.seconds()>1)
                        {
                            if(gateLoops<desiredGateLoops)
                            {
                                Robot.follower.followPath(paths.grabHuman2);
                                robot.shooter.shooting = false;
                                setPathState(5);
                                gateLoops++;
                            } else {
                                Robot.follower.followPath(paths.grabHuman2);
                                robot.shooter.shooting = false;
                                setPathState(-1);
                            }

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
                if (!Robot.follower.isBusy() && robot.shooter.velocityReached() && !pathingOnly)
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
                    if (!Robot.follower.isBusy() && timer.seconds()>1 && robot.shooter.velocityReached())
                    {
                        robot.intake.shoot();
                    } else {
                        robot.intake.rest();
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
