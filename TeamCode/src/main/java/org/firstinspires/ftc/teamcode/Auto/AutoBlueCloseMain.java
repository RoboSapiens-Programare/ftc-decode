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

@Autonomous(name = "Auto C 'Artemis' B", group = "0. Auto")
public class AutoBlueCloseMain extends OpMode {

    private int pathState; // Current autonomous path state (state machine)
    int loopCount = 0;
    private ElapsedTime pathTimer; // Timer for path state machine
    private boolean singleton = true;
    private Paths paths; // Paths defined in the Paths class

    private static Pose grabFromGate = new Pose(20, 56.045, Math.toRadians(155));

    private static Robot robot;

    private boolean pathingOnly = false;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime timer2 = new ElapsedTime();
    private final ElapsedTime autoTimer = new ElapsedTime();

    private byte gateLoops = 0;
    int targetGateLoops = 1;

    private boolean singletonRest = true;

    Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        Robot.alliance = Robot.Alliance.BLUE;

        Robot.follower.setStartingPose(new Pose(20.9, 123.1, Math.toRadians(144)));

        pathTimer = new ElapsedTime();
        paths = new Paths(Robot.follower); // Build Paths
        Robot.follower.setMaxPower(0.9);

        robot.shooter.init();
    }

    @Override
    public void init_loop() {

        robot.intake.update();
        robot.shooter.update();

        autoTimer.reset();

        dashboardTelemetry.update();
    }

    @Override
    public void start() {
        Robot.follower.followPath(paths.shootPreload);
        robot.shooter.openGate();
        robot.shooter.LL_TURRET_OFFSET_DEG = -1;
    }

    @Override
    public void loop() {
        robot.resetCache();

        Robot.follower.update(); // Update Pedro Pathing
        robot.intake.update();
        robot.shooter.update();
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        //        loopCount++;
        //        if (loopCount % 5 == 0) {
        //            dashboardTelemetry.addData("Path State", pathState);
        //            dashboardTelemetry.addData("Distance (in)", robot.shooter.llDistance);
        //            dashboardTelemetry.addData("Flywheel RPM",
        // -robot.shooter.turretMotorLeft.getVelocity());
        //            dashboardTelemetry.update();
        //            dashboardTelemetry.addData("Track State", robot.shooter.trackState);
        //            dashboardTelemetry.addData("Turret Output", robot.shooter.turretOutput);
        //            dashboardTelemetry.addData("Turret Error",
        // Math.toDegrees(robot.shooter.turretErrorRad));
        //            dashboardTelemetry.addData("Target RPM", robot.shooter.targetVelocity);
        //            dashboardTelemetry.addData("Actual RPM",
        // -robot.shooter.turretMotorLeft.getVelocity());
        //            dashboardTelemetry.addData("Distance (in)", robot.shooter.llDistance);
        //            dashboardTelemetry.addData("Velocity OK", robot.shooter.velocityReached());
        //            dashboardTelemetry.addData("Aimed", robot.shooter.isAimed());
        //            dashboardTelemetry.addData("encoder pos",
        // robot.shooter.turretEncoder.getCurrentPosition());
        //            dashboardTelemetry.addData("heading", robot.follower.getHeading());
        //            dashboardTelemetry.addData("0. POSE", Robot.follower.getPose());
        //            dashboardTelemetry.update();
        //        }

        dashboardTelemetry.update();
    }

    @Override
    public void stop() {
        Robot.transitionPose = Robot.follower.getPose();
        Robot.alliance = Robot.Alliance.BLUE;
        robot.shooter.LL_TURRET_OFFSET_DEG = 0;
        robot.shooter.stopOverride();
    }

    public static class Paths {
        public PathChain shootPreload;
        public PathChain grab2MID;
        public PathChain grab2;
        public PathChain grab2GATE;
        public PathChain shoot2;
        public PathChain gate1;
        public PathChain shoot3;
        public PathChain grab3;
        public PathChain shoot4;

        public Paths(Follower follower) {
            shootPreload =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Pose(20.923, 123.133), new Pose(56.993, 96.392)))
                            .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(144))
                            .build();

            grab2MID =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Pose(56.993, 96.392), new Pose(54.895, 60.262)))
                            .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                            .build();

            grab2 =
                    follower.pathBuilder()
                            .addPath(new BezierLine(new Pose(54.895, 60.262), new Pose(16, 59.881)))
                            .setConstantHeadingInterpolation(Math.toRadians(180))
                            .build();

            grab2GATE =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierCurve(
                                            new Pose(12.685, 59.881),
                                            new Pose(30.656, 55.189),
                                            new Pose(18.752, 66.231)))
                            .setConstantHeadingInterpolation(Math.toRadians(180))
                            .build();

            shoot2 =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierCurve(
                                            new Pose(18.752, 66.231),
                                            new Pose(52.364, 61.594),
                                            new Pose(55.000, 95.000)))
                            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                            .build();

            gate1 =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierCurve(
                                            new Pose(55.000, 95.000),
                                            new Pose(37.161, 63.811),
                                            grabFromGate))
                            .setLinearHeadingInterpolation(
                                    Math.toRadians(144), grabFromGate.getHeading())
                            .build();

            shoot3 =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierCurve(
                                            new Pose(7.853, 58.545),
                                            new Pose(39.297, 65.871),
                                            new Pose(55.000, 95.000)))
                            .setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(144))
                            .build();

            grab3 =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierCurve(
                                            new Pose(55.000, 95.000),
                                            new Pose(67.185, 82.510),
                                            new Pose(20.678, 84.252)))
                            .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                            .build();

            shoot4 =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Pose(20.678, 84.252), new Pose(47.517, 102.175)))
                            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                            .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // shoot preload
                if (!pathingOnly) {
                    if (singleton) {
                        robot.intake.rest();
                        robot.shooter.openGate();
                        singleton = false;
                    }
                }
                robot.shooter.update();
                robot.shooter.shooting = true;
                //
                if (!Robot.follower.isBusy() && robot.shooter.velocityReached() && !pathingOnly) {
                    if (!Robot.follower.isBusy()
                            && timer.seconds() > 0.7
                            && robot.shooter.velocityReached()) {
                        robot.intake.shoot();

                    } else {
                        robot.intake.rest();
                    }
                    if (robot.intake.isEmpty()) {
                        if (timer2.seconds() > 0.5) {
                            Robot.follower.followPath(paths.grab2MID);
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
                if (!pathingOnly) {
                    robot.shooter.shooting = false;
                    robot.intake.pullBallsHard();
                    robot.shooter.closeGate();
                }
                if (!Robot.follower.isBusy()) {
                    Robot.follower.followPath(paths.grab2);
                    Robot.follower.setMaxPower(1);
                    setPathState(102);
                }
                break;
            case 102:
                if (!pathingOnly) {
                    robot.shooter.shooting = false;
                    robot.intake.pullBallsHard();
                    robot.shooter.closeGate();
                }
                if (!Robot.follower.isBusy()) {
                    Robot.follower.followPath(paths.grab2GATE);
                    Robot.follower.setMaxPower(1);
                    setPathState(1);
                }
                break;
            case 1:
                // go to grab 1
                if (!pathingOnly) {
                    robot.intake.rest();
                }
                if (!Robot.follower.isBusy()) {
                    Robot.follower.followPath(paths.shoot2);
                    Robot.follower.setMaxPower(1);
                    setPathState(2);
                }
                break;
            case 2:
                if (singleton) {
                    robot.intake.rest();
                    robot.shooter.openGate();
                    singleton = false;
                }
                robot.shooter.update();
                robot.shooter.shooting = true;
                //                robot.shooter.turretLocked=true;
                if (!Robot.follower.isBusy() && robot.shooter.velocityReached() && !pathingOnly) {
                    //                    robot.shooter.turretLocked=false;
                    if (!Robot.follower.isBusy()
                            && timer.seconds() > 0.7
                            && robot.shooter.velocityReached() /* && robot.shooter.isAimed() */) {
                        robot.intake.shoot();

                    } else {
                        robot.intake.rest();
                    }

                    if (robot.intake.isEmpty()) {
                        if (timer2.seconds() > 0.5) {
                            Robot.follower.followPath(paths.gate1);
                            Robot.follower.setMaxPower(0.72);
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
            case 301:
                if (!pathingOnly) {
                    robot.intake.pullBallsHard();
                    robot.shooter.shooting = false;
                    robot.shooter.closeGate();
                }
                if (!Robot.follower.isBusy()) {
                    Robot.follower.setMaxPower(1);
                    setPathState(3);
                }
                break;
            case 3:
                // go to grab 2
                if (!pathingOnly) {
                    robot.intake.pullBallsHard();
                    robot.shooter.shooting = false;
                    robot.shooter.closeGate();
                }

                if (!Robot.follower.isBusy() && autoTimer.seconds() > 4) {
                    Robot.follower.followPath(paths.shoot3);
                    Robot.follower.setMaxPower(1);

                    setPathState(4);
                }
                break;
            case 4:
                if (singleton) {
                    robot.intake.rest();
                    robot.shooter.openGate();
                    singleton = false;
                }
                robot.shooter.update();
                robot.shooter.shooting = true;
                if (!Robot.follower.isBusy() && robot.shooter.velocityReached() && !pathingOnly) {
                    if (!Robot.follower.isBusy()
                            && timer.seconds() > 0.7
                            && robot.shooter.velocityReached()) {
                        robot.intake.shoot();

                    } else {
                        robot.intake.rest();
                    }

                    if (robot.intake.isEmpty()) {
                        if (timer2.seconds() > 0.4) {
                            if (gateLoops < targetGateLoops) {
                                setPathState(3);
                                Robot.follower.followPath(paths.gate1);
                                Robot.follower.setMaxPower(0.72);
                                gateLoops++;

                            } else {
                                setPathState(5);
                                Robot.follower.followPath(paths.grab3);
                            }
                        }
                    } else {
                        timer2.reset();
                    }
                } else {
                    timer.reset();
                }
                break;
            case 5:
                // get spike 3
                if (!pathingOnly) {
                    robot.intake.pullBallsHard();
                    robot.shooter.shooting = false;
                    robot.shooter.closeGate();
                }
                if (!Robot.follower.isBusy()) {
                    Robot.follower.followPath(paths.shoot4);
                    setPathState(6);
                }
                break;
            case 6:
                if (singleton) {
                    robot.intake.rest();
                    robot.shooter.openGate();
                    singleton = false;
                }

                robot.shooter.update();
                robot.shooter.shooting = true;
                if (!Robot.follower.isBusy() && robot.shooter.velocityReached() && !pathingOnly) {
                    if (!Robot.follower.isBusy()
                            && timer.seconds() > 0.7
                            && robot.shooter.velocityReached()) {
                        robot.intake.shoot();

                    } else {
                        robot.intake.rest();
                    }

                    if (robot.intake.isEmpty()) {
                        if (timer2.seconds() > 0.4) {
                            robot.shooter.shooting = false;
                            if (timer.seconds() < 2) {
                                Robot.follower.followPath(paths.gate1);
                            } else {
                                Robot.follower.setMaxPower(0);
                                Robot.transitionPose = Robot.follower.getPose();
                            }
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
        autoTimer.reset();
        singletonRest = true;
        singleton = true;
        pathState = pState;
        pathTimer.reset();
    }
}
