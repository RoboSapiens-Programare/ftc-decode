package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.acmerobotics.dashboard.FtcDashboard;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Revolver;
import org.firstinspires.ftc.teamcode.Robot.Utils.ColorEnum;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Auto", group = "0. Autonomous")
@Configurable // Panels
public class BlueAuto extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public Robot robot; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private boolean singleton = true;
    private int motifPosition = 0;
    private ElapsedTime loadBallTimer = new ElapsedTime();
    private int shootStep = 0;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.revolver.liftReset();
        robot.revolver.mode = Revolver.Mode.INTAKE;

        robot.turret.tracking = true;
        robot.turret.enableCamera();

        robot.intake.enableCamera();

        Robot.alliance = Robot.Alliance.BLUE;

        Robot.follower = Constants.createFollower(hardwareMap);
        Robot.follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));

        paths = new Paths(Robot.follower); // Build paths
        Robot.follower.activateAllPIDFs();
        robot.revolver.home();
    }

    @Override
    public void loop() {
        if (robot.revolver.mode== Revolver.Mode.OUTTAKE) {
            robot.revolver.setTargetSlot(robot.revolver.getSlotByMotifPosition(motifPosition));
        } else {
            robot.revolver.setTargetSlot(robot.revolver.getFreeSlot());
        }

        robot.turret.turretRotationServo.setPower(1);

        robot.intake.update();
        robot.revolver.update();
        robot.turret.updateVelocity();

        Robot.follower.update(); // Update Pedro Pathing

        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        dashboardTelemetry.addData("Path State", pathState);
        dashboardTelemetry.addData("X", Robot.follower.getPose().getX());
        dashboardTelemetry.addData("Y", Robot.follower.getPose().getY());
        dashboardTelemetry.addData("Heading", Robot.follower.getPose().getHeading());
        dashboardTelemetry.addData("is busy", Robot.follower.isBusy());
        dashboardTelemetry.addData("is stuck", Robot.follower.isRobotStuck());
        dashboardTelemetry.addData(
                "target reached score preload", targetReached(paths.scorePreload));
        dashboardTelemetry.update();
    }

    @Override
    public void init_loop() {
        // detect motif
        LLResult result = robot.turret.limelight.getLatestResult();

        if (result.isValid()) {
            for (FiducialResult fiducial : result.getFiducialResults()) {
                int id = fiducial.getFiducialId();
                if (id == 21 || id == 22 || id == 23) {
                    robot.revolver.greenPosition = id - 21;
                }
            }
        }

        if (robot.revolver.homing) {
            telemetry.addData("Status", "homing");
        }

        if (!robot.revolver.homing) {
            telemetry.addData("Status", "color sorting");

            if (robot.revolver.isSlotFull(robot.revolver.getTargetSlot())) {
                if (robot.revolver.getFreeSlot() != -1) {
                    robot.revolver.setTargetSlot(robot.revolver.getFreeSlot());
                }
            } else robot.revolver.setTargetSlot(robot.revolver.getTargetSlot());

            robot.intake.update();
        }

        if (robot.revolver.getBallCount() == 3) {
            robot.revolver.mode = Revolver.Mode.OUTTAKE;
        }

        robot.revolver.update();

        dashboardTelemetry.addData("X", Robot.follower.getPose().getX());
        dashboardTelemetry.addData("Y", Robot.follower.getPose().getY());
        dashboardTelemetry.addData("Heading", Robot.follower.getPose().getHeading());
        dashboardTelemetry.update();
    }

    public boolean targetReached(PathChain path) {
        double dx = path.endPose().getX() - Robot.follower.getPose().getX();
        double dy = path.endPose().getY() - Robot.follower.getPose().getY();
        double d = Math.sqrt(dx * dx + dy * dy);

        double dh = path.endPose().getHeading() - Robot.follower.getHeading();

        return d < 2.5 && Math.abs(dh) < 10.0 / 360 * (Math.PI * 2);
    }

    public static class Paths {
        public PathChain scorePreload;
        public PathChain grabFirstLine;
        public PathChain scoreFirstLine;
        public PathChain grabSecondLine;
        public PathChain scoreSecondLine;
        public PathChain grabThirdLine;
        public PathChain scoreThirdLine;

        public Paths(Follower follower) {
            scorePreload =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierCurve(
                                            new Pose(56.000, 8.000),
                                            new Pose(67.000, 91.000),
                                            new Pose(41.000, 102.000)))
                            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(50))
                            .build();

            grabFirstLine =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierCurve(
                                            new Pose(41.000, 102.000),
                                            new Pose(85.000, 82.500),
                                            new Pose(17.000, 84.000)))
                            .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                            .build();

            scoreFirstLine =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Pose(17.000, 84.000), new Pose(41.000, 102.000)))
                            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                            .build();

            grabSecondLine =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierCurve(
                                            new Pose(41.000, 102.000),
                                            new Pose(85.000, 56.000),
                                            new Pose(17.000, 60.000)))
                            .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                            .build();

            scoreSecondLine =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Pose(17.000, 60.000), new Pose(41.000, 102.000)))
                            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                            .build();

            grabThirdLine =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierCurve(
                                            new Pose(41.000, 102.000),
                                            new Pose(85.000, 35.000),
                                            new Pose(17.000, 35.000)))
                            .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                            .build();

            scoreThirdLine =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Pose(17.000, 35.000), new Pose(41.000, 102.000)))
                            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                            .build();
        }
    }

    public void changePathState(int x) {
        pathState = x;
        singleton = true;
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            // ---------- PRELOAD ----------
            case 0:
                if (singleton) {
                    Robot.follower.followPath(paths.scorePreload);
                    singleton = false;
                }
                if (targetReached(paths.scorePreload)) {
                    robot.revolver.mode = Revolver.Mode.OUTTAKE;

                    if (shootStep >= 0) {
                        // make sure revolver is stable before starting load sequence
                        if (shootStep == 0 && robot.revolver.isReady()) {
                            loadBallTimer.reset();
                            ++shootStep;
                        }

                        if (loadBallTimer.milliseconds() > 300 && shootStep == 1) {
                            robot.revolver.liftLoad();

                            loadBallTimer.reset();
                            ++shootStep;
                        }

                        if (loadBallTimer.milliseconds() > 500 && shootStep == 2) {
                            robot.revolver.liftReset();
                            robot.revolver.setSlotColor(
                                    robot.revolver.getTargetSlot(), ColorEnum.UNDEFINED);

                            loadBallTimer.reset();
                            ++shootStep;
                        }

                        if (robot.revolver.mihaiLimit.isPressed() && shootStep == 3) {
                            loadBallTimer.reset();
                            shootStep = -1;

                            if (motifPosition == 2) {
                                motifPosition = 0;
                            } else ++motifPosition;

                            if (robot.revolver.getBallCount() > 0) {
                                shootStep = 0;
                            } else {
                                changePathState(1);
                            }
                        }
                    }
                }
                break;

            // ---------- FIRST LINE ----------
            case 1:
                if (singleton) {
                    Robot.follower.followPath(paths.grabFirstLine);
                    singleton = false;
                    robot.intake.intakeMotor.setPower(1);
                }
                if (targetReached(paths.grabFirstLine) || robot.revolver.getBallCount() == 3) {
                    changePathState(2);
                }
                break;

            case 2:
                if (singleton) {
                    Robot.follower.followPath(paths.scoreFirstLine);
                    singleton = false;
                }
                if (targetReached(paths.scoreFirstLine)) {
                    changePathState(3);
                }
                break;

            // ---------- SECOND LINE ----------
            case 3:
                if (singleton) {
                    Robot.follower.followPath(paths.grabSecondLine);
                    singleton = false;
                }
                if (targetReached(paths.grabSecondLine)) {
                    changePathState(4);
                }
                break;

            case 4:
                if (singleton) {
                    Robot.follower.followPath(paths.scoreSecondLine);
                    singleton = false;
                }
                if (targetReached(paths.scoreSecondLine)) {
                    changePathState(5);
                }
                break;

            // ---------- THIRD LINE ----------

            case 5:
                if (singleton) {
                    Robot.follower.followPath(paths.grabThirdLine);
                    singleton = false;
                }
                if (targetReached(paths.grabThirdLine)) {
                    changePathState(6);
                }
                break;

            case 6:
                if (singleton) {
                    Robot.follower.followPath(paths.scoreThirdLine);
                    singleton = false;
                }
                if (targetReached(paths.scoreThirdLine)) {
                    changePathState(7);
                }
                break;

            // ---------- DONE ----------
            case 7:
                // robot finished all paths
                break;
        }
        return pathState;
    }
}
