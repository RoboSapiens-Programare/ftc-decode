package org.firstinspires.ftc.teamcode.Auto;

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

@Autonomous(name = "Red Auto Close", group = "0. Autonomous")
@Configurable // Panels
public class RedAutoClose extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public Robot robot; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private boolean singleton = true;
    private boolean singletonoverride = true;
    private boolean stopsingleton1 = true;
    private boolean stopsingleton2 = true;
    private boolean stopsingleton3 = true;
    private boolean stopsingleton4 = true;
    private boolean stopsingleton5 = true;
    private boolean driveSingleton = true;
    private int motifPosition = 0;
    private ElapsedTime loadBallTimer = new ElapsedTime();
    private int shootStep = 0;
    private ElapsedTime stopTimer = new ElapsedTime();
    private ElapsedTime autoTimer = new ElapsedTime();

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.revolver.liftReset();
        robot.revolver.mode = Revolver.Mode.INTAKE;

        robot.turret.tracking = true;
        robot.turret.enableCamera();

        robot.intake.enableCamera();

        Robot.alliance = Robot.Alliance.RED;

        Robot.follower = Constants.createFollower(hardwareMap);
        Robot.follower.setStartingPose(new Pose(122, 123, Math.toRadians(36)));

        paths = new Paths(Robot.follower); // Build paths
        Robot.follower.activateAllPIDFs();
        robot.revolver.home();
    }

    @Override
    public void loop() {
        if (robot.revolver.mode == Revolver.Mode.OUTTAKE && robot.revolver.isMotif()) {
            robot.revolver.setTargetSlot(robot.revolver.getSlotByMotifPosition(motifPosition));
        } else if (robot.revolver.mode == Revolver.Mode.OUTTAKE) {
            robot.revolver.setTargetSlot(robot.revolver.getFullSlot());
        } else {
            robot.revolver.setTargetSlot(robot.revolver.getFreeSlot());
        }

        robot.turret.turretRotationServo.setPower(-1);

        robot.intake.update();
        robot.revolver.update();
        robot.turret.updateVelocity();

        if (driveSingleton) {
            Robot.follower.update(); // Update Pedro Pathing
        }

        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        dashboardTelemetry.addData("Path State", pathState);
        dashboardTelemetry.addData("slot 0", robot.revolver.colorList[0]);
        dashboardTelemetry.addData("slot 1", robot.revolver.colorList[1]);
        dashboardTelemetry.addData("slot 2", robot.revolver.colorList[2]);
        dashboardTelemetry.addData("X", Robot.follower.getPose().getX());
        dashboardTelemetry.addData("Y", Robot.follower.getPose().getY());
        dashboardTelemetry.addData("Heading", Robot.follower.getPose().getHeading());
        dashboardTelemetry.addData("is busy", Robot.follower.isBusy());
        dashboardTelemetry.addData("is stuck", Robot.follower.isRobotStuck());
        dashboardTelemetry.addData(
                "target reached score preload", targetReached(paths.scorePreload));
        dashboardTelemetry.addData("timer", autoTimer.seconds());
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

    @Override
    public void start() {
        autoTimer.reset();
    }

    public boolean targetReached(PathChain path) {
        double dx = path.endPose().getX() - Robot.follower.getPose().getX();
        double dy = path.endPose().getY() - Robot.follower.getPose().getY();
        double d = Math.sqrt(dx * dx + dy * dy);

        double dh = path.endPose().getHeading() - Robot.follower.getHeading();

        return d < 3.5 && Math.abs(dh) < 10.0 / 360 * (Math.PI * 2);
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
                                    new BezierLine(
                                            new Pose(122, 123),
                                            new Pose(144-41.000, 102.000)))
                            .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(90+45))
                            .build();

            grabFirstLine =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierCurve(
                                            new Pose(144-41.000, 102.000),
                                            new Pose(144-130.670, 82.500),
                                            new Pose(144-17.000, 78.000)))
                            .setLinearHeadingInterpolation(Math.toRadians(90+45), Math.toRadians(0))
                            .build();

            scoreFirstLine =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Pose(144-17.000, 84.000), new Pose(144-41.000, 102.000)))
                            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90+45))
                            .build();

            grabSecondLine =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierCurve(
                                            new Pose(144-41.000, 102.000),
                                            new Pose(144-130.670, 56.000),
                                            new Pose(144-17.000, 56.300)))
                            .setLinearHeadingInterpolation(Math.toRadians(90+45), Math.toRadians(0))
                            .build();

            scoreSecondLine =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Pose(144-17.000, 60.000), new Pose(144-41.000, 102.000)))
                            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90+45))
                            .build();

            grabThirdLine =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierCurve(
                                            new Pose(144-41.000, 102.000),
                                            new Pose(144-130.670, 35.000),
                                            new Pose(144-16.000, 30.000)))
                            .setLinearHeadingInterpolation(Math.toRadians(90+45), Math.toRadians(0))
                            .build();

            scoreThirdLine =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Pose(144-17.000, 35.000), new Pose(144-41.000, 102.000)))
                            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90+45))
                            .build();
        }
    }

    public void changePathState(int x) {
        pathState = x;
        singleton = true;
        singletonoverride = true;
        stopsingleton2 = true;
        stopsingleton1 = true;
        stopsingleton3 = true;
        stopsingleton4 = true;
        stopsingleton5 = true;
        stopTimer.reset();
        shootStep = 0;

        robot.intake.intakeMotor.setPower(0);
        robot.turret.turretMotor.setVelocity(0);

        if (x % 2 == 1) {
            robot.revolver.reset();
            robot.revolver.home();
        }
    }

    private void sugiBileCuDintii() {
        // nu ma intreba despre aceste 2 ifuri
        // sunt oribile da merg

        if (Robot.follower.getPose().getX() > 144-40
                && Robot.follower.getPose().getX() < 144-39.5) {
            if (stopsingleton1 && stopsingleton2) {
                Robot.follower.drivetrain.breakFollowing();
                stopsingleton1 = false;
                driveSingleton = false;
                stopTimer.reset();
            }
            if (stopTimer.seconds() > 1 && stopsingleton2) {
                dashboardTelemetry.addData("step", "grab1");
                robot.revolver.setSlotColor((byte) 1, ColorEnum.UNDEFINED);
                robot.revolver.setSlotColor((byte) 2, ColorEnum.UNDEFINED);

                stopsingleton2 = false;
                Robot.follower.resumePathFollowing();
                stopsingleton1 = true;
                driveSingleton = true;
                stopTimer.reset();
            }
        }

        // pune pe pozitie aici

        if (stopTimer.seconds() > .225 && !stopsingleton2 && stopsingleton3) {
            dashboardTelemetry.addData("step", "grab2");
            if (stopsingleton4) {
                Robot.follower.drivetrain.breakFollowing();
                stopsingleton4 = false;
                driveSingleton = false;
                robot.revolver.setSlotColor((byte) 2, ColorEnum.UNDEFINED);
            }
            if (stopTimer.seconds() > 1.5) {
                stopsingleton3 = false;
                Robot.follower.resumePathFollowing();
                stopsingleton4 = true;
                driveSingleton = true;
                stopTimer.reset();
            }
        }

        if (stopTimer.seconds() > .3 && !stopsingleton2 && !stopsingleton3 && stopsingleton5) {
            dashboardTelemetry.addData("step", "grab3");
            if (stopsingleton4) {
                Robot.follower.drivetrain.breakFollowing();
                stopsingleton4 = false;
                driveSingleton = false;
            }
            if (stopTimer.seconds() > .7) {
                stopsingleton4 = false;
                Robot.follower.resumePathFollowing();
                stopsingleton5 = false;
                driveSingleton = true;
                stopTimer.reset();
            }
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            // ---------- PRELOAD ----------
            case 0:
                if (singletonoverride) {
                    for (byte i = 0; i < robot.revolver.colorList.length; i++) {
                        if (robot.revolver.colorList[i] == ColorEnum.UNDEFINED) {
                            robot.revolver.colorList[i] = ColorEnum.PURPLE;
                        }
                    }
                    singletonoverride = false;
                }

                if (singleton) {
                    Robot.follower.followPath(paths.scorePreload);
                    robot.revolver.mode = Revolver.Mode.OUTTAKE;
                    singleton = false;
                }
                if (targetReached(paths.scorePreload)) {

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
                    robot.revolver.mode = Revolver.Mode.INTAKE;
                    robot.intake.intakeMotor.setPower(1);
                    robot.turret.turretMotor.setPower(0);
                }

                sugiBileCuDintii();


                //                if (targetReached(paths.grabFirstLine) ||
                // robot.revolver.getBallCount() == 3) {
                //                    changePathState(2);
                //                }

                if (targetReached(paths.grabFirstLine) && stopTimer.seconds() > 1 &&  !stopsingleton2 && !stopsingleton3 && !stopsingleton5) {
                    changePathState(2);
                }
                break;

            case 2:
                if (singletonoverride) {
                    for (byte i = 0; i < robot.revolver.colorList.length; i++) {
                        if (robot.revolver.colorList[i] == ColorEnum.UNDEFINED) {
                            robot.revolver.colorList[i] = ColorEnum.PURPLE;
                        }
                    }
                    singletonoverride = false;
                }

                if (singleton) {
                    Robot.follower.followPath(paths.scoreFirstLine);
                    robot.revolver.mode = Revolver.Mode.OUTTAKE;
                    singleton = false;
                }
                if (targetReached(paths.scoreFirstLine)) {

                    if (robot.revolver.getBallCount() == 0) {
                        changePathState(3);
                    }

                    if (shootStep >= 0) {
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
                                changePathState(3);
                            }
                        }
                    }
                }
                break;

            // ---------- SECOND LINE ----------
            case 3:
                if (singleton) {
                    Robot.follower.followPath(paths.grabSecondLine);
                    singleton = false;
                    robot.revolver.mode = Revolver.Mode.INTAKE;
                    robot.intake.intakeMotor.setPower(1);
                    robot.turret.turretMotor.setPower(0);
                }

                sugiBileCuDintii();


                //                if (targetReached(paths.grabFirstLine) ||
                // robot.revolver.getBallCount() == 3) {
                //                    changePathState(2);
                //                }

                if (targetReached(paths.grabSecondLine) && stopTimer.seconds() > 1 &&  !stopsingleton2 && !stopsingleton3 && !stopsingleton5) {
                    changePathState(4);
                }
                break;

            case 4:
                if (singletonoverride) {
                    for (byte i = 0; i < robot.revolver.colorList.length; i++) {
                        if (robot.revolver.colorList[i] == ColorEnum.UNDEFINED) {
                            robot.revolver.colorList[i] = ColorEnum.PURPLE;
                        }
                    }
                    singletonoverride = false;
                }

                if (singleton) {
                    Robot.follower.followPath(paths.scoreSecondLine);
                    robot.revolver.mode = Revolver.Mode.OUTTAKE;
                    singleton = false;
                }
                if (targetReached(paths.scoreSecondLine)) {

                    if (robot.revolver.getBallCount() == 0) {
                        changePathState(5);
                    }

                    if (shootStep >= 0) {
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
                                changePathState(5);
                            }
                        }
                    }
                }
                break;

            // ---------- THIRD LINE ----------

            case 5:
                if (singleton) {
                    Robot.follower.followPath(paths.grabThirdLine);
                    singleton = false;
                    robot.revolver.mode = Revolver.Mode.INTAKE;
                    robot.intake.intakeMotor.setPower(1);
                    robot.turret.turretMotor.setPower(0);
                }

                sugiBileCuDintii();


                //                if (targetReached(paths.grabFirstLine) ||
                // robot.revolver.getBallCount() == 3) {
                //                    changePathState(2);
                //                }

                if (targetReached(paths.grabThirdLine) && stopTimer.seconds() > 1 &&  !stopsingleton2 && !stopsingleton3 && !stopsingleton5) {
                    changePathState(6);
                }
                break;

            case 6:
                if (singletonoverride) {
                    for (byte i = 0; i < robot.revolver.colorList.length; i++) {
                        if (robot.revolver.colorList[i] == ColorEnum.UNDEFINED) {
                            robot.revolver.colorList[i] = ColorEnum.PURPLE;
                        }
                    }
                    singletonoverride = false;
                }

                if (singleton) {
                    Robot.follower.followPath(paths.scoreThirdLine);
                    robot.revolver.mode = Revolver.Mode.OUTTAKE;
                    singleton = false;
                }
                if (targetReached(paths.scoreThirdLine)) {

                    if (robot.revolver.getBallCount() == 0) {
                        changePathState(7);
                    }

                    if (shootStep >= 0) {
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
                                changePathState(7);
                            }
                        }
                    }
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
