/* ============================================================= *
 *           Pedro Pathing Visualizer — Auto-Generated           *
 *                                                               *
 *  Version: 1.6.2.                                              *
 *  Copyright (c) 2026 Matthew Allen                             *
 *                                                               *
 *  THIS FILE IS AUTO-GENERATED — DO NOT EDIT MANUALLY.          *
 *  Changes will be overwritten when regenerated.                *
 * ============================================================= */

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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Robot.Utils.ColorEnum;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@Autonomous(name = "Auto Far x9 RED", group = "0. Auto")
public class AutoFar9Red extends OpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private int pathState; // Current autonomous path state (state machine)
    private ElapsedTime pathTimer; // Timer for path state machine
    private boolean singleton;
    private Paths paths; // Paths defined in the Paths class

    private static Robot robot;

    private final double ballWait = 5;
    private final double artefactMaxPower = 0.27;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime autoTimer = new ElapsedTime();

    Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        aprilTag.setDecimation(2);

        visionPortal =
                new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "aprilWebcam"))
                        .addProcessor(aprilTag)
                        .build();

        visionPortal.resumeStreaming();
        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);

        Robot.alliance = Robot.Alliance.BLUE;

        Robot.follower.setStartingPose(new Pose(63, 9, Math.toRadians(90)));

        pathTimer = new ElapsedTime();
        paths = new Paths(Robot.follower); // Build paths

        robot.spindexer.home();
    }

    @Override
    public void init_loop() {
        robot.spindexer.update();

        for (AprilTagDetection detection : aprilTag.getDetections()) {
            switch (detection.id) {
                case 21:
                    Spindexer.greenMotifPosition = 0;
                    break;
                case 22:
                    Spindexer.greenMotifPosition = 2;
                    break;
                case 23:
                    Spindexer.greenMotifPosition = 1;
                    break;
                default:
                    break;
            }
        }

        telemetry.addData("Motif pos", Spindexer.greenMotifPosition);
        telemetry.addData("Ball count", robot.spindexer.getBallCount());
        telemetry.update();

        if (robot.spindexer.getBallCount() < 3) {
            robot.spindexer.goToSlot(robot.spindexer.getFreeSlot());
            robot.intake.update();
        }

        autoTimer.reset();

        dashboardTelemetry.addData(" 0. Ball count", robot.spindexer.getBallCount());
        dashboardTelemetry.addData(" 1.   slot 0", robot.spindexer.getSlotColor(0));
        dashboardTelemetry.addData(" 2.   slot 1", robot.spindexer.getSlotColor(1));
        dashboardTelemetry.addData(" 3.   slot 2", robot.spindexer.getSlotColor(2));

        dashboardTelemetry.update();
    }

    @Override
    public void start() {
        visionPortal.stopStreaming();

    }

    @Override
    public void loop() {
        //        if (autoTimer.seconds() < 20) {
        //            return;
        //        }

        Robot.follower.update(); // Update Pedro Pathing
        robot.intake.update();
        robot.spindexer.update();
        robot.shooter.update();
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        if (!robot.shooter.shooting) {
            if (robot.spindexer.getBallCount() < 3) {
                robot.spindexer.goToSlot(robot.spindexer.getFreeSlot());
            }
        }

        // Log values to Panels and Driver Station
        dashboardTelemetry.addData("Path State", pathState);
        dashboardTelemetry.addData("X", Robot.follower.getPose().getX());
        dashboardTelemetry.addData("Y", Robot.follower.getPose().getY());
        dashboardTelemetry.addData("Heading", Robot.follower.getPose().getHeading());
        dashboardTelemetry.addData("balls", robot.spindexer.getBallCount());

        dashboardTelemetry.addData(" 0. Ball count", robot.spindexer.getBallCount());
        dashboardTelemetry.addData(" 1.   slot 0", robot.spindexer.getSlotColor(0));
        dashboardTelemetry.addData(" 2.   slot 1", robot.spindexer.getSlotColor(1));
        dashboardTelemetry.addData(" 3.   slot 2", robot.spindexer.getSlotColor(2));

        dashboardTelemetry.update();
    }

    @Override
    public void stop() {
        Robot.transitionPose = Robot.follower.getPose();
    }

    public static class Paths {

        public PathChain shootPreload;
        public PathChain goToGrab0;
        public PathChain grab00;
        public PathChain grab01;
        public PathChain grab02;
        public PathChain shoot0;
        public PathChain goToGrab1;
        public PathChain grab10;
        public PathChain grab11;
        public PathChain grab12;
        public PathChain shoot1;
        public PathChain leave;

        public Paths(Follower follower) {
            shootPreload =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Pose(144-63.000, 9.000),
                                            new Pose(144-59.000, 20.000)))
                            .setLinearHeadingInterpolation(
                                    Math.toRadians(90), robot.shooter.getAngle(144-59, 20))
                            .build();

            goToGrab0 =
                    follower.pathBuilder()
                            .addPath(new BezierLine(new Pose(144-59.000, 20.000), new Pose(144-42, 36)))
                            .setLinearHeadingInterpolation(
                                    robot.shooter.getAngle(144-59, 20), Math.toRadians(0))
                            .build();

            grab00 =
                    follower.pathBuilder()
                            .addPath(new BezierLine(new Pose(144-42, 36), new Pose(144-36, 36)))
                            .setConstantHeadingInterpolation(Math.toRadians(0))
                            .build();

            grab01 =
                    follower.pathBuilder()
                            .addPath(new BezierLine(new Pose(144-36, 36.000), new Pose(144-31.25, 36)))
                            .setConstantHeadingInterpolation(Math.toRadians(0))
                            .build();

            grab02 =
                    follower.pathBuilder()
                            .addPath(new BezierLine(new Pose(144-31.25, 36.000), new Pose(144-15, 36)))
                            .setConstantHeadingInterpolation(Math.toRadians(0))
                            .build();

            shoot0 =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Pose(144-15, 36.000), new Pose(144-60.500, 20.000)))
                            .setLinearHeadingInterpolation(
                                    Math.toRadians(0), robot.shooter.getAngle(144-59, 20))
                            .build();

            goToGrab1 =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierCurve(
                                            new Pose(144-59.000, 20.000),
                                            new Pose(48.724, 41.056),
                                            new Pose(59.110, 33.846),
                                            new Pose(50.000, 59.996),
                                            new Pose(144-42, 61)))
                            .setLinearHeadingInterpolation(
                                    robot.shooter.getAngle(144-59, 20), Math.toRadians(0))
                            .build();

            grab10 =
                    follower.pathBuilder()
                            .addPath(new BezierLine(new Pose(144-40, 61.000), new Pose(144-35, 61.000)))
                            .setConstantHeadingInterpolation(Math.toRadians(0))
                            .build();

            grab11 =
                    follower.pathBuilder()
                            .addPath(new BezierLine(new Pose(144-35, 61.000), new Pose(144-30.25, 61.000)))
                            .setConstantHeadingInterpolation(Math.toRadians(0))
                            .build();

            grab12 =
                    follower.pathBuilder()
                            .addPath(new BezierLine(new Pose(144-30.25, 61.000), new Pose(144-15, 61.000)))
                            .setConstantHeadingInterpolation(Math.toRadians(0))
                            .build();

            shoot1 =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Pose(144-15, 61.000), new Pose(144-60.500, 20.000)))
                            .setLinearHeadingInterpolation(
                                    Math.toRadians(0), robot.shooter.getAngle(144-60.5, 20))
                            .build();

            leave =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Pose(144-60.000, 20.000), new Pose(144-30.000, 20.000)))
                            .setLinearHeadingInterpolation(robot.shooter.getAngle(144-60.5, 20), Math.toRadians(0))
                            .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // GO TO PRELOAD SHOOTING
                Robot.follower.followPath(paths.shootPreload, true);
                robot.shooter.shooting = true;
                robot.spindexer.motifGoToStart();
                robot.intake.setPower(-1);
                setPathState(1);
                break;
            case 1:
                // ARRIVED TO PRELOAD SHOOTING
                if (!Robot.follower.isBusy()) {
                    if (robot.spindexer.getBallCount() > 0) {
                        if (robot.shooter.isShootReady() && robot.spindexer.isReady()) {
                            robot.spindexer.shoot(1);
                        }
                    } else if (robot.spindexer.isReady()) {
                        setPathState(2);
                    }
                }
                break;
            case 2:
                // GO TO 1ST GRAB
                Robot.follower.followPath(paths.goToGrab0, true);
                robot.shooter.shooting = false;
                robot.spindexer.home();
                robot.intake.setPower(1);
                setPathState(3);
                break;
            case 3:
                // ARRIVED
                if (!Robot.follower.isBusy()) {
//                    robot.spindexer.goToSlot(robot.spindexer.getFreeSlot());
                    if (robot.spindexer.isReady()) {
                        setPathState(4);
                    }
                }
                break;
            case 4:
                // GRAB FIRST BALL
                if (robot.spindexer.isReady() && !Robot.follower.isBusy()) {
                    Robot.follower.followPath(paths.grab00, true);
                    Robot.follower.setMaxPower(artefactMaxPower);
                    setPathState(5);
                }
                break;
            case 5:
                // ARRIVED
                if (!Robot.follower.isBusy()
                        && (robot.spindexer.getBallCount() == 1 || timer.seconds() > ballWait)
                        && robot.spindexer.isReady()) {
                    robot.spindexer.setSlotColor(1, ColorEnum.UNDEFINED);
                    robot.spindexer.setSlotColor(2, ColorEnum.UNDEFINED);
//                    robot.spindexer.goToSlot(robot.spindexer.getFreeSlot());
                    setPathState(6);
                }
                break;
            case 6:
                // GRAB SECOND BALL

                if (robot.spindexer.isReady()) {
                    Robot.follower.followPath(paths.grab01, true);
                    setPathState(7);
                }

                break;
            case 7:
                // ARRIVED
                if (!Robot.follower.isBusy()
                        && (robot.spindexer.getBallCount() == 2 || timer.seconds() > ballWait)
                        && robot.spindexer.isReady()) {
                    robot.spindexer.setSlotColor(2, ColorEnum.UNDEFINED);
//                    robot.spindexer.goToSlot(robot.spindexer.getFreeSlot());
                    if (robot.spindexer.isReady())
                    {
                        setPathState(8);
                    }
                }
                break;
            case 8:
                // GRAB THIRD
                if (robot.spindexer.isReady()) {
                    Robot.follower.setMaxPower(1);
                    Robot.follower.followPath(paths.grab02, true);
                    setPathState(9);
                }
                break;
            case 9:
                // ARRIVED
                if (!Robot.follower.isBusy()
                        && (robot.spindexer.getBallCount() == 3 || timer.seconds() > ballWait)) {
                    setPathState(10);
                }
                break;
            case 10:
                // SHOOT 1ST
                Robot.follower.followPath(paths.shoot0, true);

                robot.intake.setPower(-1);
                robot.shooter.shooting = true;
                robot.spindexer.motifGoToStart();
                setPathState(11);
                break;
            case 11:
                // ARRIVED
                if (!Robot.follower.isBusy()) {
                    if (robot.spindexer.getBallCount() > 0) {
                        if (robot.shooter.isShootReady() && robot.spindexer.isReady()) {
                            robot.spindexer.shoot(1);
                        }
                    } else if (robot.spindexer.isReady()) {
                        setPathState(12);
                    }
                }
                break;
            case 12:
                // GO TO 1ST GRAB
                Robot.follower.followPath(paths.goToGrab1, true);
                robot.shooter.shooting = false;
                robot.spindexer.home();
                robot.intake.setPower(1);
                setPathState(13);
                break;
            case 13:
                // ARRIVED
                if (!Robot.follower.isBusy()) {
//                    robot.spindexer.goToSlot(robot.spindexer.getFreeSlot());
                    if (robot.spindexer.isReady()) {
                        setPathState(14);
                    }
                }
                break;
            case 14:
                // GRAB FIRST BALL
                if (robot.spindexer.isReady() && !Robot.follower.isBusy()) {
                    Robot.follower.followPath(paths.grab10, true);
                    Robot.follower.setMaxPower(artefactMaxPower);
                    setPathState(15);
                }
                break;
            case 15:
                // ARRIVED
                if (!Robot.follower.isBusy()
                        && (robot.spindexer.getBallCount() == 1 || timer.seconds() > ballWait)
                        && robot.spindexer.isReady()) {
                    robot.spindexer.setSlotColor(1, ColorEnum.UNDEFINED);
                    robot.spindexer.setSlotColor(2, ColorEnum.UNDEFINED);
//                    robot.spindexer.goToSlot(robot.spindexer.getFreeSlot());
                    setPathState(16);
                }
                break;
            case 16:
                // GRAB SECOND BALL

                if (robot.spindexer.isReady()) {
                    Robot.follower.followPath(paths.grab11, true);
                    setPathState(17);
                }

                break;
            case 17:
                // ARRIVED
                if (!Robot.follower.isBusy()
                        && (robot.spindexer.getBallCount() == 2 || timer.seconds() > ballWait)
                        && robot.spindexer.isReady()) {
                    robot.spindexer.setSlotColor(2, ColorEnum.UNDEFINED);
//                    robot.spindexer.goToSlot(robot.spindexer.getFreeSlot());
                    if (robot.spindexer.isReady())
                    {
                        setPathState(18);
                    }
                }
                break;
            case 18:
                // GRAB THIRD
                if (robot.spindexer.isReady()) {
                    Robot.follower.setMaxPower(1);
                    Robot.follower.followPath(paths.grab12, true);
                    setPathState(19);
                }
                break;
            case 19:
                // ARRIVED
                if (!Robot.follower.isBusy()
                        && (robot.spindexer.getBallCount() == 3 || timer.seconds() > ballWait)) {
                    setPathState(20);
                }
                break;
            case 20:
                // SHOOT 1ST
                Robot.follower.followPath(paths.shoot1, true);
                Robot.follower.setMaxPower(1);
                robot.shooter.shooting = true;
                robot.spindexer.motifGoToStart();
                robot.intake.setPower(-1);
                setPathState(21);
                break;
            case 21:
                if (!Robot.follower.isBusy()) {
                    if (robot.spindexer.getBallCount() > 0) {
                        if (robot.shooter.isShootReady() && robot.spindexer.isReady()) {
                            robot.spindexer.shoot(1);
                        }
                    } else if (robot.spindexer.isReady()) {
                        setPathState(22);
                    }
                }
                break;
            case 22:
                Robot.follower.followPath(paths.leave, true);
                setPathState(23);
                break;
            case 23:
                if (!Robot.follower.isBusy()) {
                    setPathState(24);
                }
                break;
            case 24:
                robot.intake.setPower(0);
                requestOpModeStop();
                pathState = -1;
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