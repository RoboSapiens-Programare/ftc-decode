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

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Auto Preload BLUE", group = "Autonomous")
@Configurable // Panels
public class AutoPreloadBlue extends OpMode {

    private static Robot robot;
    private int pathState; // Current autonomous path state (state machine)
    private ElapsedTime pathTimer; // Timer for path state machine
    private Paths paths;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private ElapsedTime autoTimer = new ElapsedTime();


    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        Robot.follower.setStartingPose(new Pose(72, 8, Math.toRadians(270)));

        pathTimer = new ElapsedTime();
        paths = new Paths(Robot.follower); // Build paths

        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        aprilTag.setDecimation(2);

        visionPortal =
                new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "aprilWebcam"))
                        .addProcessor(aprilTag)
                        .build();

        robot.spindexer.home();

        Robot.alliance = Robot.Alliance.BLUE;
    }

    @Override
    public void init_loop() {
        robot.intake.update();
        robot.spindexer.update();

        for (AprilTagDetection detection : aprilTag.getDetections())  {
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
        }

    }

    @Override
    public void start() {
        robot.shooter.shooting = true;
        robot.spindexer.motifGoToStart();
        autoTimer.reset();
    }


    @Override
    public void loop() {
        Robot.follower.update(); // Update Pedro Pathing
        robot.spindexer.update();

        if (autoTimer.seconds() < 20) {
            robot.shooter.update();
            return;
        }

        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
    }

    @Override
    public void stop() {
        Robot.transitionPose = Robot.follower.getPose();
    }

    public static class Paths {

        public PathChain shoot;
        public PathChain leave;

        public Paths(Follower follower) {
            shoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(63.000, 9.000),
                                    new Pose(35.729, 26.808),
                                    new Pose(64.550, 38.913),
                                    new Pose(59.000, 22.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), robot.shooter.getAngle(59, 22))
                    .build();

            leave = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(59.000, 22.000), new Pose(19.000, 10.000))
                    )
                    .setLinearHeadingInterpolation(robot.shooter.getAngle(59, 22), Math.toRadians(90))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                Robot.follower.followPath(paths.shoot, true);
                setPathState(1);
                break;
            case 1:
                if (!Robot.follower.isBusy()) {
                    if (robot.spindexer.getBallCount() > 0) {
                        if (robot.shooter.isShootReady() && robot.spindexer.isReady()) {
                            robot.spindexer.shoot();
                        }
                    } else if (robot.spindexer.isReady()) {
                        setPathState(2);
                    }
                }
                break;
            case 2:
                Robot.follower.followPath(paths.leave, true);
                setPathState(3);
                break;
            case 3:
                if (!Robot.follower.isBusy()) {
                    setPathState(4);
                }
                break;
            case 4:
                requestOpModeStop();
                pathState = -1;
                break;
        }
        return pathState;
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }
}
