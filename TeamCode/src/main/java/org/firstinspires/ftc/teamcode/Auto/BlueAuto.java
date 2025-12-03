package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Auto", group = "0. Autonomous")
@Configurable // Panels
public class BlueAuto extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public Robot robot; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.revolver.liftReset();

        robot.turret.tracking = true;
        robot.turret.enableCamera();

        Robot.follower = Constants.createFollower(hardwareMap);
        Robot.follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(Robot.follower); // Build paths
    }

    @Override
    public void loop() {
        Robot.follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        dashboardTelemetry.addData("Path State", pathState);
        dashboardTelemetry.addData("X", Robot.follower.getPose().getX());
        dashboardTelemetry.addData("Y", Robot.follower.getPose().getY());
        dashboardTelemetry.addData("Heading", Robot.follower.getPose().getHeading());
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
                            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
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
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            // ---------- PRELOAD ----------
            case 0:
                Robot.follower.followPath(paths.scorePreload);
                if (!Robot.follower.isBusy()) {
                    changePathState(1);
                }
                break;

            // ---------- FIRST LINE ----------
            case 1:
                Robot.follower.followPath(paths.grabFirstLine);
                if (!Robot.follower.isBusy()) {
                    changePathState(2);
                }
                break;

            case 2:
                Robot.follower.followPath(paths.scoreFirstLine);
                if (!Robot.follower.isBusy()) {
                    changePathState(3);
                }
                break;

            // ---------- SECOND LINE ----------
            case 3:
                Robot.follower.followPath(paths.grabSecondLine);
                if (!Robot.follower.isBusy()) {
                    changePathState(4);
                }
                break;

            case 4:
                Robot.follower.followPath(paths.scoreSecondLine);
                if (!Robot.follower.isBusy()) {
                    changePathState(5);
                }
                break;

            // ---------- THIRD LINE ----------

            case 5:
                Robot.follower.followPath(paths.grabThirdLine);
                if (!Robot.follower.isBusy()) {
                    changePathState(6);
                }
                break;

            case 6:
                Robot.follower.followPath(paths.scoreThirdLine);
                if (!Robot.follower.isBusy()) {
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
