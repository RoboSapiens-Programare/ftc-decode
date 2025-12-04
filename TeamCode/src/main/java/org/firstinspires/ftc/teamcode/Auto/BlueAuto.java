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

    private final Pose startPose =
            new Pose(72, 120, Math.toRadians(90)); // Start Pose of our robot.
    private static final Pose scorePoseRed = new Pose(96, 95, Math.toRadians(115));
    private static final Pose scorePoseBlue = new Pose(47, 95, Math.toRadians(115));
    private static final Pose PPGPose = new Pose(100, 83.5, Math.toRadians(0));
    private static final Pose PGPPose = new Pose(100, 59.5, Math.toRadians(0));
    private static final Pose GPPPose = new Pose(100, 35.5, Math.toRadians(0));
    private static final Pose PPGPoseGrab = new Pose(130, 83.5, Math.toRadians(0));
    private static final Pose PGPPoseGrab = new Pose(130, 59.5, Math.toRadians(0));
    private static final Pose GPPPoseGrab = new Pose(130, 35.5, Math.toRadians(0));

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

        public PathChain grabPPG1;
        public PathChain grabPPG2;
        public PathChain scorePPG;
        public PathChain grabPGP1;
        public PathChain grabPGP2;
        public PathChain scorePGP;
        public PathChain grabGPP1;
        public PathChain grabGPP2;
        public PathChain scoreGPP;
        public PathChain scorePreload;

        public Paths(Follower follower) {

            // ---------- PRELOAD -----------

            // preload should make follower maintain a static position in case of the robot being pushed
            // current logic is most likely not correct

            scorePreload =
                    follower.pathBuilder()
                            .addPath(new BezierLine(scorePoseBlue, scorePoseBlue))
                            .setConstantHeadingInterpolation(scorePoseBlue.getHeading())
                            .build();

            // ---------- PPG CYCLE ----------
            grabPPG1 =
                    follower.pathBuilder()
                            .addPath(new BezierLine(scorePoseBlue, PPGPose))
                            .setLinearHeadingInterpolation(
                                    scorePoseBlue.getHeading(), PPGPose.getHeading())
                            .build();

            grabPPG2 =
                    follower.pathBuilder()
                            .addPath(new BezierLine(PPGPose, PPGPoseGrab))
                            .setLinearHeadingInterpolation(
                                    PPGPose.getHeading(), PPGPoseGrab.getHeading())
                            .build();

            scorePPG =
                    follower.pathBuilder()
                            .addPath(new BezierLine(PPGPoseGrab, scorePoseRed))
                            .setLinearHeadingInterpolation(
                                    PPGPoseGrab.getHeading(), scorePoseRed.getHeading())
                            .build();

            // ---------- PGP CYCLE ----------
            grabPGP1 =
                    follower.pathBuilder()
                            .addPath(new BezierLine(scorePoseRed, PGPPose))
                            .setLinearHeadingInterpolation(
                                    scorePoseRed.getHeading(), PGPPose.getHeading())
                            .build();

            grabPGP2 =
                    follower.pathBuilder()
                            .addPath(new BezierLine(PGPPose, PGPPoseGrab))
                            .setLinearHeadingInterpolation(
                                    PGPPose.getHeading(), PGPPoseGrab.getHeading())
                            .build();

            scorePGP =
                    follower.pathBuilder()
                            .addPath(new BezierLine(PGPPoseGrab, scorePoseRed))
                            .setLinearHeadingInterpolation(
                                    PGPPoseGrab.getHeading(), scorePoseRed.getHeading())
                            .build();

            // ---------- GPP CYCLE ----------
            grabGPP1 =
                    follower.pathBuilder()
                            .addPath(new BezierLine(scorePoseRed, GPPPose))
                            .setLinearHeadingInterpolation(
                                    scorePoseRed.getHeading(), GPPPose.getHeading())
                            .build();

            grabGPP2 =
                    follower.pathBuilder()
                            .addPath(new BezierLine(GPPPose, GPPPoseGrab))
                            .setLinearHeadingInterpolation(
                                    GPPPose.getHeading(), GPPPoseGrab.getHeading())
                            .build();

            scoreGPP =
                    follower.pathBuilder()
                            .addPath(new BezierLine(GPPPoseGrab, scorePoseRed))
                            .setLinearHeadingInterpolation(
                                    GPPPoseGrab.getHeading(), scorePoseRed.getHeading())
                            .build();
        }
    }

    public void setpathState(int x) {
        pathState = x;
    }

    public int autonomousPathUpdate() {
        switch (pathState) {

            // ---------- PRELOAD ----------
            case 0:

                //should score from start position

                Robot.follower.followPath(paths.scorePreload);

                if (!Robot.follower.isBusy()) {
                    setpathState(1);
                }
                break;

            // ---------- PPG ----------
            case 1:
                Robot.follower.followPath(paths.grabPPG1);
                if (!Robot.follower.isBusy()) {
                    setpathState(2);
                }
                break;

            case 2:
                Robot.follower.followPath(paths.grabPPG2);
                if (!Robot.follower.isBusy()) {
                    setpathState(3);
                }
                break;

            case 3:
                Robot.follower.followPath(paths.scorePPG);
                if (!Robot.follower.isBusy()) {
                    setpathState(4);
                }
                break;

            // ---------- PGP ----------
            case 4:
                Robot.follower.followPath(paths.grabPGP1);
                if (!Robot.follower.isBusy()) {
                    setpathState(5);
                }
                break;

            case 5:
                Robot.follower.followPath(paths.grabPGP2);
                if (!Robot.follower.isBusy()) {
                    setpathState(6);
                }
                break;

            case 6:
                Robot.follower.followPath(paths.scorePGP);
                if (!Robot.follower.isBusy()) {
                    setpathState(7);
                }
                break;

            // ---------- GPP ----------
            case 7:
                Robot.follower.followPath(paths.grabGPP1);
                if (!Robot.follower.isBusy()) {
                    setpathState(8);
                }
                break;

            case 8:
                Robot.follower.followPath(paths.grabGPP2);
                if (!Robot.follower.isBusy()) {
                    setpathState(9);
                }
                break;

            case 9:
                Robot.follower.followPath(paths.scoreGPP);
                if (!Robot.follower.isBusy()) {
                    setpathState(10);
                }
                break;

            // ---------- DONE ----------
            case 10:
                // robot finished all paths
                break;

        }
            return pathState;
        }
    }

