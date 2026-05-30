/* ============================================================= *
 *                 Turtle Tracer — Auto-Generated                *
 *                                                               *
 *  Version: 2.2.0.                                              *
 *  Copyright (c) 2026 Matthew Allen                             *
 *                                                               *
 *  THIS FILE IS AUTO-GENERATED — DO NOT EDIT MANUALLY.          *
 *  Changes will be overwritten when regenerated.                *
 * ============================================================= */

package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.TeleOP.TeleOpul;

@Autonomous(name = "Auto Blue Close - LFT", group = "Autonomous")
@Config // Panels
public class AutoBlueLft extends OpMode {
    private int loopCount = 0;
    private int pathState; // Current autonomous path state (state machine)
    long lastTime = System.nanoTime();
    private double averagedFrequency = 50.0; // Seed it with an expected baseline (e.g., 50-60Hz)
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private ElapsedTime pathTimer; // Timer for path state machine
    private Paths paths; // Paths defined in the Paths class
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        robot.shooter.init();

        Robot.alliance = Robot.Alliance.BLUE;

        // Determine starting heading: prefer geometric heading when a path exists, otherwise fall back to explicit startPoint values
        Robot.follower.setStartingPose(
                new Pose(23.000, 124.000, Math.toRadians(143.000))
        );

        pathTimer = new ElapsedTime();
        paths = new Paths(Robot.follower); // Build paths
    }

    @Override
    public void loop() {
        robot.resetCache();


        robot.intake.update();
        robot.shooter.update();

        pathState = autonomousPathUpdate(); // Update autonomous state machine

        Robot.follower.update(); // Update follower
        
        // Log values to Panels and Driver Station
        updateTelemetry();
    }


    /* ============================================================= *
     *                 Turtle Tracer — Auto-Generated                *
     *                                                               *
     *  Version: 2.2.0.                                              *
     *  Copyright (c) 2026 Matthew Allen                             *
     *                                                               *
     *  THIS FILE IS AUTO-GENERATED — DO NOT EDIT MANUALLY.          *
     *  Changes will be overwritten when regenerated.                *
     * ============================================================= */

    public static class Paths {

        public PathChain shootPreload;
        public PathChain grabGPP;
        public PathChain shootGPP;
        public PathChain grabPGP;
        public PathChain shootPGP;
        public PathChain openGate;
        public PathChain shootGate;

        public Paths(Follower follower) {
            shootPreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 123.500), new Pose(54.850, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                    .build();

            grabGPP = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(54.850, 84.000), new Pose(20.000, 84.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shootGPP = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.000, 84.000), new Pose(54.850, 84.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            grabPGP = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(54.850, 84.000),
                                    new Pose(65.048, 56.007),
                                    new Pose(20.000, 61.130)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shootPGP = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.000, 61.130), new Pose(54.850, 84.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            openGate = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(54.850, 84.000), new Pose(16.835, 66.995))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .addPath(
                            new BezierLine(new Pose(16.835, 66.995), new Pose(13.866, 58.298))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            shootGate = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(13.866, 58.298), new Pose(54.850, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();
        }
    }


    private void autoShoot() {
        robot.shooter.shooting = true;

        while (!robot.intake.isEmpty()) {
            if (robot.shooter.velocityReached() && robot.shooter.isAimed()) {
                robot.shooter.openGate();
                robot.intake.shoot();
            }

            robot.shooter.update();
            robot.intake.update();
            Robot.follower.update();
        }

        robot.shooter.closeGate();
        robot.shooter.shooting = false;
    }

    private void autoIntake() {
        ElapsedTime intakeTimer = new ElapsedTime();

        intakeTimer.reset();

        while (intakeTimer.milliseconds() < 1000 && !robot.intake.isFull()) {
            robot.intake.pullBallsHard();

            robot.intake.update();
            robot.shooter.update();
            Robot.follower.update();
        }

    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                Robot.follower.followPath(paths.shootPreload, true);
                setPathState(1);
                break;
            case 1:
                if (!Robot.follower.isBusy()) {
                    autoShoot();

                    setPathState(2);
                }
                break;
            case 2:
                Robot.follower.followPath(paths.grabGPP, true);
                setPathState(3);
                break;
            case 3:
                robot.intake.pullBallsHard();

                if (!Robot.follower.isBusy() || robot.intake.isFull()) {
                    setPathState(4);
                }
                break;
            case 4:
                Robot.follower.followPath(paths.shootGPP, true);
                setPathState(5);
                break;
            case 5:
                if (!Robot.follower.isBusy()) {
                    setPathState(6);
                }
                break;
            case 6:
                Robot.follower.followPath(paths.grabPGP, true);
                setPathState(7);
                break;
            case 7:
                if (!Robot.follower.isBusy()) {
                    setPathState(8);
                }
                break;
            case 8:
                Robot.follower.followPath(paths.shootPGP, true);
                setPathState(9);
                break;
            case 9:
                if (!Robot.follower.isBusy()) {
                    setPathState(10);
                }
                break;
            case 10:
                Robot.follower.followPath(paths.openGate, true);
                setPathState(11);
                break;
            case 11:
                if (!Robot.follower.isBusy()) {
                    setPathState(12);
                }
                break;
            case 12:
                // Handled by previous chained path
                setPathState(13);
                break;
            case 13:
                Robot.follower.followPath(paths.shootGate, true);
                setPathState(14);
                break;
            case 14:
                if (!Robot.follower.isBusy()) {
                    setPathState(15);
                }
                break;
            case 15:
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

    private void updateTelemetry() {
        long currentTime = System.nanoTime();
        double loopTimeSeconds = (currentTime - lastTime) / 1_000_000_000.0;
        lastTime = currentTime;

        // Guard against division by zero on initialization anomalies
        if (loopTimeSeconds > 0) {
            double instantFrequency = 1.0 / loopTimeSeconds;

            // Exponential Moving Average Formula:
            // Alpha (0.05) determines responsiveness vs smoothness. Lower = smoother.
            double alpha = 0.05;
            averagedFrequency = (alpha * instantFrequency) + ((1.0 - alpha) * averagedFrequency);
        }

        // Limit the dashboard telemetry network updates to every 10 frames
        if (loopCount++ < 10) {
            return;
        }
        loopCount = 0;

        dashboardTelemetry.addData("Loop Hz (Avg)", Math.round(averagedFrequency));
//        dashboardTelemetry.addData("desired angle", robot.shooter.getTargetFieldAngleRadStatic());
//        dashboardTelemetry.addData("Sensor1", robot.intake.sensorIntake.getDistance(DistanceUnit.CM));
//        dashboardTelemetry.addData("Sensor2", robot.intake.sensorMid.getDistance(DistanceUnit.CM));
//        dashboardTelemetry.addData("Sensor3", robot.intake.sensorOuttake.getDistance(DistanceUnit.CM));
        dashboardTelemetry.addData("State", pathState);
//        dashboardTelemetry.addData("Follower busy", Robot.follower.isBusy());
        dashboardTelemetry.addData("Distance (in)", robot.shooter.distance);
        dashboardTelemetry.addData("Flywheel RPM", -robot.shooter.turretMotorLeft.getVelocity());
//        dashboardTelemetry.addData("Track State", robot.shooter.trackState);
//        dashboardTelemetry.addData("Turret Output", robot.shooter.turretOutput);
//        dashboardTelemetry.addData("Turret Error", Math.toDegrees(robot.shooter.turretErrorRad));
        dashboardTelemetry.addData("Target RPM", Shooter.targetVelocity);
        dashboardTelemetry.addData("Velocity OK", robot.shooter.velocityReached());
        dashboardTelemetry.addData("Aimed", robot.shooter.isAimed());
        dashboardTelemetry.addData("Path State", pathState);
        dashboardTelemetry.addData("X", Robot.follower.getPose().getX());
        dashboardTelemetry.addData("Y", Robot.follower.getPose().getY());
        dashboardTelemetry.addData("Heading", Robot.follower.getPose().getHeading());
//        dashboardTelemetry.addData("Angle pose", Robot.follower.getPose().getHeading());
//        dashboardTelemetry.addData("0. POSE", Robot.follower.getPose());

        dashboardTelemetry.update();
    }

}
