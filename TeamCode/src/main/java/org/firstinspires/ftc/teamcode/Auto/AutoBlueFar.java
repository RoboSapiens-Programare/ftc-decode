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
import org.firstinspires.ftc.teamcode.Robot.uV;

@Autonomous(name = "Auto Blue Far - LFT", group = "Autonomous")
@Config
public class AutoBlueFar extends OpMode {
    private int loopCount = 0;
    private int pathState; // Current autonomous path state (state machine)
    long lastTime = System.nanoTime();
    private double averagedFrequency = 50.0; // Seed it with an expected baseline (e.g., 50-60Hz)
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private ElapsedTime pathTimer; // Timer for path state machine
    private static Paths paths; // Paths defined in the Paths class
    private Robot robot;

    private enum ShootStates {
        INACTIVE,
        SPOOL_UP,
        SHOOTING,
        COMPLETED
    };

    private enum IntakeStates {
        INACTIVE,
        PULLING,
        COMPLETED
    };

    private ShootStates shootState = ShootStates.INACTIVE;
    private IntakeStates intakeState = IntakeStates.INACTIVE;
    private boolean intakeTimeoutReset = false;

    private final ElapsedTime intakeTimeout = new ElapsedTime();

    private int gateCycleCounter = 0;
    private static final int MAX_GATE_CYCLES = 3;

    private final ElapsedTime emergencyTimer = new ElapsedTime();
    private boolean timeRanOut = false;

    private final ElapsedTime stabilizationTimer = new ElapsedTime();
    private final ElapsedTime minimumShootTimer = new ElapsedTime();

    private final Pose[] LEAVE_POINTS = {
        // inside close shoot zone
        new Pose(40.000, 130.000, Math.toRadians(180)),
        new Pose(59.000, 105.000, Math.toRadians(180)),
        new Pose(60.000, 130.000, Math.toRadians(180)),

        // mid field
        new Pose(20.000, 95.000, Math.toRadians(180)),
        new Pose(20.000, 70.000, Math.toRadians(180)),
        new Pose(35.000, 75.000, Math.toRadians(180)),

        // far field
        new Pose(48.000, 72.000, Math.toRadians(180)),
        new Pose(25.000, 45.000, Math.toRadians(180)),
        new Pose(53.000, 35.000, Math.toRadians(180)),
        new Pose(35.000, 15.000, Math.toRadians(180)),
        new Pose(52.000, 22.000, Math.toRadians(180)),
    };

    public PathChain[][] gateChains = new PathChain[3][2];

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        robot.shooter.init();
        robot.shooter.goToAngle(Math.toRadians(26));

        Robot.alliance = Robot.Alliance.BLUE;

        // Determine starting heading: prefer geometric heading when a path exists, otherwise fall
        // back to explicit startPoint values
        Robot.follower.setStartingPose(new Pose(53.000, 9.000, Math.toRadians(90.000)));

        pathTimer = new ElapsedTime();
        paths = new Paths(Robot.follower); // Build paths

        gateChains[0] = new PathChain[] {paths.grabGate, paths.shootGate};
        gateChains[1] = new PathChain[] {paths.grabGate2, paths.shootGate2};
        gateChains[2] = new PathChain[] {paths.grabGate3, paths.shootGate3};
    }

    @Override
    public void start() {
        emergencyTimer.reset();
        timeRanOut = false;
    }

    @Override
    public void loop() {
        robot.resetCache();

        robot.update();

        if (!timeRanOut) pathState = autonomousPathUpdate(); // Update autonomous state machine

        autoPark();
        // Log values to Panels and Driver Station
        updateTelemetry();
    }

    @Override
    public void stop() {
        Robot.transitionPose = Robot.follower.getPose();
        robot.shooter.stopOverride();
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

        public PathChain grabHuman;
        public PathChain shootHuman;
        public PathChain grabPPG;
        public PathChain shootPPG;
        public PathChain grabGate;
        public PathChain shootGate;
        public PathChain grabGate2;
        public PathChain shootGate2;
        public PathChain grabGate3;
        public PathChain shootGate3;

        public Paths(Follower follower) {
            grabHuman =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierCurve(
                                            new Pose(53.000, 9.000),
                                            new Pose(62.024, 18.636),
                                            new Pose(12.000, 11.247)))
                            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                            .build();

            shootHuman =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Pose(12.000, 11.247), new Pose(54.000, 17.999)))
                            .setConstantHeadingInterpolation(Math.toRadians(180))
                            .build();

            grabPPG =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierCurve(
                                            new Pose(54.000, 17.999),
                                            new Pose(47.561, 38.058),
                                            new Pose(18.673, 36.325)))
                            .setConstantHeadingInterpolation(Math.toRadians(180))
                            .build();

            shootPPG =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Pose(18.673, 36.325), new Pose(54.000, 17.999)))
                            .setConstantHeadingInterpolation(Math.toRadians(180))
                            .build();

            grabGate =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierCurve(
                                            new Pose(54.000, 17.999),
                                            new Pose(9.000, 5.044),
                                            new Pose(9.123, 29.115)))
                            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                            .build();

            shootGate =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Pose(9.123, 29.115), new Pose(54.000, 17.999)))
                            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                            .build();

            grabGate2 =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierCurve(
                                            new Pose(54.000, 17.999),
                                            new Pose(62.701, 8.000),
                                            new Pose(12.000, 11.247)))
                            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                            .build();

            shootGate2 =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Pose(12.000, 11.247), new Pose(54.000, 17.999)))
                            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                            .build();

            grabGate3 =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierCurve(
                                            new Pose(54.000, 17.999),
                                            new Pose(58.898, 30.440),
                                            new Pose(9.000, 34.000)))
                            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                            .build();

            shootGate3 =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Pose(9.000, 34.000), new Pose(54.000, 17.999)))
                            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                            .build();
        }
    }

    private void autoShoot() {
        switch (shootState) {
            case INACTIVE:
                robot.shooter.shooting = true;
                shootState = ShootStates.SPOOL_UP;
                break;
            case SPOOL_UP:
                if (robot.shooter.velocityReached() && !Robot.follower.isBusy()) {
                    robot.shooter.openGate();
                    shootState = ShootStates.SHOOTING;
                    stabilizationTimer.reset();
                    minimumShootTimer.reset();
                }
                break;
            case SHOOTING:
                if (Robot.follower.isBusy()) {
                    robot.intake.rest();
                    stabilizationTimer.reset();
                    minimumShootTimer.reset();
                    break;
                }

                if (stabilizationTimer.milliseconds() < uV.STABILIZATION_TIMEOUT_MS * 2) {
                    break;
                }

                robot.intake.shoot();

                if (robot.intake.isEmpty()
                        && minimumShootTimer.milliseconds() > uV.MINIMUM_SHOOT_TIMEOUT_MS) {
                    shootState = ShootStates.COMPLETED;

                    robot.shooter.closeGate();
                    robot.shooter.shooting = false;
                }
                break;
        }
    }

    private void autoIntake() {
        switch (intakeState) {
            case INACTIVE:
                intakeState = IntakeStates.PULLING;
                intakeTimeoutReset = false;
                break;
            case PULLING:
                robot.intake.pullBallsHard();
                if (!Robot.follower.isBusy() && !intakeTimeoutReset) {
                    intakeTimeoutReset = true;
                    intakeTimeout.reset();
                }

                if (robot.intake.isFull()
                        || (intakeTimeout.milliseconds() > uV.INTAKE_TIMEOUT_MS
                                && intakeTimeoutReset)) intakeState = IntakeStates.COMPLETED;
                break;
            case COMPLETED:
                break;
        }
    }

    private PathChain generateEmergencyLeavePath() {
        // Grab the absolute latest position tracking from Pedro Pathing
        Pose currentPose = Robot.follower.getPose();

        Pose closestTarget = LEAVE_POINTS[0];
        double shortestDistance = Double.MAX_VALUE;

        // Loop through all points to find the closest one geometrically
        for (Pose target : LEAVE_POINTS) {
            // Calculate stand
            //                                                                    ard Euclidean
            // distance: sqrt((x2-x1)^2 + (y2-y1)^2)
            double distance =
                    Math.hypot(
                            target.getX() - currentPose.getX(), target.getY() - currentPose.getY());

            if (distance < shortestDistance) {
                shortestDistance = distance;
                closestTarget = target;
            }
        }

        // Smoothly interpolate from wherever the robot physically is right now
        // to the closest safe zone heading
        return Robot.follower
                .pathBuilder()
                .addPath(new BezierLine(currentPose, closestTarget))
                .setLinearHeadingInterpolation(currentPose.getHeading(), closestTarget.getHeading())
                .build();
    }

    private void autoPark() {
        if (emergencyTimer.seconds() >= uV.AUTOPARK_TIMEOUT_MS && !timeRanOut) {
            // DO NOT autopark if going for score (would win 9 points instead of 6)
            timeRanOut = true;

            if (!Robot.follower.isBusy()
                    && (shootState == ShootStates.SPOOL_UP || shootState == ShootStates.SHOOTING)) {
                return;
            }

            Robot.follower.breakFollowing();
            Robot.follower.followPath(generateEmergencyLeavePath());
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                autoShoot();
                if (shootState == ShootStates.COMPLETED) {
                    Robot.follower.followPath(paths.grabHuman, true);
                    robot.shooter.goToAngle(Math.toRadians(-72));
                    setPathState(1);
                }

                break;
            case 1:
                autoIntake();
                if (intakeState == IntakeStates.COMPLETED) {
                    setPathState(2);
                }

                break;
            case 2:
                Robot.follower.followPath(paths.shootHuman, true);
                setPathState(3);
                break;
            case 3:
                autoShoot();

                if (shootState == ShootStates.COMPLETED) {
                    setPathState(4);
                }
                break;
            case 4:
                Robot.follower.followPath(paths.grabPPG, true);
                setPathState(5);
                break;
            case 5:
                autoIntake();
                if (intakeState == IntakeStates.COMPLETED) {
                    setPathState(6);
                }
                break;
            case 6:
                Robot.follower.followPath(paths.shootPPG, true);
                setPathState(7);
                break;
            case 7:
                autoShoot();

                if (shootState == ShootStates.COMPLETED) {
                    setPathState(8);
                }
                break;
            case 8: // begin gate cycle
                ++gateCycleCounter;

                Robot.follower.followPath(gateChains[(gateCycleCounter - 1) % 3][0], true);
                setPathState(9);
                break;
            case 9:
                autoIntake();

                if (intakeState == IntakeStates.COMPLETED) {
                    setPathState(10);
                }
                break;
            case 10:
                Robot.follower.followPath(gateChains[(gateCycleCounter - 1) % 3][1], true);
                setPathState(11);
                break;
            case 11:
                autoShoot();

                if (shootState == ShootStates.COMPLETED) {
                    if (gateCycleCounter >= MAX_GATE_CYCLES) {
                        // exit
                        setPathState(13);
                    } else {
                        setPathState(8);
                    }
                }
                break;
            case 13:
                requestOpModeStop();
                pathState = -1;
                break;
        }
        return pathState;
    }

    public void setPathState(int pState) {
        shootState = ShootStates.INACTIVE;
        intakeState = IntakeStates.INACTIVE;
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
        //        dashboardTelemetry.addData("desired angle",
        // robot.shooter.getTargetFieldAngleRadStatic());
        //        dashboardTelemetry.addData("Sensor1",
        // robot.intake.sensorIntake.getDistance(DistanceUnit.CM));
        //        dashboardTelemetry.addData("Sensor2",
        // robot.intake.sensorMid.getDistance(DistanceUnit.CM));
        //        dashboardTelemetry.addData("Sensor3",
        // robot.intake.sensorOuttake.getDistance(DistanceUnit.CM));
        dashboardTelemetry.addData("State", pathState);
        //        dashboardTelemetry.addData("Follower busy", Robot.follower.isBusy());
        dashboardTelemetry.addData("Distance (in)", robot.shooter.distance);
        dashboardTelemetry.addData("Flywheel RPM", -robot.shooter.turretMotorLeft.getVelocity());
        //        dashboardTelemetry.addData("Track State", robot.shooter.trackState);
        //        dashboardTelemetry.addData("Turret Output", robot.shooter.turretOutput);
        //        dashboardTelemetry.addData("Turret Error",
        // Math.toDegrees(robot.shooter.turretErrorRad));
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
