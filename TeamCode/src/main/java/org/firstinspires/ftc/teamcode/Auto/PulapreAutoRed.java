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

@Autonomous(name = "PURPLE RED", group = "Autonomous - Purple")
@Config
public class PulapreAutoRed extends OpMode {
    private int loopCount = 0;
    private boolean sinigermiguelphonk = true;
    private int pathState; // Current autonomous path state (state machine)
    long lastTime = System.nanoTime();
    private double averagedFrequency = 50.0; // Seed it with an expected baseline (e.g., 50-60Hz)
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private ElapsedTime pathTimer; // Timer for path state machine
    private Paths paths; // Paths defined in the Paths class
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

    private int cycleCounter = 0;
    private static final int MAX_CYCLES = 5;

    private final ElapsedTime emergencyTimer = new ElapsedTime();
    private boolean timeRanOut = false;

    private final ElapsedTime stabilizationTimer = new ElapsedTime();
    private final ElapsedTime minimumShootTimer = new ElapsedTime();

    private final Pose[] LEAVE_POINTS = {
        // inside close shoot zone
        new Pose(144 - 40.000, -130.000, Math.toRadians(0)),
        new Pose(144 - 59.000, -105.000, Math.toRadians(0)),
        new Pose(144 - 60.000, -130.000, Math.toRadians(0)),

        // mid field
        new Pose(144 - 20.000, -95.000, Math.toRadians(0)),
        new Pose(144 - 20.000, -70.000, Math.toRadians(0)),
        new Pose(144 - 35.000, -75.000, Math.toRadians(0)),

        // far field
        new Pose(144 - 48.000, -72.000, Math.toRadians(0)),
        new Pose(144 - 25.000, -45.000, Math.toRadians(0)),
        new Pose(144 - 53.000, -35.000, Math.toRadians(0)),
        new Pose(144 - 35.000, -15.000, Math.toRadians(0)),
        new Pose(144 - 52.000, -22.000, Math.toRadians(0)),
    };

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        robot.shooter.init();

        Robot.alliance = Robot.Alliance.RED;

        // Determine starting heading: prefer geometric heading when a path exists, otherwise fall
        // back to explicit startPoint values

        pathTimer = new ElapsedTime();
        paths = new Paths(Robot.follower); // Build paths

        Robot.follower.setStartingPose(paths.startPose);
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

        //        autoPark();
        // Log values to Panels and Driver Station
        updateTelemetry();
    }

    @Override
    public void stop() {
        Robot.transitionPose = Robot.follower.getPose();
        robot.shooter.togglePurpleObelisk();
    }

    public static class Paths {

        public final Pose startPose = new Pose(84, -115, -Math.PI / 2 - 0.601);
        public final Pose shootPose = new Pose(86.575572, -82.335349, -Math.toRadians(36));

        public final Pose grabFirstPose = new Pose(102.774101, -82.335349, 0.012239);
        public final Pose grabFirstEndPose = new Pose(127.774101, -82.335349, 0.012239);

        public final Pose grabSecondPose = new Pose(102.773753, -62.020596, 0.038969);
        public final Pose grabSecondEndPose = new Pose(127.773753, -62.020596, 0.038969);

        public final Pose grabThirdPose = new Pose(102.496711, -34.523277, 0.003746);
        public final Pose grabThirdEndPose = new Pose(132.496711, -34.523277, 0.003746);

        public final Pose grabHumanPose = new Pose(137.069169, -136.480140, -1.542359);
        public final Pose shootPose2 = new Pose(100.189295, -96.559203, -Math.PI / 2);
        public final Pose leavePose = new Pose(115.189295, -96.559203, -Math.PI / 2);

        public PathChain shootPreload;
        public PathChain grabFirst;
        public PathChain shootFirst;
        public PathChain grabSecond;
        public PathChain shootSecond;
        public PathChain grabThird;
        public PathChain shootThird;
        public PathChain leave;

        public Paths(Follower follower) {
            shootPreload =
                    follower.pathBuilder()
                            .addPath(new BezierLine(startPose, shootPose))
                            .setLinearHeadingInterpolation(
                                    startPose.getHeading(), shootPose.getHeading())
                            .build();

            grabFirst =
                    follower.pathBuilder()
                            .addPath(new BezierLine(shootPose, grabFirstPose))
                            .setLinearHeadingInterpolation(
                                    shootPose.getHeading(), grabFirstPose.getHeading())
                            .addPath(new BezierLine(grabFirstPose, grabFirstEndPose))
                            .setLinearHeadingInterpolation(
                                    grabFirstPose.getHeading(), grabFirstEndPose.getHeading())
                            .build();

            shootFirst =
                    follower.pathBuilder()
                            .addPath(new BezierLine(grabFirstEndPose, shootPose))
                            .setLinearHeadingInterpolation(
                                    grabFirstEndPose.getHeading(), shootPose.getHeading())
                            .build();

            grabSecond =
                    follower.pathBuilder()
                            .addPath(new BezierLine(shootPose, grabSecondPose))
                            .setLinearHeadingInterpolation(
                                    shootPose.getHeading(), grabSecondPose.getHeading())
                            .addPath(new BezierLine(grabSecondPose, grabSecondEndPose))
                            .setLinearHeadingInterpolation(
                                    grabSecondPose.getHeading(), grabSecondEndPose.getHeading())
                            .build();

            shootSecond =
                    follower.pathBuilder()
                            .addPath(new BezierLine(grabSecondEndPose, shootPose2))
                            .setLinearHeadingInterpolation(
                                    grabSecondEndPose.getHeading(), shootPose2.getHeading())
                            .build();

            grabThird =
                    follower.pathBuilder()
                            .addPath(new BezierLine(shootPose2, grabThirdPose))
                            .setLinearHeadingInterpolation(
                                    shootPose2.getHeading(), grabThirdPose.getHeading())
                            .addPath(new BezierLine(grabThirdPose, grabThirdEndPose))
                            .setLinearHeadingInterpolation(
                                    grabThirdPose.getHeading(), grabThirdEndPose.getHeading())
                            .build();

            shootThird =
                    follower.pathBuilder()
                            .addPath(new BezierLine(grabThirdEndPose, shootPose2))
                            .setLinearHeadingInterpolation(
                                    grabThirdEndPose.getHeading(), shootPose.getHeading())
                            .build();

            leave =
                    follower.pathBuilder()
                            .addPath(new BezierLine(shootPose2, leavePose))
                            .setLinearHeadingInterpolation(
                                    shootPose2.getHeading(), leavePose.getHeading())
                            .build();
        }
    }

    private PathChain generateShootPath() {
        //        Pose humanPose = new Pose[]{paths.grabHumanPose,
        // paths.grabHumanSecondPose}[cycleCounter % 2];

        return Robot.follower
                .pathBuilder()
                .addPath(new BezierLine(paths.grabHumanPose, paths.shootPose2))
                .setLinearHeadingInterpolation(
                        paths.grabHumanPose.getHeading(), paths.shootPose2.getHeading())
                .build();
    }

    private PathChain generateGrabHumanPath() {
        //        Pose humanPose = new Pose[]{paths.grabHumanPose,
        // paths.grabHumanSecondPose}[cycleCounter % 2];

        return Robot.follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                paths.shootPose2, new Pose(134.07, -100), paths.grabHumanPose))
                .setLinearHeadingInterpolation(
                        paths.shootPose2.getHeading(), paths.grabHumanPose.getHeading())
                .build();
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
                if (sinigermiguelphonk) {
                    stabilizationTimer.reset();
                    sinigermiguelphonk = false;
                }

                if (Robot.follower.isBusy()) {
                    robot.intake.rest();
                    stabilizationTimer.reset();
                    minimumShootTimer.reset();
                    break;
                }

                if (stabilizationTimer.milliseconds() < uV.STABILIZATION_TIMEOUT_MS * 2) {
                    robot.intake.rest();
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

                intakeTimeout.reset();
                intakeTimeoutReset = true;
                break;
            case PULLING:
                robot.intake.pullBalls();

                if (robot.intake.isFull()
                        || (intakeTimeout.milliseconds() > uV.INTAKE_TIMEOUT_MS
                                && intakeTimeoutReset)) {
                    intakeState = IntakeStates.COMPLETED;
                    intakeTimeoutReset = false;
                }
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
        if (emergencyTimer.milliseconds() >= uV.AUTOPARK_TIMEOUT_MS && !timeRanOut) {
            // DO NOT autopark if going for score (would win 9 points instead of 6)
            timeRanOut = true;

            if (!Robot.follower.isBusy()
                    && (shootState == ShootStates.SPOOL_UP || shootState == ShootStates.SHOOTING)) {
                return;
            }

            Robot.follower.breakFollowing();
            Robot.follower.followPath(generateEmergencyLeavePath());

            robot.intake.rest();
            robot.shooter.stop();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.shooter.togglePurpleObelisk();

                Robot.follower.followPath(paths.shootPreload, true);
                setPathState(1);

                break;

            // FIRST SPIKE

            case 1:
                autoShoot();
                if (shootState == ShootStates.COMPLETED) {
                    setPathState(2);
                }

                break;
            case 2:
                Robot.follower.followPath(paths.grabFirst, true);
                setPathState(3);
                break;
            case 3:
                autoIntake();

                if (intakeState == IntakeStates.COMPLETED) {
                    setPathState(4);
                }
                break;
            case 4:
                Robot.follower.followPath(paths.shootFirst, true);
                setPathState(5);
                break;
            case 5:
                autoShoot();
                if (shootState == ShootStates.COMPLETED) {
                    setPathState(6);
                }
                break;

            // SECOND SPIKE
            case 6:
                Robot.follower.followPath(paths.grabSecond, true);
                setPathState(7);
                break;
            case 7:
                autoIntake();

                if (intakeState == IntakeStates.COMPLETED) {
                    setPathState(8);
                }
                break;
            case 8:
                Robot.follower.followPath(paths.shootSecond, true);
                setPathState(9);
                break;
            case 9:
                if (pathTimer.milliseconds() > 950 && pathTimer.milliseconds() < 1010) {
                    robot.intake.reverse();
                } else if (pathTimer.milliseconds() > 1010 && pathTimer.milliseconds() < 1060) {
                    robot.intake.rest();
                }

                autoShoot();
                if (shootState == ShootStates.COMPLETED) {
                    setPathState(14);
                }
                break;

            // HUMAN
            case 14:
                if (++cycleCounter >= MAX_CYCLES) {
                    setPathState(17);
                    break;
                }

                Robot.follower.followPath(generateGrabHumanPath(), true);
                setPathState(15);
                break;

            case 15:
                autoIntake();

                if (intakeState == IntakeStates.COMPLETED) {
                    setPathState(16);
                }
                break;
            case 16:
                pathTimer.reset();
                Robot.follower.followPath(generateShootPath(), true);
                setPathState(9);
                break;

            case 17:
                Robot.follower.followPath(paths.leave, true);
                break;
            case 18:
                if (!Robot.follower.isBusy()) {
                    setPathState(200);
                }
                break;

            case 200:
                requestOpModeStop();
                pathState = -1;
                break;
        }
        return pathState;
    }

    public void setPathState(int pState) {
        sinigermiguelphonk = true;
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
        dashboardTelemetry.addData("Timer", emergencyTimer.seconds());
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
