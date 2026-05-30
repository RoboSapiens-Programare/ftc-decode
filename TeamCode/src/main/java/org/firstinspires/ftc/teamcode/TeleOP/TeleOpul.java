package org.firstinspires.ftc.teamcode.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.seattlesolvers.solverslib.photon.PhotonCore;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

@TeleOp(name = "TeleOp")
@Config
public class TeleOpul extends OpMode {

    long lastTime = System.nanoTime();

    public static double pos = 0.0;
    public static double velo = 0.0;
    private static final long INPUT_COOLDOWN_MS = 200;
    private static final long INPUT_COOLDOWN_LONG_MS = 400;
    private static final long FOLLOWER_SETTLE_MS = 300;
    private static final double TRIGGER_THRESHOLD = 0.1;
    private static final double STICK_THRESHOLD = 0.1;

    private Robot robot;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime followerIdleTimer = new ElapsedTime();
    private final ElapsedTime inputTimer = new ElapsedTime();
    private double averagedFrequency = 50.0; // Seed it with an expected baseline (e.g., 50-60Hz)

    private int loopCount = 0;
    private boolean aimOnce = true;
    private boolean overrideCancelled = true;
    private boolean isAimingChassis = false;

    private enum State {
        INTAKE,
        OUTTAKE
    }

    private State state = State.INTAKE;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        state = State.INTAKE;
        gamepad1.setLedColor(0, 255, 0, 10);
        robot.shooter.openGate();

        robot.shooter.init();
//        robot.shooter.shootingLobComp = false;
    }

    @Override
    public void init_loop() {
        Pose startPoseBlue = new Pose(54, 8, Math.toRadians(90));
        Pose startPoseRed = new Pose(88, 8, Math.toRadians(90));
        if (gamepad1.options) {
            Robot.alliance = Robot.Alliance.RED;
            Robot.transitionPose = startPoseRed;
            gamepad1.setLedColor(255, 0, 0, 10000);
        } else if (gamepad1.share) {
            Robot.alliance = Robot.Alliance.BLUE;
            Robot.transitionPose = startPoseBlue;
            gamepad1.setLedColor(0, 0, 255, 10000);
        }
    }

    @Override
    public void start() {
        Robot.follower.setStartingPose(Robot.transitionPose);
        Robot.follower.startTeleOpDrive(true);
    }

    @Override
    public void loop() {
        robot.resetCache();

        switch (state) {
            case INTAKE:
                handleIntake();
                break;
            case OUTTAKE:
                handleOuttake();
                break;
        }

        robot.intake.update();
        handlePoseReset();
        handleOverrideButtons();
        handleChassisAiming();
        handleDriverOverride();
//        handleTurretControls();
        handleDrive();

        robot.shooter.update();
//        robot.shooter.lobServo.setPosition(pos);
        Robot.follower.update();


        updateTelemetry();

//        LAST RESORT!
//        PhotonCore.CONTROL_HUB.clearBulkCache();
//        PhotonCore.EXPANSION_HUB.clearBulkCache();

    }

    @Override
    public void stop() {
        Robot.transitionPose = Robot.follower.getPose();
    }

    // State Transitions
    private void changeState(State newState) {
        state = newState;
        stateTimer.reset();
        aimOnce = false;
        overrideCancelled = false;

        robot.shooter.shooting = newState == State.OUTTAKE;
        if (newState == State.OUTTAKE) {
            robot.shooter.resetShooterPID();
            robot.shooter.openGate();

        } else {

            robot.shooter.stopOverride();
            robot.shooter.closeGate();
        }
    }

    // State Handlers
    private void handleIntake() {
        robot.shooter.shooting = false;
        robot.shooter.closeGate();

        if (gamepad1.right_trigger > TRIGGER_THRESHOLD
                && !(gamepad1.left_trigger > TRIGGER_THRESHOLD)) {
            robot.intake.pullBalls();
        } else if (gamepad1.left_trigger > TRIGGER_THRESHOLD
                && !(gamepad1.right_trigger > TRIGGER_THRESHOLD)) {
            robot.intake.spitBalls();
        } else {
            robot.intake.rest();
        }

        if (gamepad1.cross && stateTimer.milliseconds() > INPUT_COOLDOWN_LONG_MS) {
//            isAimingChassis = true;
            changeState(State.OUTTAKE);
        }
    }

    private void handleOuttake() {
        boolean fireMain = gamepad1.right_trigger > TRIGGER_THRESHOLD
                && robot.shooter.velocityReached()
                && robot.shooter.isAimed();
        boolean fireSecondary = gamepad2.right_trigger > TRIGGER_THRESHOLD
                && robot.shooter.velocityReached();

        if (fireMain || fireSecondary) {
            robot.intake.shoot();
        } else {
            robot.intake.rest();
        }

        if (gamepad1.cross && stateTimer.milliseconds() > INPUT_COOLDOWN_LONG_MS) {
            isAimingChassis = false;
            Robot.follower.breakFollowing();
            Robot.follower.startTeleOpDrive(true);
            changeState(State.INTAKE);
        }
    }

    // Chassis
    private void aimChassis(double targetHeadingRad) {
        Pose current = Robot.follower.getPose();

        while (targetHeadingRad < 0) targetHeadingRad += 2.0 * Math.PI;
        while (targetHeadingRad >= 2.0 * Math.PI) targetHeadingRad -= 2.0 * Math.PI;

        double currentHeading = current.getHeading();

        Pose target = new Pose(
                current.getX() + Math.cos(targetHeadingRad),
                current.getY() + Math.sin(targetHeadingRad),
                targetHeadingRad
        );

        PathChain path = Robot.follower.pathBuilder()
                .addPath(new BezierLine(current, target))
                .setLinearHeadingInterpolation(currentHeading, targetHeadingRad)
                .build();
        Robot.follower.followPath(path, true);
    }

    private void handlePoseReset() {
        Pose homingPoseRed = new Pose(123.077, 123.133, Math.toRadians(36));
        Pose homingPoseBlue = new Pose(20.9, 123.1, Math.toRadians(144));

        if (gamepad1.circle) {
            Robot.alliance = Robot.Alliance.RED;
            Robot.transitionPose = homingPoseRed;
            Robot.follower.setPose(Robot.transitionPose);
            robot.shooter.reset();
            gamepad1.setLedColor(255, 0, 0, 10000);
        } else if (gamepad1.square) {
            Robot.alliance = Robot.Alliance.BLUE;
            Robot.transitionPose = homingPoseBlue;
            Robot.follower.setPose(Robot.transitionPose);
            robot.shooter.reset();
            gamepad1.setLedColor(0, 0, 255, 10000);
        }
    }

    private void handleOverrideButtons() {
        if (gamepad1.right_bumper && inputTimer.milliseconds() > INPUT_COOLDOWN_LONG_MS) {
            robot.shooter.lock();
            inputTimer.reset();
        }

        if (gamepad1.left_bumper && inputTimer.milliseconds() > INPUT_COOLDOWN_LONG_MS) {
            robot.shooter.stopOverride();
            inputTimer.reset();
        }
    }

    private void handleChassisAiming() {
        if (isAimingChassis && !aimOnce) {
            Robot.follower.breakFollowing();
            aimChassis(robot.shooter.getTargetFieldAngleRadStatic());
            aimOnce = true;
        }
    }

    private void handleDriverOverride() {
        boolean driverMovingSticks = Math.abs(gamepad1.left_stick_x) > STICK_THRESHOLD
                || Math.abs(gamepad1.left_stick_y) > STICK_THRESHOLD
                || Math.abs(gamepad1.right_stick_x) > STICK_THRESHOLD;

        boolean followerBusy = Robot.follower.isBusy();
        if (followerBusy) {
            followerIdleTimer.reset();
        }
//        boolean followerSettled = !followerBusy && followerIdleTimer.milliseconds() > FOLLOWER_SETTLE_MS;

        if (isAimingChassis && aimOnce && !overrideCancelled && driverMovingSticks) {
            Robot.follower.breakFollowing();
            Robot.follower.startTeleOpDrive(true);
            overrideCancelled = true;
        }
    }

//    private void handleTurretControls() {
//        if (gamepad2.dpad_left && inputTimer.milliseconds() > INPUT_COOLDOWN_MS) {
//            robot.shooter.incremental(4 * Math.PI / 90);
//            inputTimer.reset();
//        }
//        if (gamepad2.dpad_right && inputTimer.milliseconds() > INPUT_COOLDOWN_MS) {
//            robot.shooter.incremental(-4 * Math.PI / 90);
//            inputTimer.reset();
//        }
//        if (gamepad2.dpad_up && inputTimer.milliseconds() > INPUT_COOLDOWN_MS) {
//            robot.shooter.incremental(Math.PI / 90);
//            inputTimer.reset();
//        }
//        if (gamepad2.dpad_down && inputTimer.milliseconds() > INPUT_COOLDOWN_MS) {
//            robot.shooter.incremental(-Math.PI / 90);
//            inputTimer.reset();
//        }
//        if (gamepad2.touchpad && inputTimer.milliseconds() > INPUT_COOLDOWN_LONG_MS) {
//            robot.shooter.stopOverride();
//            inputTimer.reset();
//        }
//    }

    private void handleDrive() {
        boolean allowDrive = !isAimingChassis || overrideCancelled
                || (!Robot.follower.isBusy()
                && followerIdleTimer.milliseconds() > FOLLOWER_SETTLE_MS);

        if (allowDrive) {
            Robot.follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true);
        }
    }

    // Telemetry
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
        dashboardTelemetry.addData("State", state);
//        dashboardTelemetry.addData("Follower busy", Robot.follower.isBusy());
        dashboardTelemetry.addData("Distance (in)", robot.shooter.distance);
        dashboardTelemetry.addData("Flywheel RPM", -robot.shooter.turretMotorLeft.getVelocity());
//        dashboardTelemetry.addData("Track State", robot.shooter.trackState);
//        dashboardTelemetry.addData("Turret Output", robot.shooter.turretOutput);
//        dashboardTelemetry.addData("Turret Error", Math.toDegrees(robot.shooter.turretErrorRad));
        dashboardTelemetry.addData("Target RPM", Shooter.targetVelocity);
        dashboardTelemetry.addData("Velocity OK", robot.shooter.velocityReached());
        dashboardTelemetry.addData("Aimed", robot.shooter.isAimed());
//        dashboardTelemetry.addData("Angle pose", Robot.follower.getPose().getHeading());
//        dashboardTelemetry.addData("0. POSE", Robot.follower.getPose());

        dashboardTelemetry.update();
    }
}
