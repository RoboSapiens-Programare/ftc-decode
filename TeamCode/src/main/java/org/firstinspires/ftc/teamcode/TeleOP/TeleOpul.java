package org.firstinspires.ftc.teamcode.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
// import com.seattlesolvers.solverslib.photon.PhotonCore;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Utils.NNLogging;
import org.firstinspires.ftc.teamcode.Robot.Utils.ShootAssist;
import org.firstinspires.ftc.teamcode.Robot.uV;

@TeleOp(name = "TeleOp")
@Config
public class TeleOpul extends OpMode {

    long lastTime = System.nanoTime();

    private static final long INPUT_COOLDOWN_LONG_MS = 400;
    private static final long FOLLOWER_SETTLE_MS = 300;
    private static final double TRIGGER_THRESHOLD = 0.1;
    private static final double STICK_THRESHOLD = 0.1;
    private boolean lastD1Trigger = false;

    private Robot robot;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime followerIdleTimer = new ElapsedTime();
    private final ElapsedTime inputTimer = new ElapsedTime();
    private double averagedFrequency = 50.0; // Seed it with an expected baseline (e.g., 50-60Hz)

    private int loopCount = 0;
    private boolean overrideCancelled = true;

    private final ElapsedTime rumbleTimer = new ElapsedTime();

    private enum State {
        INTAKE,
        OUTTAKE
    }

    private State state = State.INTAKE;

    private final NNLogging logger = new NNLogging();
    private final ShootAssist shootAssist = new ShootAssist();
    private boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        state = State.INTAKE;
        gamepad1.setLedColor(0, 255, 0, 10);
        robot.shooter.openGate();

        robot.shooter.init();
        //        robot.shooter.shootingLobComp = false;
        if (uV.NN_LOGGING_ENABLE) logger.init(hardwareMap);

        if (uV.USE_NN_AIM_ASSIST) {
            shootAssist.init("shoot_predictor.tflite");
            shootAssist.debugModelLoading();
        }
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
        robot.update();

        switch (state) {
            case INTAKE:
                handleIntake();
                break;
            case OUTTAKE:
                handleOuttake();
                break;
        }

        handlePoseReset();
        handleOverrideButtons();
        handleDrive();
        handleVelocityChange();

        updateTelemetry();

        //        LAST RESORT!
        //        PhotonCore.CONTROL_HUB.clearBulkCache();
        //        PhotonCore.EXPANSION_HUB.clearBulkCache();
    }

    @Override
    public void stop() {
        Robot.transitionPose = Robot.follower.getPose();

        if (uV.NN_LOGGING_ENABLE) {
            logger.close();
        }
    }

    // State Transitions
    private void changeState(State newState) {
        state = newState;
        stateTimer.reset();
        overrideCancelled = false;

        robot.intake.rest();

        robot.shooter.shooting = newState == State.OUTTAKE;
        if (newState == State.OUTTAKE) {
            robot.shooter.resetShooterPID();
            robot.shooter.openGate();
            gamepad2.setLedColor(0xff, 0x00, 0x00, 0);

        } else {

            robot.shooter.stopOverride();
            robot.shooter.closeGate();

            gamepad2.setLedColor(0x00
                    , 0xff, 0x00, 0);
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

        if (gamepad2.cross && stateTimer.milliseconds() > INPUT_COOLDOWN_LONG_MS) {
            lastD1Trigger = gamepad1.right_trigger > TRIGGER_THRESHOLD;
            changeState(State.OUTTAKE);
        }
    }

    private void handleOuttake() {
        if (gamepad2.cross && stateTimer.milliseconds() > INPUT_COOLDOWN_LONG_MS) {
            robot.intake.rest();
            Robot.follower.breakFollowing();
            Robot.follower.startTeleOpDrive(true);
            changeState(State.INTAKE);
        }

        if (lastD1Trigger) {
            lastD1Trigger = gamepad1.right_trigger > TRIGGER_THRESHOLD;
            return;
        }
        boolean fireMain = gamepad1.right_trigger > TRIGGER_THRESHOLD && robot.shooter.velocityReached() && robot.shooter.isAimed();

        if (fireMain) {
            robot.intake.shoot();
        } else {
            robot.intake.rest();
        }

        if (uV.NN_LOGGING_ENABLE) {
            handleNNLogging();
        }

        if (uV.USE_NN_AIM_ASSIST && rumbleTimer.milliseconds() > 300) {
            rumbleTimer.reset();

            double robotX = Robot.follower.getPose().getX();
            double robotY = Robot.follower.getPose().getY();
            double heading = Robot.follower.getHeading();

            // 2. Replicate the Logger's Math to get true spatial targets
            double deltaX = Shooter.targetGoal.getX() - robotX; // Matches your GOAL_X
            double deltaY = Shooter.targetGoal.getY() - robotY; // Matches your GOAL_Y
            double targetDist = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

            double absoluteAngleToGoal = Math.atan2(deltaY, deltaX);
            double angleError = absoluteAngleToGoal - heading;

            // Normalize angle error to [-pi, pi] to perfectly match the training scaling
            while (angleError > Math.PI) angleError -= 2 * Math.PI;
            while (angleError < -Math.PI) angleError += 2 * Math.PI;

            // 3. Extract ROBOT-CENTRIC local velocities instead of global field ones
            double globalVx = Robot.follower.getVelocity().getXComponent();
            double globalVy = Robot.follower.getVelocity().getYComponent();

            double localVx = globalVx * Math.cos(-heading) - globalVy * Math.sin(-heading);
            double localVy = globalVx * Math.sin(-heading) + globalVy * Math.cos(-heading);

            double omega = Robot.follower.getAngularVelocity();
            double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

            // 4. Feed the perfectly aligned features to the interpreter
            int predictedBalls =
                    shootAssist.predictBallCount(
                            targetDist, angleError, localVx, localVy, omega, voltage);

            FtcDashboard.getInstance()
                    .getTelemetry()
                    .addData("NN Target Prediction", predictedBalls);

            // --- ENHANCED HAPTIC RUMBLE PATTERNS ---
            if (predictedBalls == 0) {
                gamepad1.rumbleBlips(2);
                gamepad2.rumbleBlips(2);
            }
        }
    }

    private void handleVelocityChange() {
        if (uV.USE_VELOCITY_REGRESSION) {
            return;
        }

        if (gamepad1.dpad_up && inputTimer.milliseconds() > 200) {
            Shooter.targetVelocity += 50;
            inputTimer.reset();
        }

        if (gamepad1.dpad_down && inputTimer.milliseconds() > 200) {
            Shooter.targetVelocity -= 50;
            inputTimer.reset();
        }
    }

    private void handlePoseReset() {
        Pose homingPose = new Pose(72, 134, Math.toRadians(90));

        if (gamepad2.touchpad) {
            Robot.follower.breakFollowing();
            Robot.follower.setPose(homingPose);
            Robot.follower.startTeleOpDrive(true);
            robot.shooter.reset();
            gamepad2.setLedColor(0xff, 0xff, 0x00, 100);
            gamepad1.rumbleBlips(3);
            gamepad2.rumbleBlips(3);
            robot.shooter.trackOffset = 0;
            robot.shooter.velocityOffset = 0;
        }
    }

    private void handleOverrideButtons() {
        if (gamepad1.right_bumper && inputTimer.milliseconds() > INPUT_COOLDOWN_LONG_MS) {
            robot.shooter.goToAngle(0);
            inputTimer.reset();
        }

        if (gamepad1.left_bumper && inputTimer.milliseconds() > INPUT_COOLDOWN_LONG_MS) {
            robot.shooter.stopOverride();
            inputTimer.reset();
        }

        if (gamepad2.dpad_up && inputTimer.milliseconds() > INPUT_COOLDOWN_LONG_MS) {
            robot.shooter.velocityOffset += 10;
            inputTimer.reset();
        }

        if (gamepad2.dpad_down && inputTimer.milliseconds() > INPUT_COOLDOWN_LONG_MS) {
            robot.shooter.velocityOffset -= 10;
            inputTimer.reset();
        }

        if (gamepad2.dpad_left && inputTimer.milliseconds() > INPUT_COOLDOWN_LONG_MS) {
            robot.shooter.trackOffset += 0.02;
            inputTimer.reset();
        }

        if (gamepad2.dpad_right && inputTimer.milliseconds() > INPUT_COOLDOWN_LONG_MS) {
            robot.shooter.trackOffset -= 0.02;
            inputTimer.reset();
        }


    }

    private void handleDrive() {
        boolean allowDrive =
                overrideCancelled
                        || (!Robot.follower.isBusy()
                                && followerIdleTimer.milliseconds() > FOLLOWER_SETTLE_MS);

        if (allowDrive) {
            Robot.follower.setTeleOpDrive(
                    -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        }
    }

    // Define a boolean tracker for your trigger button state in your class variables
    private double lastRightBumper = 0;

    private void handleNNLogging() {
        // 1. Drivetrain state capture loop
        Pose p = Robot.follower.getPose();
        Vector velocity = Robot.follower.getVelocity();

        // 2. AUTOMATIC CAPTURE: Take a snapshot the split-second you shoot
        // Replace 'gamepad1.right_bumper' with whatever trigger you use to activate your outtake
        if (gamepad1.right_trigger > .5 && lastRightBumper < .5) {
            logger.takeSnapshot(
                    p.getX(),
                    p.getY(),
                    p.getHeading(),
                    velocity.getXComponent(),
                    velocity.getYComponent(),
                    Robot.follower.getAngularVelocity());
        }
        lastRightBumper = gamepad1.right_trigger;

        // 3. MANUAL INPUT: Commit the snapshot only if one is waiting in the chamber
        if (logger.hasPendingSnapshot()) {
            if (gamepad1.dpad_up && !lastUp) {
                logger.commitSnapshot(3);
                gamepad1.rumbleBlips(3);
                gamepad2.rumbleBlips(3);
            }
            if (gamepad1.dpad_right && !lastRight) {
                logger.commitSnapshot(2);
                gamepad1.rumbleBlips(2);
                gamepad2.rumbleBlips(2);
            }

            if (gamepad1.dpad_left && !lastLeft) {
                logger.commitSnapshot(0);
                gamepad1.rumbleBlips(0);
                gamepad2.rumbleBlips(0);
            }
            if (gamepad1.dpad_down && !lastDown) {
                logger.commitSnapshot(1);
                gamepad1.rumbleBlips(1);
                gamepad2.rumbleBlips(1);
            }
        }

        // Standard edge detection state tracking
        lastUp = gamepad1.dpad_up;
        lastRight = gamepad1.dpad_right;
        lastLeft = gamepad1.dpad_left;
        lastDown = gamepad1.dpad_down;
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
        dashboardTelemetry.addData("State", state);
        dashboardTelemetry.addData("Distance (in)", robot.shooter.distance);
        dashboardTelemetry.addData("Flywheel RPM", -robot.shooter.turretMotorLeft.getVelocity());
        dashboardTelemetry.addData("Target RPM", Shooter.targetVelocity);
        dashboardTelemetry.addData("Velocity OK", robot.shooter.velocityReached());
        dashboardTelemetry.addData("Aimed", robot.shooter.isAimed());
        dashboardTelemetry.addData("X", Robot.follower.getPose().getX());
        dashboardTelemetry.addData("Y", Robot.follower.getPose().getY());
        dashboardTelemetry.addData("Heading", Robot.follower.getPose().getHeading());

        dashboardTelemetry.addData("Total Balls Scored Captured", logger.getTotalBallsScored());
        dashboardTelemetry.update();
    }
}
