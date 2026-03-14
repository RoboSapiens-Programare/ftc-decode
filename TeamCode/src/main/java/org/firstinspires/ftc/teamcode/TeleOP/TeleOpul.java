package org.firstinspires.ftc.teamcode.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TeleOp")
public class TeleOpul extends OpMode {

    private Robot robot;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    private final ElapsedTime stateTimer = new ElapsedTime();
    private int loopCount = 0;
    private boolean singleton = true;
    private final ElapsedTime followerIdleTimer = new ElapsedTime();
    private boolean singleton2 = true;
    private boolean isAimingChassis = false;

    private ElapsedTime inputTimer = new ElapsedTime();


    enum State {
        INTAKE,
        OUTTAKE
    }

    private State state = State.INTAKE;

    private void aimChassis(double targetHeadingRad) {
        Pose current = Robot.follower.getPose();

        // atan2 returns [-π, π], PedroPathing heading is [0, 2π] — must match
        while (targetHeadingRad < 0)            targetHeadingRad += 2.0 * Math.PI;
        while (targetHeadingRad >= 2.0 * Math.PI) targetHeadingRad -= 2.0 * Math.PI;

        double currentHeading = current.getHeading(); // already [0, 2π]

        // 1-inch offset so BezierLine is non-degenerate
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

    private void changeState(State newState) {
        state = newState;
        stateTimer.reset();
        singleton = false;
        singleton2 = false;

        robot.shooter.shooting = newState == State.OUTTAKE;
        if (newState == State.OUTTAKE) {
            // Reseteaza PID flywheel si ramp — previne integral windup
            robot.shooter.resetShooterPID();
        } else {
            robot.shooter.stopOverride();
            // Inchide gate la intrarea in INTAKE
            robot.shooter.closeGate();
        }
    }

    private void handleIntake() {
        robot.shooter.shooting = false;

        robot.shooter.closeGate();

        if (gamepad1.right_trigger > 0.1 && !(gamepad1.left_trigger > 0.1)) {
            robot.intake.pullBalls();
        } else if (gamepad1.left_trigger > 0.1 && !(gamepad1.right_trigger > 0.1)) {
            robot.intake.spitBalls();
        } else {
            robot.intake.rest();
        }

        if (gamepad1.cross && stateTimer.milliseconds() > 400) {
            isAimingChassis = true;
            changeState(State.OUTTAKE);
        }
    }

    private void handleOuttake() {
        robot.shooter.shooting = true;


        robot.shooter.openGate();


        if ((gamepad1.right_trigger > 0.1 && robot.shooter.velocityReached() && robot.shooter.isAimed()) || (gamepad2.right_trigger>0.1 && robot.shooter.velocityReached()))  {
            robot.intake.shoot();
        } else {
//            robot.shooter.shootingLobComp = false;
            robot.intake.rest();
        }


//        if (gamepad1.right_trigger > 0.1 && robot.shooter.velocityReached() && robot.shooter.isAimed() && robot.shooter.llDistance<40)
//        {
//            robot.intake.shoot();
//            robot.shooter.shootingLobComp = true;
//            robot.shooter.lobServo.setPosition(robot.shooter.lobServo.getPosition()+0.02);
//        } else if (gamepad1.right_trigger > 0.1 && robot.shooter.velocityReached() && robot.shooter.isAimed() && robot.shooter.llDistance>40 && robot.shooter.llDistance<80)
//        {
//            robot.intake.shoot();
//            robot.shooter.shootingLobComp = true;
//            robot.shooter.lobServo.setPosition(robot.shooter.lobServo.getPosition()+0.01);
//        }else if (gamepad1.right_trigger > 0.1 && robot.shooter.velocityReached() && robot.shooter.isAimed() && robot.shooter.llDistance>80)
//        {
//            robot.intake.shoot();
//        }else {
//            robot.shooter.shootingLobComp = false;
//            robot.intake.rest();
//        }

        if (gamepad1.cross && stateTimer.milliseconds() > 400) {
            isAimingChassis = false;
            Robot.follower.breakFollowing();
            Robot.follower.startTeleOpDrive(true);
            changeState(State.INTAKE);

        }
    }

//    private void resetFollowerPose(Pose p) {
//        Robot.follower = Constants.createFollower(hardwareMap);
//        Robot.follower.setStartingPose(p);
//        Robot.follower.startTeleOpDrive(true);
//    }

    @Override
    public void stop() {
        Robot.transitionPose = Robot.follower.getPose();
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        state = State.INTAKE;

//        Robot.follower = Constants.createFollower(hardwareMap);



        gamepad1.setLedColor(0, 255, 0, 10);

        robot.shooter.openGate();
    }

    @Override
    public void start() {
        Robot.follower.setStartingPose(Robot.transitionPose);
        Robot.follower.startTeleOpDrive(true);
    }

    public void init_loop()
    {
        Pose startPoseBlue = new Pose(56, 8, Math.toRadians(90));
        Pose startPoseRed = new Pose(88, 8, Math.toRadians(90));
        if (gamepad1.options)
        {
            Robot.alliance = Robot.Alliance.RED;
            Robot.transitionPose = startPoseRed;
            gamepad1.setLedColor(255, 0,0,10000);
        } else if (gamepad1.share) {
            Robot.alliance = Robot.Alliance.BLUE;
            Robot.transitionPose =startPoseBlue;
            gamepad1.setLedColor(0, 0,255,10000);
        }

//        telemetry.addData("Alliance", Robot.alliance);
//        telemetry.update();
    }

    @Override
    public void loop() {
        switch (state) {
            case INTAKE:
                handleIntake();
                break;

            case OUTTAKE:
                handleOuttake();
                break;
        }

        robot.intake.updateHeadlight();

        Pose homingPoseBlue = new Pose(20.9, 123.1, Math.toRadians(144));
        Pose homingPoseRed = new Pose(123.077, 123.133, Math.toRadians(36));
        if (gamepad1.circle)
        {
            Robot.alliance = Robot.Alliance.RED;
            Robot.transitionPose = homingPoseRed;

//            resetFollowerPose(homingPoseRed);
            Robot.follower.setPose(Robot.transitionPose);
            robot.shooter.reset();

            gamepad1.setLedColor(255, 0,0,10000);
        } else if (gamepad1.square) {
            Robot.alliance = Robot.Alliance.BLUE;
            Robot.transitionPose = homingPoseBlue;

//            resetFollowerPose(homingPoseBlue);
            Robot.follower.setPose(Robot.transitionPose);
            robot.shooter.reset();

            gamepad1.setLedColor(0, 0,255,10000);
        }

        if (gamepad1.right_bumper && inputTimer.milliseconds() > 400) {
            robot.shooter.lock();

            inputTimer.reset();
        }

        if (gamepad1.left_bumper && inputTimer.milliseconds() > 400) {
            robot.shooter.stopOverride();

            inputTimer.reset();
        }

        if (isAimingChassis && !singleton) {
            Robot.follower.breakFollowing();
            aimChassis(robot.shooter.getTargetFieldAngleRadStatic());
            singleton = true;
        }

        boolean driverMovingSticks = Math.abs(gamepad1.left_stick_x) > 0.1
                || Math.abs(gamepad1.left_stick_y) > 0.1
                || Math.abs(gamepad1.right_stick_x) > 0.1;

        boolean followerBusy = Robot.follower.isBusy();
        if (followerBusy) {
            followerIdleTimer.reset();
        }
        boolean followerSettled = !followerBusy && followerIdleTimer.milliseconds() > 300;

        if (isAimingChassis && singleton && !singleton2 && driverMovingSticks) {
            Robot.follower.breakFollowing();
            Robot.follower.startTeleOpDrive(true);
            singleton2 = true;
        }

//        robot.shooter.turretLocked = !followerSettled;

        if (gamepad2.dpad_left && inputTimer.milliseconds() > 200) {
            robot.shooter.incremental(4 * Math.PI / 90);
            inputTimer.reset();
        }
        if (gamepad2.dpad_right && inputTimer.milliseconds() > 200) {
            robot.shooter.incremental(- 4 * Math.PI / 90);
            inputTimer.reset();
        }

        if (gamepad2.dpad_up && inputTimer.milliseconds() > 200) {
            robot.shooter.incremental(Math.PI / 90);
            inputTimer.reset();
        }
        if (gamepad2.dpad_down && inputTimer.milliseconds() > 200) {
            robot.shooter.incremental(- Math.PI / 90);
            inputTimer.reset();
        }
        if (gamepad2.touchpad && inputTimer.milliseconds() > 400) {
            robot.shooter.stopOverride();
            inputTimer.reset();
        }

        robot.shooter.update();

        boolean allowDrive = !isAimingChassis
                || singleton2
                || followerSettled;

        if (allowDrive) {
            Robot.follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true);
        }

        Robot.follower.update();

//        loopCount++;
//        if (loopCount % 5 == 0) {
//            dashboardTelemetry.addData("desired angle", robot.shooter.getTargetFieldAngleRadStatic());
//            dashboardTelemetry.addData("Sensor1", robot.intake.sensorIntake.getDistance(DistanceUnit.CM));
//            dashboardTelemetry.addData("Sensor2", robot.intake.sensorMid.getDistance(DistanceUnit.CM));
//            dashboardTelemetry.addData("Sensor3", robot.intake.sensorOuttake.getDistance(DistanceUnit.CM));
//            dashboardTelemetry.addData("State", state);dashboardTelemetry.addData("State", state);
//            dashboardTelemetry.addData("Follower busy", Robot.follower.isBusy());
//            dashboardTelemetry.addData("Distance (in)", robot.shooter.llDistance);
//            dashboardTelemetry.addData("Flywheel RPM", -robot.shooter.turretMotorLeft.getVelocity());
//            dashboardTelemetry.addData("Track State", robot.shooter.trackState);
//            dashboardTelemetry.addData("Turret Output", robot.shooter.turretOutput);
//            dashboardTelemetry.addData("Turret Error", Math.toDegrees(robot.shooter.turretErrorRad));
//            dashboardTelemetry.addData("Target RPM", robot.shooter.targetVelocity);
//            dashboardTelemetry.addData("Actual RPM", -robot.shooter.turretMotorLeft.getVelocity());
//            dashboardTelemetry.addData("Distance (in)", robot.shooter.llDistance);
//            dashboardTelemetry.addData("Velocity OK", robot.shooter.velocityReached());
//            dashboardTelemetry.addData("Aimed", robot.shooter.isAimed());
//            dashboardTelemetry.addData("Distance", robot.shooter.llDistance);
//            dashboardTelemetry.addData("encoder pos", robot.shooter.turretEncoder.getCurrentPosition());
//            dashboardTelemetry.addData("Angle pose", robot.follower.getPose().getHeading());
//            dashboardTelemetry.addData("0. POSE", Robot.follower.getPose());
//            dashboardTelemetry.update();
//        }


    }
}