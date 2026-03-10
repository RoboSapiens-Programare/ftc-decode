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
    private boolean isAimingChassis = false;


    enum State {
        INTAKE,
        OUTTAKE
    }

    private State state = State.INTAKE;

    private void aimChassis(Pose target) {
        Pose current = Robot.follower.getPose();
        PathChain path = Robot.follower.pathBuilder()
                .addPath(new BezierLine(current, target))
                .setLinearHeadingInterpolation(current.getHeading(), target.getHeading())
                .build();
        Robot.follower.followPath(path, true);
    }

    private void changeState(State newState) {
        state = newState;
        stateTimer.reset();


        robot.shooter.shooting = newState == State.OUTTAKE;
        if (newState == State.OUTTAKE) {
            // Reseteaza PID flywheel si ramp — previne integral windup
            robot.shooter.resetShooterPID();
        } else {
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


        if (gamepad1.right_trigger > 0.1 && robot.shooter.velocityReached() && robot.shooter.isAimed())
        {
            robot.intake.shoot();
        } else {
            robot.intake.rest();
        }

        if (gamepad1.cross && stateTimer.milliseconds() > 400) {
            changeState(State.INTAKE);

        }
    }

    @Override
    public void stop()
    {
        Robot.transitionPose = Robot.follower.getPose();
    }

    @Override
    public void init() {


        robot = new Robot(hardwareMap);

        state = State.INTAKE;

        Robot.follower = Constants.createFollower(hardwareMap);

        Robot.follower.startTeleOpDrive(true);

        gamepad1.setLedColor(0, 255, 0, 10);

        robot.shooter.openGate();
    }

    @Override
    public void start()
    {
        Robot.follower.setPose(Robot.transitionPose);
    }

    public void init_loop()
    {
        Pose startPoseBlue = new Pose(56, 8, Math.toRadians(90));
        Pose startPoseRed = new Pose(88, 8, Math.toRadians(90));
        if (gamepad1.options)
        {
            Robot.alliance = Robot.Alliance.RED;
            Robot.transitionPose =startPoseRed;
            Robot.follower.setPose(startPoseRed);
            gamepad1.setLedColor(255, 0,0,10000);
        } else if (gamepad1.share) {
            Robot.alliance = Robot.Alliance.BLUE;
            Robot.transitionPose =startPoseBlue;
            Robot.follower.setPose(startPoseBlue);
            gamepad1.setLedColor(0, 0,255,10000);
        }
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

        if (isAimingChassis && !((Math.abs(gamepad1.left_stick_x)>0.1)||(Math.abs(gamepad1.left_stick_y)>0.1)||(Math.abs(gamepad1.right_stick_x)>0.1)))
        {
            aimChassis(new Pose(Robot.follower.getPose().getX(), Robot.follower.getPose().getY(), robot.shooter.getTargetFieldAngleRad()));
        }

        Robot.follower.update();

        robot.shooter.turretLocked = Robot.follower.isBusy();

        robot.shooter.update();

        Robot.follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x - 0.1 * gamepad2.right_stick_x,
                true);

        loopCount++;
        if (loopCount % 5 == 0) {
            dashboardTelemetry.addData("Sensor1", robot.intake.sensorIntake.getDistance(DistanceUnit.CM));
            dashboardTelemetry.addData("Sensor2", robot.intake.sensorMid.getDistance(DistanceUnit.CM));
            dashboardTelemetry.addData("Sensor3", robot.intake.sensorOuttake.getDistance(DistanceUnit.CM));
            dashboardTelemetry.addData("State", state);
            dashboardTelemetry.addData("Distance (in)", robot.shooter.llDistance);
            dashboardTelemetry.addData("Flywheel RPM", -robot.shooter.turretMotorLeft.getVelocity());
            dashboardTelemetry.addData("Track State", robot.shooter.trackState);
            dashboardTelemetry.addData("Turret Output", robot.shooter.turretOutput);
            dashboardTelemetry.addData("Turret Error", Math.toDegrees(robot.shooter.turretErrorRad));
            dashboardTelemetry.addData("Target RPM", robot.shooter.targetVelocity);
            dashboardTelemetry.addData("Actual RPM", -robot.shooter.turretMotorLeft.getVelocity());
            dashboardTelemetry.addData("Distance (in)", robot.shooter.llDistance);
            dashboardTelemetry.addData("Velocity OK", robot.shooter.velocityReached());
            dashboardTelemetry.addData("Aimed", robot.shooter.isAimed());
            dashboardTelemetry.addData("encoder pos", robot.shooter.turretEncoder.getCurrentPosition());
            dashboardTelemetry.addData("Pose", robot.follower.getPose());
            dashboardTelemetry.update();
        }

    }
}