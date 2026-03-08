package org.firstinspires.ftc.teamcode.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TeleOp")
public class TeleOpul extends OpMode {

    private Robot robot;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    private final ElapsedTime stateTimer = new ElapsedTime();
    private int loopCount = 0;


    enum State {
        INTAKE,
        OUTTAKE
    }

    private State state = State.INTAKE;

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
            changeState(State.OUTTAKE);
        }
    }

    private void handleOuttake() {
        robot.shooter.shooting = true;

        robot.shooter.openGate();

//        robot.shooter.track();

//        if (robot.shooter.velocityReached()
//                && robot.shooter.isShootReady()
//                && gamepad1.right_trigger > 0.1) {
//            robot.intake.shoot();
//        }

        // În handleOuttake() — TeleOp
        if (gamepad1.right_trigger > 0.1
                && robot.shooter.velocityReached())
//                && robot.shooter.isAimed())
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
    public void init() {


        robot = new Robot(hardwareMap);

        state = State.INTAKE;

        Robot.follower = Constants.createFollower(hardwareMap);

        Robot.follower.startTeleOpDrive(true);

        Pose startPoseBlue = new Pose(56, 8, Math.toRadians(90));
        Robot.follower.setPose(startPoseBlue);


        robot.shooter.openGate();
    }

    public void init_loop()
    {
        if (gamepad1.options)
        {
            Robot.alliance = Robot.Alliance.RED;
            gamepad1.setLedColor(255, 0,0,1000);
        } else if (gamepad1.share) {
            Robot.alliance = Robot.Alliance.BLUE;
            gamepad1.setLedColor(0, 0,255,1000);
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


        Robot.follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x - 0.1 * gamepad2.right_stick_x,
                true);

        Robot.follower.update();
        robot.shooter.update();
        loopCount++;
        if (loopCount % 5 == 0) {
            dashboardTelemetry.addData("State", state);
            dashboardTelemetry.addData("Distance (in)", robot.shooter.llDistance);
            dashboardTelemetry.addData("Flywheel RPM", -robot.shooter.turretMotorLeft.getVelocity());
            dashboardTelemetry.update();
            dashboardTelemetry.addData("Track State", robot.shooter.trackState);
            dashboardTelemetry.addData("Turret Output", robot.shooter.turretOutput);
            dashboardTelemetry.addData("Turret Error", Math.toDegrees(robot.shooter.turretErrorRad));
            dashboardTelemetry.addData("Target RPM", robot.shooter.targetVelocity);
            dashboardTelemetry.addData("Actual RPM", -robot.shooter.turretMotorLeft.getVelocity());
            dashboardTelemetry.addData("Distance (in)", robot.shooter.llDistance);
            dashboardTelemetry.addData("Velocity OK", robot.shooter.velocityReached());
            dashboardTelemetry.addData("Aimed", robot.shooter.isAimed());
            dashboardTelemetry.update();
        }

    }
}