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

    enum State {
        INTAKE,
        OUTTAKE
    }

    private State state = State.INTAKE;

    private void changeState(State newState) {
        state = newState;
        stateTimer.reset();


        robot.shooter.shooting = newState == State.OUTTAKE;
    }

    private void handleIntake() {
//        robot.shooter.shooting = false;

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
//        robot.shooter.shooting = true;

        robot.shooter.openGate();

//        robot.shooter.track();

//        if (robot.shooter.velocityReached()
//                && robot.shooter.isShootReady()
//                && gamepad1.right_trigger > 0.1) {
//            robot.intake.shoot();
//        }

        if (gamepad1.right_trigger > 0.1) {
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

        Robot.follower.setStartingPose(new Pose(0, 0));


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

        telemetry.addData("encoder 1", robot.shooter.turretMotorRight.getCurrentPosition());
        telemetry.addData("encoder 2", robot.shooter.turretMotorLeft.getVelocity());

        Robot.follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x - 0.1 * gamepad2.right_stick_x,
                true);

        Robot.follower.update();
        robot.shooter.update();
    }
}