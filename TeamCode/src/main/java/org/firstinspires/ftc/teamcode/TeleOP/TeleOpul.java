package org.firstinspires.ftc.teamcode.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.uV;
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

        if (newState == State.OUTTAKE) {
            robot.shooter.openGate();
            robot.shooter.shooting = true;
        } else {
            robot.shooter.closeGate();
            robot.shooter.shooting = false;
        }

    }

    private void handleIntake() {

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

        if (gamepad1.right_trigger > 0.1) {
            robot.intake.shoot();
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

        Robot.follower.setStartingPose(Robot.transitionPose);

    }
    
    @Override
    public void init_loop()
    {
        if (gamepad1.share)
        {
            gamepad1.setLedColor(0,0,255, 1000);
            Robot.alliance = Robot.Alliance.BLUE;
        } else if (gamepad1.options) {
            gamepad1.setLedColor(255,0,0,1000);
            Robot.alliance = Robot.Alliance.RED;
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

        telemetry.addData("encoder 1", robot.shooter.turretMotorRight.getCurrentPosition());
        telemetry.addData("encoder 2", robot.shooter.turretMotorLeft.getVelocity());
        telemetry.addData("distance intake", robot.intake.sensorIntake.getDistance(DistanceUnit.CM));
        telemetry.addData("distance outtake", robot.intake.sensorOuttake.getDistance(DistanceUnit.CM));
        telemetry.addData("distance mid", robot.intake.sensorMid.getDistance(DistanceUnit.CM));


        dashboardTelemetry.addData("State", state);

        dashboardTelemetry.update();

        Robot.follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x - 0.1 * gamepad2.right_stick_x,
                true);

        Robot.follower.update();
        robot.shooter.update();

        robot.intake.updateHeadlight();

    }
}
