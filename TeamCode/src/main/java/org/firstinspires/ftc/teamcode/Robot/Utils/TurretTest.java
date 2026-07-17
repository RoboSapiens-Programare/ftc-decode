package org.firstinspires.ftc.teamcode.Robot.Utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "test tureta")
@Disabled
public class TurretTest extends OpMode {

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

    @Override
    public void init() {

        robot = new Robot(hardwareMap);

        state = State.INTAKE;

        Robot.follower = Constants.createFollower(hardwareMap);

        Robot.follower.startTeleOpDrive(true);

        Robot.follower.setStartingPose(Robot.transitionPose);
    }

    @Override
    public void loop() {

        dashboardTelemetry.addData("State", state);

        dashboardTelemetry.update();

        Robot.follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x - 0.1 * gamepad2.right_stick_x,
                true);

        //        Robot.follower.update();
        //        robot.shooter.update();

        robot.shooter.lobServo.setPosition(1);
    }
}
