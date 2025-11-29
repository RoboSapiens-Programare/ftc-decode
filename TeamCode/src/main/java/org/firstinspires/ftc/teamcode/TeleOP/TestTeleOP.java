package org.firstinspires.ftc.teamcode.TeleOP;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Revolver;

@Config
@TeleOp(name = "TestTeleOp")
public class TestTeleOP extends OpMode {
    private Robot robot;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    private final ElapsedTime inputTimer = new ElapsedTime();

    private float applyDeadzone(float input) {
        // 0.1 is the threshold (10%). Adjust if your controller is older/looser.
        if (abs(input) < 0.05f) {
            return 0.0f;
        }
        return input;
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.revolver.liftReset();

        inputTimer.reset();
        gamepad2.setLedColor(255, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);

        robot.drive.driverGamepad = gamepad1;

        // TODO: change to auto park pos
        Robot.follower.setStartingPose(new Pose(72, 72));
    }

    @Override
    public void start() {
        gamepad1.setLedColor(255, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
        gamepad2.setLedColor(255, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);

        robot.revolver.mode = Revolver.Mode.INTAKE;
        robot.revolver.setTargetSlot((byte) 0);
    }

    @Override
    public void loop() {
        robot.drive.update();
        robot.revolver.update();
        robot.turret.update();

        Robot.follower.update();

        if (gamepad1.dpad_left && inputTimer.milliseconds() > 400) {
            robot.revolver.prevSlot();
            inputTimer.reset();
        }

        if (gamepad1.dpad_right && inputTimer.milliseconds() > 400) {
            robot.revolver.nextSlot();
            inputTimer.reset();
        }

        dashboardTelemetry.addData("target position", robot.revolver.target);

        dashboardTelemetry.update();
    }
}
