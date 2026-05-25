package org.firstinspires.ftc.teamcode.Auto.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.uV;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Velocity")
@Config
public class VelocityTuner extends OpMode {

    private Robot robot;
    public static double pos = 0.0;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        Robot.follower = Constants.createFollower(hardwareMap);

        robot.shooter.shooting = true;

        Robot.alliance = Robot.Alliance.BLUE;
        Pose startPoseBlue = new Pose(54, 8, Math.toRadians(90));;
        Robot.follower.setStartingPose(startPoseBlue);
    }

    @Override
    public void loop() {
        dashboardTelemetry.update();

        Robot.follower.update();
        robot.shooter.update();
        robot.intake.update();

        if (gamepad1.right_trigger > .1) {
            robot.intake.shoot();
        }

        robot.shooter.lobServo.setPosition(pos);
    }
}
