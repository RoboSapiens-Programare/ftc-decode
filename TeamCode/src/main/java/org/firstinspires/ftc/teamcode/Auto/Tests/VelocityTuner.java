package org.firstinspires.ftc.teamcode.Auto.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.uV;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Velocity")
public class VelocityTuner extends OpMode {

    private Robot robot;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        Robot.follower = Constants.createFollower(hardwareMap);

        robot.shooter.shooting = true;

        Robot.follower.setStartingPose(Robot.transitionPose);
    }

    @Override
    public void loop() {
        dashboardTelemetry.addData("current velocity", robot.shooter.turretMotorLeft.getVelocity());
        dashboardTelemetry.addData("odom distance", robot.shooter.getOdometryDistance());

        dashboardTelemetry.update();

        Robot.follower.update();
        robot.shooter.update();

    }
}
