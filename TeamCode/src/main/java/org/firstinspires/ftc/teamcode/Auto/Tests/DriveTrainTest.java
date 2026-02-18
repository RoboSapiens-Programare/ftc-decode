package org.firstinspires.ftc.teamcode.Auto.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "drive train test")
public class DriveTrainTest extends OpMode {

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        Robot.follower = Constants.createFollower(hardwareMap);

        Robot.follower.startTeleOpDrive(true);



        Robot.follower.setStartingPose(new Pose(0,0));
    }

    @Override
    public void loop() {
        Robot.follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x - 0.1 * gamepad2.right_stick_x,
                true);

        Robot.follower.update();
    }


}
