package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "Leave", group = "1. Auto Tests")
public class LeaveFar extends OpMode {

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);


        Robot.follower.setStartingPose(new Pose(0, 0, 0));
    }

    @Override
    public void start() {
        Robot.follower.startTeleOpDrive();

    }

    @Override
    public void loop() {

        Robot.follower.update();


        Robot.follower.setTeleOpDrive(-1, 0, 0, true);


    }
}
