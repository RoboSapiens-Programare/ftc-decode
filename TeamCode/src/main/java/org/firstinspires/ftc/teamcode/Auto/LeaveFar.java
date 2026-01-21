package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "Leave", group = "1. Auto Tests")
public class LeaveFar extends OpMode {

    private Robot robot;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        robot = new Robot(hardwareMap);


        Robot.follower.setStartingPose(new Pose(0, 0, 0));
    }

    @Override
    public void start() {
        Robot.follower.startTeleOpDrive();
        timer.reset();

    }

    @Override
    public void loop() {

        Robot.follower.update();

        if (timer.seconds() > 25) {
            Robot.follower.setTeleOpDrive(-1, 0, 0, true);
        }


    }
}
