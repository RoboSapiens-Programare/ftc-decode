package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "servo test", group = "1. Auto Tests")
public class ServoTest extends OpMode {

    private Robot robot;
    private ElapsedTime timer = new ElapsedTime();
    private boolean singleton = true;
    private Servo servo;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        servo = (Servo) hardwareMap.get("servo");

        servo.setPosition(0.5);
    }

    @Override
    public void start() {
//        timer.reset();
    }

    @Override
    public void loop() {
        if (timer.seconds() > 1) {
            if (singleton) {
                servo.setPosition(0.5);
                singleton = false;
            }
            if (timer.seconds() > 2) {
                servo.setPosition(0);
                singleton = true;
            }
        }

    }
}
