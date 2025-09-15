package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "servo test", group = "1. Auto Tests")
public class ServoTest extends OpMode {

    private Robot robot;
    private ElapsedTime timer = new ElapsedTime();
    private boolean singleton = true;


    @Override
    public void init() {
        robot = new Robot(hardwareMap);
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        if (timer.seconds() > 1) {
            if (singleton) {
                robot.lp.OpenGate();
                singleton = false;
            }
            if (timer.seconds() > 3) {
                robot.lp.CloseGate();
                if (timer.seconds() > 6) {
                    timer.reset();
                    singleton = true;
                }
            }
        }
    }
}
