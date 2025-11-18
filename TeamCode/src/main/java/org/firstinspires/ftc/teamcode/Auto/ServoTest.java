package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Config
@Autonomous(name = "servo test", group = "1. Auto Tests")
public class ServoTest extends OpMode {

    private Robot robot;
    private ElapsedTime timer = new ElapsedTime();
    private boolean singleton = true;
    private CRServo servo;
    public static double mata = 0.1;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        servo = (CRServo) hardwareMap.get("revolverSpin");
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        if (singleton) {
            singleton = false;
            timer.reset();
        }
        if (timer.milliseconds() > 1000) {
            servo.setPower(0);
        } else {
            servo.setPower(mata);
        }

        FtcDashboard.getInstance().getTelemetry().addData("Timer", timer.milliseconds());
        FtcDashboard.getInstance().getTelemetry().update();
    }
}
