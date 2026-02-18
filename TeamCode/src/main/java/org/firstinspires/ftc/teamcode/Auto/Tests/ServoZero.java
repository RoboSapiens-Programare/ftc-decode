package org.firstinspires.ftc.teamcode.Auto.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.Robot.Utils.PIDFController;

@Autonomous(name = "servo 0", group = "1. Auto Tests")
public class ServoZero extends OpMode {
    private Servo servo;
    private Servo servo2;


    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");
        servo2 = hardwareMap.get(Servo.class, "servo2");
    }

    @Override
    public void loop() {

        //servo 1 is left
        servo.setPosition(0.6);
        servo2.setPosition(0.5);

    }
}