package org.firstinspires.ftc.teamcode.Auto.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

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

        // servo 1 is left
        servo.setPosition(0.5);
        servo2.setPosition(0.5);
    }
}
