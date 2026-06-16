package org.firstinspires.ftc.teamcode.Auto.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "servo 0", group = "1. Auto Tests")
public class ServoZero extends OpMode {
    private Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "lobServo");
    }

    @Override
    public void loop() {
        // servo 1 is left
        servo.setPosition(0.2);
    }
}
