package org.firstinspires.ftc.teamcode.Auto.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.Robot.Utils.PIDFController;

@Autonomous(name = "Motor test", group = "1. Auto Tests")
public class MotorTest extends OpMode {
    private DcMotorEx motor;


    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "testMotor");

    }

    @Override
    public void loop() {

        motor.setPower(1);

    }
}