package org.firstinspires.ftc.teamcode.Auto.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.Robot.Utils.PIDFController;

@Autonomous(name = "Double Motor test", group = "1. Auto Tests")
public class DualMotorTest extends OpMode {
    private DcMotorEx motor;
    private DcMotorEx motor2;


    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "testMotor");
        motor2 = hardwareMap.get(DcMotorEx.class, "testMotor2");

        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {

        motor.setPower(1);
        motor2.setPower(1);

    }
}