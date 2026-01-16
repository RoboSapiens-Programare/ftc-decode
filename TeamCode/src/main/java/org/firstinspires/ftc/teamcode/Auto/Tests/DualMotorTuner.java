package org.firstinspires.ftc.teamcode.Auto.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Dual Motor Tuner", group = "2. Tuners")
public class DualMotorTuner extends OpMode {
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;

    @Override
    public void init() {
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        rightMotor.setPower(1);
        leftMotor.setPower(1);

        telemetry.addData("left motor tps", leftMotor.getVelocity());
        telemetry.addData("right motor tps", rightMotor.getVelocity());
    }
}
