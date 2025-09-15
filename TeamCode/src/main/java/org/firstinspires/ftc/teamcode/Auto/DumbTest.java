package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Dumb test", group = "1. Auto Tests")
public class DumbTest extends OpMode {
    private DcMotorEx leftFront;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, Constants.mecanumConstants.leftFrontMotorName);
        leftFront.setDirection(Constants.mecanumConstants.leftFrontMotorDirection);
    }

    @Override
    public void loop() {
        leftFront.setPower(0.4);
    }
}
