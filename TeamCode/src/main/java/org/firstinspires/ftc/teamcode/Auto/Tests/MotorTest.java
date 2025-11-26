package org.firstinspires.ftc.teamcode.Auto.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import java.util.Arrays;
import java.util.List;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Motor test", group = "1. Auto Tests")
public class MotorTest extends OpMode {
    private DcMotorEx TestMotor;

    @Override
    public void init() {

        TestMotor = hardwareMap.get(DcMotorEx.class, "TestMotor");

        TestMotor.setDirection(Constants.mecanumConstants.leftFrontMotorDirection);

        TestMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {
        TestMotor.setPower(1);
    }
}