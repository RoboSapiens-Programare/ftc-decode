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

@Autonomous(name = "Motor test", group = "1. Auto Tests")
public class MotorTest extends OpMode {
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;

    private DcMotorEx geko;


    private void updateFollower(double power, double gx, double gy, double gr) {
        double y = -gy*power; // Remember, this is reversed!
        double x = gx*power; // this is strafing
        double rx = gr*power;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator;
        double leftRearPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightRearPower = (y + x - rx) / denominator;

        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);
    }

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, Constants.mecanumConstants.leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, Constants.mecanumConstants.leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, Constants.mecanumConstants.rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, Constants.mecanumConstants.rightFrontMotorName);
        leftFront.setDirection(Constants.mecanumConstants.leftFrontMotorDirection);
        leftRear.setDirection(Constants.mecanumConstants.leftRearMotorDirection);
        rightFront.setDirection(Constants.mecanumConstants.rightFrontMotorDirection);
        rightRear.setDirection(Constants.mecanumConstants.rightRearMotorDirection);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        geko = hardwareMap.get(DcMotorEx.class, "gekoMotor");
        geko.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        updateFollower(1, 0, 1, 0);
        geko.setPower(1);
    }
}
