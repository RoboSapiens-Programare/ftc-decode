package org.firstinspires.ftc.teamcode.TeleOP;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;
import java.util.List;

@Configurable
@TeleOp(name = "unga bunga", group = "0. TeleOp")
public class PizdelDrive extends OpMode {
    private Robot robot;
//    private Follower follower;

    private ElapsedTime speedChangeTimer = new ElapsedTime();
    private boolean slowMode = false;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;

    private void updateFollower(double power) {
        double y = gamepad1.left_stick_y*power; // Remember, this is reversed!
        double x = -gamepad1.left_stick_x*power; // this is strafing
        double rx = -gamepad1.right_stick_x*power;

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

        speedChangeTimer.reset();
    }

    @Override
    public void init() {

        //robot = new Robot(hardwareMap);

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

    }

    @Override
    public void loop() {
        if (gamepad1.touchpad && speedChangeTimer.seconds() > .5){
            slowMode = !slowMode;
            speedChangeTimer.reset();
        }

        updateFollower(1);

        telemetry.update();
    }
}