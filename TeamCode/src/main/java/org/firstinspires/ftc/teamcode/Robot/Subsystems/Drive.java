package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Robot.uV;

public class Drive {
    private DcMotorEx leftFront, rightFront, leftRear, rightRear;
public Drive(HardwareMap hwMap){
    leftFront = hwMap.get(DcMotorEx.class, "leftFront");
    leftRear = hwMap.get(DcMotorEx.class, "leftRear");
    rightFront = hwMap.get(DcMotorEx.class, "rightFront");
    rightRear = hwMap.get(DcMotorEx.class, "rightRear");

    leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
}
public void updateDrive() {
        double y = -gamepad1.left_stick_y*uV.drivePower; // Remember, this is reversed!
        double x = gamepad1.left_stick_x*uV.drivePower; // this is strafing
        double rx = gamepad1.right_stick_x*uV.drivePower;

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
}
