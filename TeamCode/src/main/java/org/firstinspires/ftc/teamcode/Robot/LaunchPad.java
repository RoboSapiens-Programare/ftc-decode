package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LaunchPad {
    private DcMotorEx gekoLauncherM1;
    private DcMotorEx gekoLauncherM2;
    private DcMotorEx gekoLauncherFinal;
    private Servo gateServo;

    public LaunchPad(HardwareMap hwMap) {
        gekoLauncherM1 = hwMap.get(DcMotorEx.class, "gekoLauncherM1");
        gekoLauncherM2 = hwMap.get(DcMotorEx.class, "gekoLauncherM2");
        gekoLauncherFinal = hwMap.get(DcMotorEx.class, "gekoLauncherFinal");
        gateServo = hwMap.get(Servo.class, "gateServo");


        gekoLauncherM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gekoLauncherM2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gekoLauncherFinal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gekoLauncherM1.setDirection(DcMotorSimple.Direction.FORWARD);
        gekoLauncherM2.setDirection(DcMotorSimple.Direction.REVERSE);
        gekoLauncherFinal.setDirection(DcMotorSimple.Direction.FORWARD);

        gekoLauncherM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gekoLauncherM2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gekoLauncherFinal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void StartLaunchTrain() {
        gekoLauncherM1.setPower(RobotConstants.gekoLauncherMPower);
        gekoLauncherM2.setPower(RobotConstants.gekoLauncherMPower);
        gekoLauncherFinal.setPower(RobotConstants.gekoLauncherFinalPower);
    }

    public void StopLaunchTrain() {
        gekoLauncherM1.setPower(0);
        gekoLauncherM2.setPower(0);
        gekoLauncherFinal.setPower(0);
    }

    public void OpenGate() {
        gateServo.setPosition(RobotConstants.gateServoOpen);
    }

    public void CloseGate() {
        gateServo.setPosition(RobotConstants.gateServoClose);
    }
}
