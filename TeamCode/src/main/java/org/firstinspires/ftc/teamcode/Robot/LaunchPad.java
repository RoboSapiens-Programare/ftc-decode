package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LaunchPad {
    private DcMotorEx gekoLauncherFinal;
    private DcMotorEx gekoLoader;
    private Servo outtakeServo;
    private Servo gateServo;

    public LaunchPad(HardwareMap hardwareMap) {
        gekoLauncherFinal = hardwareMap.get(DcMotorEx.class, "gekoMotor");
        gekoLoader = hardwareMap.get(DcMotorEx.class, "gekoLoader");
        outtakeServo = hardwareMap.get(Servo.class, "outtakeServo");
        gateServo = hardwareMap.get(Servo.class, "gateServo");

        gekoLauncherFinal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gekoLoader.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gekoLauncherFinal.setDirection(DcMotorSimple.Direction.FORWARD);
        gekoLoader.setDirection(DcMotorSimple.Direction.FORWARD);

        gekoLauncherFinal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gekoLoader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void StartLaunchTrain() {
        gekoLauncherFinal.setPower(RobotConstants.gekoLauncherPower);
    }

    public void StartGekoIntake()
    {
        gekoLauncherFinal.setPower(RobotConstants.gekoIntakePower);
    }

    public void StopLaunchTrain() {
        gekoLauncherFinal.setPower(0);
    }

    public void Launch() {
        outtakeServo.setPosition(RobotConstants.outtakeServoOpen);
    }

    public void Reset() {
        outtakeServo.setPosition(RobotConstants.outtakeServoClose);
    }

    public void OpenGate()
    {
        gateServo.setPosition(RobotConstants.openGate);
    }

    public void Load() {
        gekoLoader.setPower(0.6);
        gekoLauncherFinal.setPower(0.7);
    }

    public void StopLoad() {
        gekoLoader.setPower(0);
    }
    public void CloseGate()
    {
        gateServo.setPosition(RobotConstants.closeGate);
    }
}
