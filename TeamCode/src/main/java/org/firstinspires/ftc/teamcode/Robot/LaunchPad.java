package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LaunchPad {
    private DcMotorEx geckoLauncherFinal;
    private DcMotorEx geckoLoader;
    private Servo outtakeServo;
    private Servo gateServo;

    private Servo leftRoomPush;
    private Servo rightRoomPush;

    private Servo roomGate;



    public LaunchPad(HardwareMap hardwareMap) {
        geckoLauncherFinal = hardwareMap.get(DcMotorEx.class, "geckoMotor");
        geckoLoader = hardwareMap.get(DcMotorEx.class, "geckoLoader");
        outtakeServo = hardwareMap.get(Servo.class, "outtakeServo");
        gateServo = hardwareMap.get(Servo.class, "gateServo");

        leftRoomPush = hardwareMap.get(Servo.class, "leftRoomPush");
        rightRoomPush = hardwareMap.get(Servo.class, "rightRoomPush");

        roomGate = hardwareMap.get(Servo.class, "roomGate");

        geckoLauncherFinal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        geckoLoader.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        geckoLauncherFinal.setDirection(DcMotorSimple.Direction.FORWARD);
        geckoLoader.setDirection(DcMotorSimple.Direction.FORWARD);

        geckoLauncherFinal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        geckoLoader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void StartLaunchTrain() {
        geckoLauncherFinal.setPower(uV.outtakePower);
    }

    public void StartGekoIntake()
    {
        geckoLauncherFinal.setPower(uV.intakePower);
    }

    public void StopLaunchTrain() {
        geckoLauncherFinal.setPower(0);
    }

    public void Launch() {
        outtakeServo.setPosition(uV.outtakeServoOpen);
    }

    public void Reset() {
        outtakeServo.setPosition(uV.outtakeServoClose);
    }

    public void OpenGate()
    {
        gateServo.setPosition(uV.camLeversUp);
    }

    public void    OpenLeftRoom()
    {
        leftRoomPush.setPosition(1);
        roomGate.setPosition(1);
    }

    public void CloseLeftRoom()
    {
        leftRoomPush.setPosition(0);
        roomGate.setPosition(0);
    }

    public void Load() {
        geckoLoader.setPower(0.6);
        geckoLauncherFinal.setPower(0.7);
    }

    public void StopLoad() {
        geckoLoader.setPower(0);
    }
    public void CloseGate()
    {
        gateServo.setPosition(uV.camLeversUp);
    }
}
