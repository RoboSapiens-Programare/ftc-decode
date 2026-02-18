package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.uV;

@SuppressWarnings("FieldCanBeLocal")
public class Intake extends Subsystem {
    private final DcMotorEx rollerOne;
    private final DcMotorEx rollerTwo;
    private final Servo pivotLeft;
    private final Servo pivotRight;
    private final Servo gate;

    public Intake(HardwareMap hwMap) {

        rollerOne = hwMap.get(DcMotorEx.class, "rollerOne");
        rollerTwo = hwMap.get(DcMotorEx.class, "rollerTwo");

        rollerOne.setDirection(DcMotorSimple.Direction.FORWARD);
        rollerTwo.setDirection(DcMotorSimple.Direction.FORWARD);

        pivotLeft = hwMap.get(Servo.class, "intakePivotLeft");
        pivotRight = hwMap.get(Servo.class, "intakePivotRight");

        gate = hwMap.get(Servo.class, "gate");
    }

    @Override
    public void update() {

    }

    public void openGate()
    {
        gate.setPosition(uV.gateOpen);
    }

    public void closeGate()
    {
        gate.setPosition(uV.gateClosed);
    }

    public void shoot()
    {
        rollerOne.setPower(uV.rollerOneP);
        rollerTwo.setPower(uV.rollerTwoP);

        intakeDown();
    }

    public void pullBalls() {
        rollerOne.setPower(uV.rollerOneP);
        rollerTwo.setPower(uV.rollerTwoP);
        intakeMid();
    }

    public void spitBalls() {
        rollerOne.setPower(-uV.rollerOneP);
        rollerTwo.setPower(-uV.rollerTwoP);
        intakeUp();
    }

    public void rest()
    {
        rollerOne.setPower(0);
        rollerTwo.setPower(0);
        intakeMid();
    }

    public void intakeDown()
    {
        pivotLeft.setPosition(uV.intakeDownLeft);
        pivotRight.setPosition(uV.intakeDownRight);
    }

    public void intakeUp()
    {
        pivotLeft.setPosition(uV.intakeUpLeft);
        pivotRight.setPosition(uV.intakeUpRight);
    }

    public void intakeMid()
    {
        pivotLeft.setPosition(uV.intakeLeftMid);
        pivotRight.setPosition(uV.intakeRightMid);
    }
}