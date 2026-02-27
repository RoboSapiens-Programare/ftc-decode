package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
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

    public Intake(HardwareMap hwMap) {

        rollerOne = hwMap.get(DcMotorEx.class, "rollerOne");
        rollerTwo = hwMap.get(DcMotorEx.class, "rollerTwo");

        rollerOne.setDirection(DcMotorSimple.Direction.REVERSE);
        rollerTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        rollerOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rollerTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pivotLeft = hwMap.get(Servo.class, "intakePivotLeft");

        pivotRight = hwMap.get(Servo.class, "intakePivotRight");
        pivotRight.setDirection(Servo.Direction.REVERSE);

    }

    @Override
    public void update() {}

    public void shoot() {
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
        intakeMid();
    }

    public void rest() {
        rollerOne.setPower(0);
        rollerTwo.setPower(0);
        intakeMid();
    }

    public void intakeDown() {
        pivotLeft.setPosition(uV.intakeDown);
        pivotRight.setPosition(uV.intakeDown);
    }

    public void intakeMid() {
        pivotLeft.setPosition(uV.intakeMid);
        pivotRight.setPosition(uV.intakeMid);
    }
}
