package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.uV;

@SuppressWarnings("FieldCanBeLocal")
public class Intake extends Subsystem {
    private final DcMotorEx rollerOne;
    private final DcMotorEx rollerTwo;
    public final Servo headlight;

    public final DistanceSensor sensorIntake;
    public final DistanceSensor sensorOuttake;
    public final DistanceSensor sensorMid;
    private final ElapsedTime hlTimer = new ElapsedTime();
    private final ElapsedTime ballTimer = new ElapsedTime();

    public Intake(HardwareMap hwMap) {

        rollerOne = hwMap.get(DcMotorEx.class, "rollerOne");
        rollerTwo = hwMap.get(DcMotorEx.class, "rollerTwo");

        rollerOne.setDirection(DcMotorSimple.Direction.REVERSE);
        rollerTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        rollerOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rollerTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        headlight = hwMap.get(Servo.class, "headlight");

        sensorIntake = hwMap.get(DistanceSensor.class, "sensorIntake");
        sensorOuttake = hwMap.get(DistanceSensor.class, "sensorOuttake");
        sensorMid = hwMap.get(DistanceSensor.class, "sensorMid");
    }

    @Override
    public void update() {}

    public void updateHeadlight()
    {
        if (hlTimer.milliseconds()>100)
        {
            if (sensorOuttake.getDistance(DistanceUnit.CM) < 8 && sensorMid.getDistance(DistanceUnit.CM) < 8 && sensorIntake.getDistance(DistanceUnit.CM) < 8) {
                headlight.setPosition(1);
            } else {
                headlight.setPosition(0.277);
            }
        }
    }

    public void shoot() {
        rollerOne.setPower(uV.rollerOneP);
        rollerTwo.setPower(uV.rollerTwoP);
    }

    public boolean isEmpty()
    {
        return sensorMid.getDistance(DistanceUnit.CM)>8 && sensorIntake.getDistance(DistanceUnit.CM)>8 && sensorOuttake.getDistance(DistanceUnit.CM)>8;
    }

    public void pullBalls() {
        if (sensorOuttake.getDistance(DistanceUnit.CM) < 8)
        {
            rollerTwo.setPower(0);
        } else {
            rollerTwo.setPower(uV.rollerTwoP);
        }

        if (sensorMid.getDistance(DistanceUnit.CM) < 8 && sensorIntake.getDistance(DistanceUnit.CM) < 8)
        {
            rollerOne.setPower(0);
        } else {
            rollerOne.setPower(uV.rollerOneP);
        }
    }

    public void spitBalls() {
        rollerOne.setPower(-uV.rollerOneP);
        rollerTwo.setPower(-uV.rollerTwoP);
    }

    public void rest() {
        rollerOne.setPower(0);
        rollerTwo.setPower(0);
    }
}
