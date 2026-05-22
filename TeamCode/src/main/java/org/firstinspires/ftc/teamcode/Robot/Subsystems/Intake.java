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

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class Intake extends Subsystem {

    // Hardware
    private final CachingDcMotorEx rollerOne;
    private final CachingDcMotorEx rollerTwo;

    private final CachingServo headlight;

    private final DistanceSensor sensorIntake;
    private final DistanceSensor sensorOuttake;
    private final DistanceSensor sensorMid;

    public  double intakeDistance = 0;
    public double outtakeDistance = 0;
    public double midDistance = 0;

    public final ElapsedTime sensorTimer = new ElapsedTime();
    private final ElapsedTime hlTimer = new ElapsedTime();

    public Intake(HardwareMap hwMap) {
        rollerOne = new CachingDcMotorEx(hwMap.get(DcMotorEx.class, "rollerOne"));
        rollerTwo = new CachingDcMotorEx(hwMap.get(DcMotorEx.class, "rollerTwo"));

        rollerOne.setDirection(DcMotorSimple.Direction.REVERSE);
        rollerTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        rollerOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rollerTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        headlight = new CachingServo(hwMap.get(Servo.class, "headlight"));

        sensorIntake = hwMap.get(DistanceSensor.class, "sensorIntake");
        sensorOuttake = hwMap.get(DistanceSensor.class, "sensorOuttake");
        sensorMid = hwMap.get(DistanceSensor.class, "sensorMid");

        sensorTimer.reset();
    }

    @Override
    public void update() {
        updateHeadlight();
        if (sensorTimer.milliseconds() > 100) {
            outtakeDistance = sensorOuttake.getDistance(DistanceUnit.CM);
            intakeDistance = sensorIntake.getDistance(DistanceUnit.CM);
            midDistance = sensorMid.getDistance(DistanceUnit.CM);

            sensorTimer.reset();
        }
    }

    public void updateHeadlight() {
        if (hlTimer.milliseconds() > 100) {
            if (outtakeDistance < 8 && midDistance < 8 && intakeDistance < 8) {
                headlight.setPosition(1);
            } else {
                headlight.setPosition(0.277);
            }
        }
    }

    // Rollers
    public void shoot() {
        rollerOne.setPower(uV.rollerOneP);
        rollerTwo.setPower(uV.rollerTwoP);
    }

    public boolean isEmpty() {
        return midDistance > 8
                && intakeDistance > 8
                && outtakeDistance > 8;
    }

    public void pullBalls() {
        if (outtakeDistance < 8) {
            rollerTwo.setPower(0);
        } else {
            rollerTwo.setPower(uV.rollerTwoP);
        }

        if (midDistance < 8 && intakeDistance < 8) {
            rollerOne.setPower(0);
        } else {
            rollerOne.setPower(uV.rollerOneP);
        }
    }

    public void pullBallsHard() {
        if (outtakeDistance < 8) {
            rollerTwo.setPower(0);
        } else {
            rollerTwo.setPower(uV.rollerTwoP);
        }

        if (midDistance < 8 && intakeDistance < 8) {
            rollerOne.setPower(0.55);
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
