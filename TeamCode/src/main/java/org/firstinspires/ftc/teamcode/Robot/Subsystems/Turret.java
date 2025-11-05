package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.uV;

public class Turret {
    public DcMotorEx turretMotor;
//    private Servo turretServo, pivotServo;
    private float turretAngle;

    public Turret(HardwareMap hwMap) {
        turretMotor = hwMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        turretServo = hwMap.get(Servo.class, "turretServo");
//        pivotServo = hwMap.get(Servo.class, "pivotServo");
    }
//    public void setAngle(float angle) {
//        turretAngle = angle;
//        turretServo.setPosition(angle);
//    }
//    public float getAngle() {
//        return turretAngle;
//    }
    public void setPower(float power) {
        turretMotor.setPower(power);
    }
    public void startMotor() {
        turretMotor.setPower(uV.outtakePower);
    }

    public void stopMotor() {
        turretMotor.setPower(0);
    }
//    public void setPivot(float position) {
//        pivotServo.setPosition(position);
//    }
}
