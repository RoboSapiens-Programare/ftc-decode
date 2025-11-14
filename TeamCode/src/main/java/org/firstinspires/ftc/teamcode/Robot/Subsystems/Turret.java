package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot.uV;

public class Turret {
    public DcMotorEx turretMotor;
    private CRServo turretRotationServo;
    private float turretRotation;

    public Turret(HardwareMap hwMap) {
        turretMotor = hwMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretRotationServo = hwMap.get(CRServo.class, "turretRotationServo");
    }
    public void setAngle(float rotation) {
        turretRotation = rotation;
        turretRotationServo.setPower(rotation);
    }
    public float getTurretRotation() {
        return turretRotation;
    }
    public void setPower(float power) {
        turretMotor.setPower(power);
    }
    public void startMotor() {
        turretMotor.setPower(uV.outtakePower);
    }

    public void stopMotor() {
        turretMotor.setPower(0);
    }
}
