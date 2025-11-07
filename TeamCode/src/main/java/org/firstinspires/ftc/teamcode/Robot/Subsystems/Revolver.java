package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Utils.PIDController;
import org.firstinspires.ftc.teamcode.Robot.uV;

public class Revolver {
    private CRServo revolverSpin;
    private final Servo lift;

    double power = 0;

    private final PIDController pidController;
    private final ElapsedTime pidTimer = new ElapsedTime();

    private byte targetSlot = 0;

    public AnalogInput encoder;


    public enum Mode {
        INTAKE,
        OUTTAKE
    }

    public Mode mode = Mode.INTAKE;


    public Revolver(HardwareMap hwMap) {
        revolverSpin = hwMap.get(CRServo.class, "revolverSpin");
        encoder = hwMap.get(AnalogInput.class, "encoder");
        revolverSpin.setDirection(CRServo.Direction.REVERSE);
        lift = hwMap.get(Servo.class, "lift");

        pidController = new PIDController(uV.revolverKp, uV.revolverKi, uV.revolverKd);
    }

    public Mode getMode() {
        return mode;
    }

    public void setTargetSlot(byte targetSlot) {
        this.targetSlot = targetSlot;
        double start = mode == Mode.INTAKE ? uV.revolverPositonIntake0 : uV.revolverPositonOuttake0;
        double target = targetSlot * 5.0 / 3 + start;

        if (target >= 5) {
            target -= 5;
        } else if (target < 0) {
            target += 5;
        }

        pidController.setSetpoint(target);

        pidTimer.reset();
    }

    public byte getTargetSlot() {
        return targetSlot;
    }

    public void update() {
        power = pidController.update(getCurrentPosition(), pidTimer.milliseconds());
        revolverSpin.setPower(power);
        pidTimer.reset();
    }

    public void load() {
        lift.setPosition(uV.liftUp);
    }

    public void retract() {
        lift.setPosition(uV.uppiesDown);
    }

    public void nextSlot() {
        if (targetSlot == 2)
            setTargetSlot((byte) 0);
        else
            setTargetSlot((byte) (targetSlot + 1));
    }

    public void prevSlot() {
        if (targetSlot == 0)
            setTargetSlot((byte) 2);
        else
            setTargetSlot((byte) (targetSlot - 1));

    }

    public double getCurrentPosition() {
        return encoder.getVoltage();
    }

    public void telemetryDump() {
        double start = mode == Mode.INTAKE ? uV.revolverPositonIntake0 : uV.revolverPositonOuttake0;
        double target = targetSlot * 5.0 / 3 + start;

        telemetry.addData("ec pos", encoder.getVoltage());
        telemetry.addData("target slot", targetSlot);
        telemetry.addData("target pos", target);
        telemetry.addData("pow", power);
    }
}
