package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.uV;

public class Revolver {
    private CRServo revolverSpin;
    private Servo lift;

    private byte targetSlot = 0;

    public AnalogInput encoder;




    public enum Mode {
        INTAKE,
        OUTTAKE
    }

    ;

    public Mode mode = Mode.INTAKE;

    public Revolver(HardwareMap hwMap) {
        revolverSpin = hwMap.get(CRServo.class, "revolverSpin");
        encoder = hwMap.get(AnalogInput.class, "encoder");
        revolverSpin.setDirection(CRServo.Direction.REVERSE);
        lift = hwMap.get(Servo.class, "lift");
    }

    public Mode getMode() {
        return mode;
    }

    public void setPosition() {
        switch (mode) {
            case INTAKE:

                break;
            case OUTTAKE:

                break;

        }
    }



    public byte getTargetSlot() {
        return targetSlot;
    }

    public void load() {
        lift.setPosition(uV.liftUp);
    }

    public void retract() {
        lift.setPosition(uV.uppiesDown);
    }

    public void nextSlot() {
        if (targetSlot == 2)
            targetSlot = 0;
        else
            targetSlot++;
    }

    public void prevSlot() {
        if (targetSlot == 0)
            targetSlot = 2;
        else
            targetSlot--;

    }
}
