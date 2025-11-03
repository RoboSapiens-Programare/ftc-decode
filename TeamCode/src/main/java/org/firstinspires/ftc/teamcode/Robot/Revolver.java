package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Revolver {
    private CRServo revolverSpin;
    private Servo lift;

    private int targetPosition = -1;

    private TouchSensor limitSwitchLeftIntake, limitSwitchRightIntake, limitSwitchLeftOuttake, limitSwitchRightOuttake;

    private TouchSensor limitSwitchLeft, limitSwitchRight;

    public enum Mode {
        INTAKE,
        OUTTAKE
    };

    private Mode mode = Mode.INTAKE;

    public Revolver(HardwareMap hwMap) {
        revolverSpin = hwMap.get(CRServo.class, "revolverSpin");
        lift = hwMap.get(Servo.class, "lift");
        limitSwitchLeftIntake = hwMap.get(TouchSensor.class, "limitSwitchLeftIntake");
        limitSwitchRightIntake = hwMap.get(TouchSensor.class, "limitSwitchRightIntake");
        limitSwitchLeftOuttake = hwMap.get(TouchSensor.class, "limitSwitchLeftOuttake");
        limitSwitchRightOuttake = hwMap.get(TouchSensor.class, "limitSwitchRightOuttake");
    }

    public Mode getMode() {
        return mode;
    }

    public void setMode(Mode m) {
        mode = m;

        switch (mode) {
            case INTAKE:
                limitSwitchLeft = limitSwitchLeftIntake;
                limitSwitchRight = limitSwitchRightIntake;
                break;

            case OUTTAKE:
                limitSwitchLeft = limitSwitchLeftOuttake;
                limitSwitchRight = limitSwitchRightOuttake;
                break;
        }
    }

    public int getCurrentPosition() {
        if (limitSwitchLeft.isPressed() && !limitSwitchRight.isPressed()) {
            return 0;
        }

        if (limitSwitchLeft.isPressed() && limitSwitchRight.isPressed()) {
            return 1;
        }

        if (!limitSwitchLeft.isPressed() && limitSwitchRight.isPressed()) {
            return 2;
        }
        return -1;
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public void setTargetPosition(int position) {
        targetPosition = position;

        if (getCurrentPosition() == position) {
            revolverSpin.setPower(0);
        } else if (getCurrentPosition() == -1 || getCurrentPosition() > targetPosition) {
            revolverSpin.setPower(uV.revolverPower);
        } else if (getCurrentPosition() < targetPosition) {
            revolverSpin.setPower(-uV.revolverPower);
        }
    }

    public void update() {
        if (getCurrentPosition() == targetPosition) {
            revolverSpin.setPower(0);
        }
    }

    public void load() {
        lift.setPosition(uV.liftUp);
    }

    public void retract() {
        lift.setPosition(uV.uppiesDown);
    }

    public void nextSlot() {
        setTargetPosition((getTargetPosition() + 1) % 3);
    }

    public void prevSlot() {
        setTargetPosition((getTargetPosition() - 1) % 3);
    }
}
