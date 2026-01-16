package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Robot.Utils.ColorEnum;
import org.firstinspires.ftc.teamcode.Robot.Utils.PIDFController;
import org.firstinspires.ftc.teamcode.Robot.uV;

import java.util.Arrays;

public class Spindexer extends Subsystem {
    public DcMotorEx motor;
    public TouchSensor limitSwitch;

    // TODO: change to actual value
    public static double ticksPerRevolution = 1;

    public static double Kp = 0;
    public static double Kd = 0;
    public static double Ki = 0;
    public static double Kf = 0;

    // TODO: change to byte for memory effieciency
    private byte targetSlot = 0;
    public static double tolerance = 5;
    public static double targetPosition = 0;
    private boolean homing = true;

    private PIDFController pidfController = new PIDFController(Kp, Ki, Kd, Kf);

    public static final double shootDirection = 1;

    /* greenMotifPosition means the position [0, 1, 2] of the green ball in the motif
     * 0 -> GPP
     * 1 -> PGP
     * 2 -> PPG
     */
    public static int greenMotifPosition = 0;

    public ColorEnum[] slotColors = {ColorEnum.UNDEFINED, ColorEnum.UNDEFINED, ColorEnum.UNDEFINED};

    public enum Mode {
        INTAKE,
        OUTTAKE
    };

    public Mode mode = Mode.INTAKE;

    public Spindexer(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, "spindexerMotor");

        // if using custom PID controller
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // uncomment if using pre-defined PID
        // motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pidfController.setTolerance(25);

        limitSwitch = hwMap.get(TouchSensor.class, "spindexerLimitSwitch");
    }

    // Moving functions
    public void goToSlot(int slot) {
        double distance = Math.abs(targetPosition - slot) * ticksPerRevolution / 3;

        targetPosition += -1 * shootDirection * distance;

        pidfController.setSetpoint(targetPosition);
    }

    public void shootCurrentSlot() {
        if (!isReady()) return;

        slotColors[targetSlot++] = ColorEnum.UNDEFINED;

        targetPosition += shootDirection * ticksPerRevolution / 3;
        pidfController.setSetpoint(targetPosition);
    }

    // Sorting functions

    // getter and setter functions
    public void setSlotColor(int i, ColorEnum color) {
        slotColors[i] = color;
    }

    public ColorEnum getSlotColor(int i) {
        return slotColors[i];
    }

    // slot availability functions
    public boolean isSlotFull(int targetSlot) {
        return slotColors[targetSlot] != ColorEnum.UNDEFINED;
    }

    public boolean isSlotFree(int targetSlot) {
        return !isSlotFull(targetSlot);
    }

    public byte getFreeSlot() {
        for (byte b = 0; b < slotColors.length; ++b) {
            if (slotColors[b] == ColorEnum.UNDEFINED) {
                return b;
            }
        }

        return -1;
    }

    public int getFullSlot() {
        for (int b = 0; b < slotColors.length; ++b) {
            if (slotColors[b] != ColorEnum.UNDEFINED) {
                return b;
            }
        }

        return -1;
    }

    public int getSlotByColor(ColorEnum color) {
        for (int b = 0; b < slotColors.length; ++b) {
            if (slotColors[b] == color) {
                return b;
            }
        }

        return -1;
    }

    // motif functions

    public void motifGoToStart() {
        int greenSlot = getSlotByColor(ColorEnum.GREEN);
        int begin = (greenSlot - greenMotifPosition) % 3;

        goToShootStartPose(begin);
    }

    public void goToShootStartPose(int slot) {
        double distance = Math.abs(targetPosition - slot) * ticksPerRevolution / 3;

        targetPosition += -1 * shootDirection * distance + uV.shootOffset;
        pidfController.setSetpoint(targetPosition);

        while (!pidfController.targetReached()) {
            motor.setPower(pidfController.updatePID(motor.getCurrentPosition()));
        }

        goToSlot(slot);
    }

    // system-status functions

    public boolean isReady() {
        return pidfController.targetReached() && !homing;
    }

    public void setTargetSlot(byte targetSlot) {
        this.targetSlot = targetSlot;
        goToSlot(targetSlot);
    }

    public byte getTargetSlot() {
        return targetSlot;
    }

    // homing functions
    public void home() {
        reset();
        this.homing = true;
    }

    @Override
    public void reset() {
        Arrays.fill(slotColors, ColorEnum.UNDEFINED);
    }

    @Override
    public void update() {
        // TODO: remove these once tuned
        pidfController.kP = Kp;
        pidfController.kI = Ki;
        pidfController.kD = Kd;
        pidfController.kF = Kf;

        // uncomment if using pre-defined PID
        // motor.setVelocityPIDFCoefficients(Kp, Ki, Kd, Kf);

        // update PID controller
        if (!homing)
            motor.setPower(pidfController.updatePID(motor.getCurrentPosition()));
        else if (limitSwitch.isPressed()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            motor.setPower(-shootDirection * .4);
        }

        // uncomment if using pre-defined PID
        // motor.setTargetPosition()
    }
}
