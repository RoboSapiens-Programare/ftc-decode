package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Robot.Utils.ColorEnum;
import org.firstinspires.ftc.teamcode.Robot.Utils.PIDFController;

public class Spindexer extends Subsystem {
    public DcMotorEx motor;

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

    private PIDFController pidfController = new PIDFController(Kp, Ki, Kd, Kf);

    // e uite aici voiam sa bag un 2 enum-uri frumoas cu 2 directii si motif-uri dar voua nu v-ar fi
    // placut :<

    // TODO: remove this later
    public static final double shootDirection = 1;

    /* greenMotifPosition means the position [0, 1, 2] of the green ball in the motif
     * 0 -> GPP
     * 1 -> PGP
     * 2 -> PPG
     */
    public static int greenMotifPosition = 0;
    public static int currentMotifPosition = 0;

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
    }

    // Moving functions
    public void goToSlot(int slot) {
        double distance = Math.abs(targetPosition - slot) * ticksPerRevolution / 3;

        targetPosition += -1 * shootDirection * distance;

        pidfController.setSetpoint(targetPosition);
    }

    public void shootCurrentSlot() {
        if (!isReady()) return;

        slotColors[++targetSlot] = ColorEnum.UNDEFINED;

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

    public int getMotifOffset() {
        return greenMotifPosition - currentMotifPosition;

        // moves 2 slots if the offset is 2 becuase of passive 1way intake
    }

    public void gotoMotif() {
        goToSlot(targetSlot + getMotifOffset());
    }

    public void motifGoToStart() {
        int greenSlot = getSlotByColor(ColorEnum.GREEN);
        int begin = (greenSlot - greenMotifPosition) % 3;

        goToSlot(begin);
    }

    public void motifShootNext() {
        shootCurrentSlot();
    }

    // system-status functions

    public boolean isReady() {
        return pidfController.targetReached();
    }

    public void setTargetSlot(byte targetSlot) {
        this.targetSlot = targetSlot;
        goToSlot(targetSlot);
    }

    public byte getTargetSlot() {
        return targetSlot;
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
        motor.setPower(pidfController.updatePID(motor.getCurrentPosition()));

        // uncomment if using pre-defined PID
        // motor.setTargetPosition()
    }
}
