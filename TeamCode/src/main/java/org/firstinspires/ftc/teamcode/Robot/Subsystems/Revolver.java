package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.Robot.Utils.ColorEnum;
import org.firstinspires.ftc.teamcode.Robot.Utils.PIDFController;
import org.firstinspires.ftc.teamcode.Robot.uV;

@Config
public class Revolver extends Subsystem {
    public final DcMotorEx revolverSpin;
    private final Servo lift;
    public int greenPosition = 0;

    public TouchSensor revolverLimit, mihaiLimit;

    public ColorEnum[] colorList = {ColorEnum.UNDEFINED, ColorEnum.UNDEFINED, ColorEnum.UNDEFINED};

    // target slot and encoder position
    public byte targetSlot = 0;
    public static double target = 0;

    // PID values for revolver
    // WHEN TUNING USE ZIEGLER-NICHOLS METHOD
    // IT WAS MADE FOR THIS
    // LITERALLY FOR THIS
    public static double Kp = 0.008;
    public static double Kd = 0.0003;
    public static double Ki = 0;
    public static double Kf = 0; // Power to overcome inertia and friction

    private PIDFController pidfController = new PIDFController(Kp, Ki, Kd, Kf);
    public boolean homing = true;

    // Align with INTAKE / OUTTAKE
    public enum Mode {
        INTAKE,
        OUTTAKE
    };

    public Mode mode = Mode.INTAKE;

    public Revolver(HardwareMap hwMap) {
//        limitSwitch = hwMap.get(TouchSensor.class, "revolverLimitSwitch");


        revolverSpin = hwMap.get(DcMotorEx.class, "revolverSpin");
        revolverSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revolverSpin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        revolverSpin.setDirection(CRServo.Direction.REVERSE);

        lift = hwMap.get(Servo.class, "lift");

        revolverLimit = hwMap.get(TouchSensor.class, "revolverLimit");
        mihaiLimit = hwMap.get(TouchSensor.class, "mihaiLimit");

        pidfController.setTolerance(1);
    }

    public boolean isReady() {
        return pidfController.targetReached();
    }

    public void setTargetSlot(byte n) {
        // determine if aligning for intake or outtake
        target = mode == Mode.INTAKE ? 0 : uV.ticksPerRevolution / 2;

        target += uV.ticksPerRevolution / 3 * n;

        targetSlot = n;
    }

    public byte getTargetSlot() {
        return targetSlot;
    }

    /*
     * increments target slot by one, clamping it's max value to 2
     * and resetting at 0 when needed
     */
    public void nextSlot() {
        if (targetSlot == 2) setTargetSlot((byte) 0);
        else {
            setTargetSlot((byte) ((targetSlot) + (byte) (1)));
        }
    }

    /*
     * decrements target slot by one, clamping it's min value to 0
     * and resetting at 2 when needed
     */
    public void prevSlot() {
        if (targetSlot == 0) {
            setTargetSlot((byte) 2);
        } else {
            setTargetSlot((byte) ((targetSlot) - (byte) (1)));
        }
    }

    // load game element into turret via lift
    public void liftLoad() {
        lift.setPosition(uV.liftUp);
    }

    // retract lift back to normal
    public void liftReset() {
        lift.setPosition(uV.liftDown);
    }

    public void setSlotColor(byte i, ColorEnum color) {
        colorList[i] = color;
    }

    public ColorEnum getSlotColor(byte i) {
        return colorList[i];
    }

    public byte getFullSlot() {
        for (byte b = 0; b < colorList.length; ++b) {
            if (colorList[b] != ColorEnum.UNDEFINED) {
                return b;
            }
        }

        return -1;
    }

    public boolean isSlotFull(byte targetSlot) {
        return colorList[targetSlot] != ColorEnum.UNDEFINED;
    }

    public byte getFreeSlot() {
        for (byte b = 0; b < colorList.length; ++b) {
            if (colorList[b] == ColorEnum.UNDEFINED) {
                return b;
            }
        }

        return -1;
    }

    public byte getBallCount() {
        byte count = 0;
        for (ColorEnum color : colorList) {
            if (color != ColorEnum.UNDEFINED) {
                count++;
            }
        }
        return count;
    }

    public byte getSlotByColor(ColorEnum color) {
        for (byte b = 0; b < colorList.length; ++b) {
            if (getSlotColor(b) == color) {
                return b;
            }
        }
        return -1;
    }

    public void nextMotif() {
        if (greenPosition == 2) greenPosition = 0;
        else {
            ++greenPosition;
        }
    }

    public void prevMotif() {
        if (greenPosition == 0) {
            greenPosition = 2;
        } else {
            --greenPosition;
        }
    }

    public byte getSlotByMotifPosition(int p) {
        if (p == greenPosition) {
            return getSlotByColor(ColorEnum.GREEN);
        } else {
            return getSlotByColor(ColorEnum.PURPLE);
        }
    }

    public void home() {
        if(!revolverLimit.isPressed()) {
            if (homing) {
                revolverSpin.setPower(0.04);
            }
        }
        else {
            revolverSpin.setPower(0);
            revolverSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            revolverSpin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            homing = false;
        }
    }

    @Override
    public void update() {
        pidfController.kP = Kp;
        pidfController.kI = Ki;
        pidfController.kD = Kd;
        pidfController.kF = Kf;

        if (!homing) {
            pidfController.setSetpoint(target);

            revolverSpin.setPower(uV.revolverPower * pidfController.updatePID(revolverSpin.getCurrentPosition()));
        }

    }
}
