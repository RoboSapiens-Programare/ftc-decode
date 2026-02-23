package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import java.util.Arrays;
import org.firstinspires.ftc.teamcode.Robot.Utils.ColorEnum;
import org.firstinspires.ftc.teamcode.Robot.Utils.PIDFController;
import org.firstinspires.ftc.teamcode.Robot.uV;

@Config
public class Spindexer extends Subsystem {
    public DcMotorEx motor;
    public TouchSensor limitSwitch;

    public static double ticksPerRevolution = 8192;

    public static double Kp = -0.0009;
    public static double Ki = -0.001;
    public static double Kd = -0.000043;
    public static double Kf = 0;

    public int targetSlot = 0;
    public int begin = 0;
    public double tolerance = 75;
    public double targetPosition = 0;
    public boolean homing = false;
    public boolean homingSingleton = false;

    private boolean wentToStart = false;

    private final PIDFController pidfController = new PIDFController(Kp, Ki, Kd, Kf);

    public double shootDirection = -1;

    /* greenMotifPosition means the position [0, 1, 2] of the green ball in the motif
     * 0 -> GPP
     * 1 -> PPG
     * 2 -> PGP
     */
    public static int greenMotifPosition = 0;

    public ColorEnum[] slotColors = {ColorEnum.UNDEFINED, ColorEnum.UNDEFINED, ColorEnum.UNDEFINED};

    public Spindexer(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, "spindexer");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidfController.setTolerance(tolerance);

        limitSwitch = hwMap.get(TouchSensor.class, "spindexerLimitSwitch");
    }

    // Moving functions
    public void goToSlot(int slot) {
        double distance = abs(targetSlot - slot) * ticksPerRevolution / 3;

        targetPosition += -1 * shootDirection * distance;

        targetSlot = slot;

        pidfController.setSetpoint(targetPosition);
    }

    public void shoot(int n) {
        if (n == -1) {
            n = getBallCount();
        }

        if (wentToStart) {
            wentToStart = false;
            targetPosition -= uV.shootOffset;
            pidfController.setSetpoint(targetPosition);

            while (!pidfController.targetReached()) {
                motor.setPower(pidfController.updatePID(motor.getCurrentPosition()));
            }
        }

        for (int i = 0; i < n; ++i) {
            slotColors[targetSlot] = ColorEnum.UNDEFINED;

            targetPosition += shootDirection * ticksPerRevolution / 3;

            setSlotColor(targetSlot, ColorEnum.UNDEFINED);

            if (--targetSlot == -1) {
                targetSlot = 2;
            }
        }

        pidfController.setSetpoint(targetPosition);
    }

    public void shoot() {
        shoot(1);
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

    public byte getFreeSlot() {
        for (byte b = 0; b < slotColors.length; ++b) {
            if (slotColors[b] == ColorEnum.UNDEFINED) {
                return b;
            }
        }

        return -1;
    }

    public int getBallCount() {
        int count = 0;
        for (ColorEnum slotColor : slotColors) {
            if (slotColor != ColorEnum.UNDEFINED) {
                ++count;
            }
        }

        return count;
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

        begin = greenSlot + greenMotifPosition - getSlotByColor(ColorEnum.GREEN) + 2;

        if (greenSlot == 0) {
            begin--;
        } else if (greenSlot == 1) {
            begin++;
        }

        if (begin >= 3) {
            begin = begin - 3;
        }

        goToSlot(begin);
        targetPosition += uV.shootOffset;
        pidfController.setSetpoint(targetPosition);

        wentToStart = true;
    }

    // system-status functions

    public boolean isReady() {
        return pidfController.targetReached() && !homing;
    }

    public int getTargetSlot() {
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
        pidfController.kP = Kp;
        pidfController.kI = Ki;
        pidfController.kD = Kd;
        pidfController.kF = Kf;

        if (homing) {
            motor.setPower(-0.2);

            if (limitSwitch.isPressed()) {
                targetSlot = 0;
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                targetPosition = 8192.0 / 12 + uV.homingOffset;
                pidfController.setSetpoint(targetPosition);

                while (!pidfController.targetReached()) {
                    motor.setPower(pidfController.updatePID(motor.getCurrentPosition()));
                }

                targetPosition = uV.homingOffset;
                pidfController.setSetpoint(targetPosition);

                homing = false;
                homingSingleton = true;
            }
        } else {
            pidfController.setSetpoint(targetPosition);
            double pidOut = pidfController.updatePID(motor.getCurrentPosition());
            motor.setPower(pidOut * uV.revolverPowerMultiplier);
        }

        FtcDashboard.getInstance()
                .getTelemetry()
                .addData("green slot", getSlotByColor(ColorEnum.GREEN));
        FtcDashboard.getInstance().getTelemetry().addData("motif pos", greenMotifPosition);
        FtcDashboard.getInstance().getTelemetry().update();
    }
}
