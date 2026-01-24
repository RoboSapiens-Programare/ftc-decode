package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import java.util.Arrays;
import org.firstinspires.ftc.teamcode.Robot.Utils.ColorEnum;
import org.firstinspires.ftc.teamcode.Robot.Utils.PIDFController;
import org.firstinspires.ftc.teamcode.Robot.uV;

public class Spindexer extends Subsystem {
    public DcMotorEx motor;
    public TouchSensor limitSwitch;

    // TODO: change to actual value
    public static double ticksPerRevolution = 8192;

    public static double Kp = -0.0008;
    public static double Ki = -0.00001;
    public static double Kd = -0.000053;
    public static double Kf = 0;

    public int targetSlot = 0;
    public int begin = 0;
    public double tolerance = 100;
    public double targetPosition = 0;
    public boolean homing = false;
    public boolean homingSingleton = false;

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

        // if using custom PID controller
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // uncomment if using pre-defined PID
        // motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

    public void shoot() {
        slotColors[targetSlot] = ColorEnum.UNDEFINED;

        targetPosition += shootDirection * ticksPerRevolution / 3;

        pidfController.setSetpoint(targetPosition);

        setSlotColor(targetSlot, ColorEnum.UNDEFINED);

        if (--targetSlot == -1) {
            targetSlot = 2;
        }
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

    //    public boolean isSlotFree(int targetSlot) {
    //        return !isSlotFull(targetSlot);
    //    }

    public byte getFreeSlot() {
        for (byte b = 0; b < slotColors.length; ++b) {
            if (slotColors[b] == ColorEnum.UNDEFINED) {
                return b;
            }
        }

        return -1;
    }

    //    public int getFullSlot() {
    //        for (int b = 0; b < slotColors.length; ++b) {
    //            if (slotColors[b] != ColorEnum.UNDEFINED) {
    //                return b;
    //            }
    //        }
    //
    //        return -1;
    //    }

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
    }

    //    public void goToShootStartPose(int slot) {
    //
    //        double distance = abs(targetSlot - slot) * ticksPerRevolution / 3;
    //
    ////        distance = floor(distance-distance/ticksPerRevolution);
    //
    ////        targetPosition += shootDirection * distance + uV.shootOffset;
    //        targetPosition -= shootDirection * distance - uV.shootOffset - ticksPerRevolution/3;
    //
    //        pidfController.setSetpoint(targetPosition);
    //
    //        // REMOVED BLOCKING WHILE LOOP
    //        // Let update() handle the movement - this is now non-blocking
    //        // The position will be reached when isReady() returns true
    //    }

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

        // uncomment if using pre-defined PID
        // motor.setVelocityPIDFCoefficients(Kp, Ki, Kd, Kf);

        // update PID controller
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

        // uncomment if using pre-defined PID
        // motor.setTargetPosition()
    }
}
