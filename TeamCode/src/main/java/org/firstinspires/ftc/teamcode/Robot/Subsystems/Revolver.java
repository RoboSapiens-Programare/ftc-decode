package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Utils.ColorEnum;
import org.firstinspires.ftc.teamcode.Robot.uV;

@Config
public class Revolver implements Runnable {
    private final CRServo revolverSpin;
    private final Servo lift;
    public DcMotorEx leftFront;

    public ColorEnum[] colorList = {
            ColorEnum.UNDEFINED,
            ColorEnum.UNDEFINED,
            ColorEnum.UNDEFINED
    };

    // target slot and encoder position
    public static byte targetSlot = 0;
    public static int target = 0;

    // PID values for empty revolver
    // NOTE: should implement a 2D array for 4 PID tunes
    // (tuned for 0, 1, 2 and 3 loaded game elements)
    public static double Kp = 0.000285775;
    public static double Kd = 0.000022344;

    // Minimal power so servo does not move
    public static double kms = 0.031;

    // PID
    double lastError = 0;

    ElapsedTime timer = new ElapsedTime();

    // Align with INTAKE / OUTTAKE
    public enum Mode {
        INTAKE,
        OUTTAKE
    };

    public Mode mode = Mode.INTAKE;

    // TODO: implement map of slot position - color

    public int distance;

    public Revolver(HardwareMap hwMap) {
        revolverSpin = hwMap.get(CRServo.class, "revolverSpin");
        revolverSpin.setDirection(CRServo.Direction.FORWARD);
    
        lift = hwMap.get(Servo.class, "lift");
    
        // TODO: replace to dedicated motor
        leftFront = hwMap.get(DcMotorEx.class, "leftFront");
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    @Override
    public void run() {
        // declare only once for memory efficiency
        double encoderPosition, error, derivative, out;

        while (!Thread.currentThread().isInterrupted()) {
            // obtain the encoder position
            encoderPosition = leftFront.getCurrentPosition();

            // calculate the error
            error = target - encoderPosition;

            // rate of change of the error
            derivative = (error - lastError) / timer.seconds();

            // pid formula
            out = (Kp * error) + (Kd * derivative);

            // add kms so the out value is significant for the motor
            if (out <= 0) {
                revolverSpin.setPower(-kms + out);
            } else {
                revolverSpin.setPower(kms + out);
            }


            // reset for next iteration
            lastError = error;
            timer.reset();

        }
    }


    public void setTarget(int target){
        Revolver.target = target;
    }


    public void setTargetSlot(byte n) {
        // determine if aligning for intake or outtake
        target = mode == Mode.INTAKE
                ? uV.revolverPositonIntake0 : uV.revolverPositonOuttake0;

        target += uV.ticksPerRevolution / 3 * (n - 1);


        targetSlot = n;
        setTarget(target);
    }



    public byte getTargetSlot() {
        return targetSlot;
    }

    // load game element into turret via lift
    public void load() {
        lift.setPosition(uV.liftUp);
    }

    // retract lift back to normal
    public void retract() {
        lift.setPosition(uV.uppiesDown);
    }

    /*
     * increments target slot by one, clamping it's max value to 2
     * and resetting at 0 when needed
     */
    public void nextSlot() {

        if (targetSlot == 2)
            setTargetSlot((byte) 0);
        else {
            setTargetSlot((byte) ( (targetSlot) + (byte)(1)));
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
            setTargetSlot((byte) ( (targetSlot) - (byte)(1)));
        }
    }

    public void setSlotColor(byte i, ColorEnum color) {
        colorList[i] = color;
    }

    public ColorEnum getSlotColor(byte i) {
        return  colorList[i];
    }

    public byte getFreeSlot() {
        for (byte b = 0; b < colorList.length; ++b) {
            if (colorList[b] == ColorEnum.UNDEFINED) {
                return b;
            }
        }

        return 5;
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
}
