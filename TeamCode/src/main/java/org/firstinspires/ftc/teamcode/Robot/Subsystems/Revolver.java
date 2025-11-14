package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Utils.ColorEnum;
import org.firstinspires.ftc.teamcode.Robot.Utils.PIDController;
import org.firstinspires.ftc.teamcode.Robot.uV;

@Config
public class Revolver{
    private final CRServo revolverSpin;
    private final Servo lift;
    public DcMotorEx encoderRevolver;
    public Thread t;

    public Revolver(HardwareMap hwMap) {
        encoderRevolver = hwMap.get(DcMotorEx.class, "encoderRevolver");
        encoderRevolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        revolverSpin = hwMap.get(CRServo.class, "revolverSpin");
        revolverSpin.setDirection(CRServo.Direction.REVERSE);

        lift = hwMap.get(Servo.class, "lift");
    }

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
    public static double Kmin = 0.031;

    private PIDController pidController = new PIDController(Kp, 0, Kd, Kmin);

    // PID
    double lastError = 0;

    ElapsedTime timer = new ElapsedTime();

    // Align with INTAKE / OUTTAKE
    public enum Mode {
        INTAKE,
        OUTTAKE
    };

    public Mode mode = Mode.INTAKE;

    public int distance;

    public void setTarget(int target){
        Revolver.target = target;

        pidController.setSetpoint(target);
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
    public void liftLoad() {
        lift.setPosition(uV.liftUp);
    }

    // retract lift back to normal
    public void liftReset() {
        lift.setPosition(uV.liftDown);
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

    public void update() {
        pidController.update(encoderRevolver.getCurrentPosition());
    }

    public void start() {
        t = new Thread(pidController);
        t.start();
    }
}
