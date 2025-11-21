package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Utils.ColorEnum;
import org.firstinspires.ftc.teamcode.Robot.Utils.PIDFController;
import org.firstinspires.ftc.teamcode.Robot.uV;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@Config
public class Revolver{
    private final CRServo revolverSpin;
    private final Servo lift;
    public DcMotor encoderRevolver;
    public Thread t;
    public int greenPosition = 0;

    public ColorEnum[] colorList = {
            ColorEnum.UNDEFINED,
            ColorEnum.UNDEFINED,
            ColorEnum.UNDEFINED
    };


    // target slot and encoder position
    public static byte targetSlot = 0;
    public static int target = 0;


    // PID values for revolver
    // WHEN TUNING USE ZIEGLER-NICHOLS METHOD
    // IT WAS MADE FOR THIS
    // LITERALLY FOR THIS
    public static double Kp = 0.0001625;
    public static double Kd = 0.0000375;
    public static double Ki = 0.000265;
    public static double Kf = 0.055; // Power to overcome inertia and friction

    private PIDFController pidfController = new PIDFController(Kp, Ki, Kd, Kf);

    // Align with INTAKE / OUTTAKE
    public enum Mode {
        INTAKE,
        OUTTAKE
    };

    public Mode mode = Mode.INTAKE;

    public Revolver(HardwareMap hwMap) {
        encoderRevolver = hwMap.get(DcMotor.class, "encoderRevolver");
        encoderRevolver.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderRevolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        revolverSpin = hwMap.get(CRServo.class, "revolverSpin");
        revolverSpin.setDirection(CRServo.Direction.REVERSE);

        lift = hwMap.get(Servo.class, "lift");

        pidfController.setTolerance(25);
    }

    public void setTarget(int target){
        Revolver.target = target;
    }


    public void setTargetSlot(byte n) {
        // determine if aligning for intake or outtake
        target = mode == Mode.INTAKE ? uV.revolverPositonIntake0 : uV.revolverPositonOuttake0;


        target += uV.ticksPerRevolution / 3 * n;
        //TODO SEE IF IT WORKS
        if(target == 5460) target = -2730;


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

    public byte getFullSlot(){
        for (byte b = 0; b < colorList.length; ++b) {
            if (colorList[b] != ColorEnum.UNDEFINED) {
                return b;
            }
        }

        return -1;
    }

    public boolean isSlotFull(byte targetSlot){
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

    public void update() {
        pidfController.setSetpoint(target);

        double out = pidfController.updatePID(encoderRevolver.getCurrentPosition());

        revolverSpin.setPower(out);

        // FtcDashboard.getInstance().getTelemetry().addData("out", out);
        // FtcDashboard.getInstance().getTelemetry().addData("err", pidfController.error);
        // FtcDashboard.getInstance().getTelemetry().update();
    }

    public void start() {
    }

    public void nextMotif() {
        if (greenPosition == 2)
            greenPosition = 0;
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

    public byte getSlotByColor(ColorEnum color) {
        for (byte b = 0; b < colorList.length; ++b) {
            if (colorList[b] == color) {
                return b;
            }
        }
        return -1;
    }

    public byte getSlotByMotifPosition(int p) {
        if (p == greenPosition) {
            return getSlotByColor(ColorEnum.GREEN);
        } else {
            return getSlotByColor(ColorEnum.PURPLE);
        }
    }
}
