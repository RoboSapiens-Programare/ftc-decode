package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.uV;
import org.firstinspires.ftc.teamcode.TeleOP.FullFSMIHope;
@Config
public class Revolver implements Runnable {
    private CRServo revolverSpin;
    private Servo lift;
    public DcMotorEx leftFront;

    // target slot and encoder position
    private byte targetSlot = 0;
    public static double target = 0;

    // PID values for empty revolver
    // NOTE: should implement a 2D array for 4 PID tunes
    // (tuned for 0, 1, 2 and 3 loaded game elements)
    public static double Kp = 0.00001325;
    public static double Ki = 0;
    public static double Kd = 0.000225;

    // Minimal power so servo does not move
    public static double kms = 0.031;

    // PID
    double reference = 0;
    double integralSum = 0;
    double lastError = 0;

    ElapsedTime timer = new ElapsedTime();

    // Align with INTAKE / OUTTAKE
    public enum Mode {
        INTAKE,
        OUTTAKE
    };

    public Mode mode = Mode.INTAKE;

    // TODO: implement map of slot position - color


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

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            // pid formula
            out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

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


    public void setTarget(double target){
        Revolver.target = target;
    }

    
    /*
    * choose fastest way there (hope it works) 
    */
    public void setTargetSlot(byte n) {
        int distanceToWalk = targetSlot - n;

        // determine if aligning for intake or outtake
        target = mode == Mode.INTAKE ? uV.revolverPositonIntake0 : uV.revolverPositonOuttake0;

        // determine fastest way there
        if (Math.abs(distanceToWalk) == 1) {
            target += uV.ticksPerRevolution / 3 * distanceToWalk;
        }

        // fastest way for 2 slots in a 3 slot system is just one step in oposite direction
        else if(Math.abs(distanceToWalk) == 2) {
            target -= uV.ticksPerRevolution / 3 * distanceToWalk > 0 ? 1 : -1;
        }


        n = targetSlot;
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
            targetSlot = 0;
        else {
            targetSlot++;
            setTarget(100);
        }
    }

    /*
     * decrements target slot by one, clamping it's min value to 0
     * and resetting at 2 when needed
     */
    public void prevSlot() {
        if (targetSlot == 0)
            targetSlot = 2;
        else
            targetSlot--;

    }


}
