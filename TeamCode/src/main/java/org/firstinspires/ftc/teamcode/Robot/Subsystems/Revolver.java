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
public class Revolver implements Runnable{
    private CRServo revolverSpin;
    private Servo lift;
    public DcMotorEx leftFront;

    private byte targetSlot = 0;

    public static double Kp = 0.00001325;
    public static double Ki = 0;
    public static double Kd = 0.000225;
    public static double kms = 0.031;

    double reference = 0;

    double integralSum = 0;

    double lastError = 0;

    public static double target = 0;

    ElapsedTime timer = new ElapsedTime();

    public void setTarget(double target){
        this.target = target;
    }

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()) {
            //            if (target - 150 < leftFront.getCurrentPosition() && target + 150 > leftFront.getCurrentPosition()) {
            //                continue;
            //            }

            // obtain the encoder position
            double encoderPosition = leftFront.getCurrentPosition();
            // calculate the error
            double error = target - encoderPosition;

            // rate of change of the error
            double derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            if (out <= 0) {
                revolverSpin.setPower(-kms + out);
            } else {
                revolverSpin.setPower(kms + out);
            }


            //            if (Math.abs(out)<0.08)
            //            {
            //                revolverSpin.setPower(0.08 * out/Math.abs(out));
            //            } else
            //            {
            //                revolverSpin.setPower(out);
            //            }


            lastError = error;

            // reset the timer for next time
            timer.reset();

        }
    }


    public enum Mode {
        INTAKE,
        OUTTAKE
    }

    ;

    public Mode mode = Mode.INTAKE;

    public Revolver(HardwareMap hwMap) {
        revolverSpin = hwMap.get(CRServo.class, "revolverSpin");
        revolverSpin.setDirection(CRServo.Direction.FORWARD);
        lift = hwMap.get(Servo.class, "lift");
        leftFront = hwMap.get(DcMotorEx.class, "leftFront");
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public Mode getMode() {
        return mode;
    }

    public void setPosition() {
        switch (mode) {
            case INTAKE:

                break;
            case OUTTAKE:

                break;

        }
    }



    public byte getTargetSlot() {
        return targetSlot;
    }

    public void load() {
        lift.setPosition(uV.liftUp);
    }

    public void retract() {
        lift.setPosition(uV.uppiesDown);
    }

    public void nextSlot() {
        if (targetSlot == 2)
            targetSlot = 0;
        else {
            targetSlot++;
            setTarget(100);
        }
    }

    public void prevSlot() {
        if (targetSlot == 0)
            targetSlot = 2;
        else
            targetSlot--;

    }


}
