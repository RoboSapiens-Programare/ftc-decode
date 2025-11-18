package org.firstinspires.ftc.teamcode.Robot.Utils;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;

@Config
public class PIDController implements Runnable {
    public static double kp; // Proportional gain
    public static double ki; // Integral gain
    public static double kd; // Derivative gain
    public static double error = 0, derivative = 0;


    private double integral;
    private double previousError;
    private double setpoint;
    private double currentValue;
    private double output;

    private double outputMin = Double.NEGATIVE_INFINITY;
    private double outputMax = Double.POSITIVE_INFINITY;
    public static double Kmin = 0;

    private ElapsedTime timer1 = new ElapsedTime();
    private ElapsedTime timer2 = new ElapsedTime();


    public PIDController(double kp, double ki, double kd, double Kmin) {
        PIDController.kp = kp;
        PIDController.ki = ki;
        PIDController.kd = kd;
        this.Kmin = Kmin;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void setOutputLimits(double min, double max) {
        this.outputMin = min;
        this.outputMax = max;
    }

    public double update(double currentValue) {
        this.currentValue = currentValue;

        return output;
    }

    public void reset() {
        integral = 0;
        previousError = 0;
        timer1.reset();
    }

    @Override
    public void run() {

        while (!Thread.currentThread().isInterrupted()) {
            // Delay for slowing down thread to calculate derivative
            if (timer1.milliseconds() > 100) {
                derivative = (error - previousError) / timer1.milliseconds();
                timer1.reset();
                previousError = error;

            }

            error = setpoint - currentValue;

            // Integral term

            integral += error * timer2.milliseconds();

            // Derivative term
//P: 0.000042

//            timer1.reset();

            // PID output
            output = kp * error + ki * integral + kd * derivative;

//            output = kp * error + kd * derivative;



            if (output < 0) {
                output -= Kmin;
            } else if (output > 0) {
                output += Kmin;
            }

//            if (Math.abs(error) > 300)
//            {
//                output += Kmin * error / Math.abs(error);
//            }




            // Clamp output
            //output = Math.max(outputMin, Math.min(output, outputMax));

            // Save for next iteration
            timer2.reset();


        }
    }
}