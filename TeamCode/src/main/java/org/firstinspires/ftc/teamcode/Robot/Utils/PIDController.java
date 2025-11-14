package org.firstinspires.ftc.teamcode.Robot.Utils;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class PIDController implements Runnable {
    public static double kp; // Proportional gain
    public static double ki; // Integral gain
    public static double kd; // Derivative gain
    double error = 0, derivative = 0;


    private double integral;
    private double previousError;
    private double setpoint;
    private double currentValue;
    private double output;

    private double outputMin = Double.NEGATIVE_INFINITY;
    private double outputMax = Double.POSITIVE_INFINITY;
    public static double Kmin = 0;

    private ElapsedTime timer = new ElapsedTime();


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
        timer.reset();
    }

    @Override
    public void run() {

        while (!Thread.currentThread().isInterrupted()) {
            // Delay for slowing down thread to calculate derivative
            if (timer.milliseconds() > 150) {
                derivative = (error - previousError) / timer.milliseconds();
                timer.reset();
            }

            error = setpoint - currentValue;

            // Integral term
            integral += error * timer.milliseconds();

            // Derivative term


            // PID output
            //output = kp * error + ki * integral + kd * derivative;

            output = kp * error + kd * derivative;

            if (output <= 0) {
                output -= Kmin;
            } else {
                output += Kmin;
            }


            // Clamp output
            //output = Math.max(outputMin, Math.min(output, outputMax));

            // Save for next iteration
            previousError = error;

            timer.reset();

        }
    }
}