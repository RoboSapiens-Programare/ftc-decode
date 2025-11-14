package org.firstinspires.ftc.teamcode.Robot.Utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController implements Runnable {
    private double kp; // Proportional gain
    private double ki; // Integral gain
    private double kd; // Derivative gain

    private double integral;
    private double previousError;
    private double setpoint;
    private double currentValue;
    private double output;

    private double outputMin = Double.NEGATIVE_INFINITY;
    private double outputMax = Double.POSITIVE_INFINITY;
    private double relevanceValue = 0;

    private ElapsedTime timer = new ElapsedTime();

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public PIDController(double kp, double ki, double kd, double relevanceValue) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.relevanceValue = relevanceValue;
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
        double error, derivative;

        while (!Thread.currentThread().isInterrupted()) {
            error = setpoint - currentValue;

            // Integral term
            integral += error * timer.milliseconds();

            // Derivative term
            derivative = (error - previousError) / timer.milliseconds();

            // PID output
            output = kp * error + ki * integral + kd * derivative;

            if (output <= 0) {
                output -= relevanceValue;
            } else {
                output += relevanceValue;
            }


            // Clamp output
            output = Math.max(outputMin, Math.min(output, outputMax));

            // Save for next iteration
            previousError = error;

            timer.reset();

        }
    }
}