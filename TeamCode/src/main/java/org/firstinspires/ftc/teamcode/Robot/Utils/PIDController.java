package org.firstinspires.ftc.teamcode.Robot.Utils;

public class PIDController {
    private double kp;  // Proportional gain
    private double ki;  // Integral gain
    private double kd;  // Derivative gain

    private double integral;
    private double previousError;
    private double setpoint;

    private double outputMin = Double.NEGATIVE_INFINITY;
    private double outputMax = Double.POSITIVE_INFINITY;

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void setOutputLimits(double min, double max) {
        this.outputMin = min;
        this.outputMax = max;
    }

    public double update(double currentValue, double deltaTime) {
        double error = setpoint - currentValue;

        // Integral term
        integral += error * deltaTime;

        // Derivative term
        double derivative = (error - previousError) / deltaTime;

        // PID output
        double output = kp * error + ki * integral + kd * derivative;

        // Clamp output
        output = Math.max(outputMin, Math.min(output, outputMax));

        // Save for next iteration
        previousError = error;

        return output;
    }

    public void reset() {
        integral = 0;
        previousError = 0;
    }
}
