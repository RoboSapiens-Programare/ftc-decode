package org.firstinspires.ftc.teamcode.Robot.Utils;

// import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDFController {
    // 1. TUNING VARIABLES (Not static, so you can have multiple PIDs)
    public double kP;
    public double kI;
    public double kD;
    public double kF;

    // 2. STATE VARIABLES
    public double error = 0;
    private double previousError = 0;
    private double integral = 0;
    private double setpoint = 0;
    private double maxIntegral = 1.0; // Cap for the "I" term
    private int tolerance = 0;

    private ElapsedTime timer = new ElapsedTime();

    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        timer.reset();
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double updatePID(double currentValue) {
        // Calculate time difference in SECONDS
        double timeChange = timer.seconds();

        // Calculate Error
        error = setpoint - currentValue;

        // Calculate Derivative
        double derivative = (error - previousError) / timeChange;
        previousError = error;

        // Only accumulate if within 400 ticks to prevent massive windup
        if (Math.abs(error) < 400) {
            integral += error * timeChange;
        } else {
            integral = 0;
        }

        // Clamp Integral contribution to 25% max
        if (Math.abs(integral * kI) > 0.25) {
            integral = Math.signum(integral) * (0.25 / kI);
        }

        // Calculate PID Output
        double output = (kP * error) + (kI * integral) + (kD * derivative);

        // 4. Apply kF
        // This adds the constant force needed to overcome friction

        // Only apply if we are NOT at the target (outside 55 ticks tolerance)
        if (Math.abs(error) > tolerance) {
            output += kF * Math.signum(error);
        } else {
            // If inside tolerance, shut off everything to prevent jitter
            output = 0;
            integral = 0;
        }

        timer.reset();

        // 5. Clip result to motor limits
        return Range.clip(output, -1.0, 1.0);
    }

    public void reset() {
        integral = 0;
        previousError = 0;
        timer.reset();
    }

    public void setTolerance(int tolerance) {
        this.tolerance = tolerance;
    }

    public boolean targetReached() {
        return Math.abs(error) < tolerance;
    }
}
