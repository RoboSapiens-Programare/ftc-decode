package org.firstinspires.ftc.teamcode.Robot.Utils;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;

@Config
public class PIDController {
    // 1. TUNING VARIABLES (Not static, so you can have multiple PIDs)
    public static double kP;
    public static double kI;
    public static double kD;
    public static double kStatic;

    // 2. STATE VARIABLES
    public static double error = 0;
    private static double previousError = 0;
    private static double integral = 0;
    private static double setpoint = 0;
    private double maxIntegral = 1.0; // Cap for the "I" term

    private ElapsedTime timer = new ElapsedTime();

    public PIDController(double kP, double kI, double kD, double kStatic) {
        PIDController.kP = kP;
        PIDController.kI = kI;
        PIDController.kD = kD;
        PIDController.kStatic = kStatic;
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

        // 4. Apply kStatic
        // This adds the constant force needed to overcome friction

        // Only apply if we are NOT at the target (outside 10 tick tolerance)
        if (Math.abs(error) > 50) {
            if (output > 0) {
                // Moving forward: Add positive static friction
                output += kStatic;
            } else {
                // Moving backward: Subtract static friction (add negative)
                output -= kStatic;
            }
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
}