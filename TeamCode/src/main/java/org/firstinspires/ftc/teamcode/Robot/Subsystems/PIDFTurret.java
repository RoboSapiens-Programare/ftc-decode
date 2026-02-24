package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDFTurret {
    public double kP;
    public double kI;
    public double kD;
    public double kF;

    public double error = 0;
    private double previousError = 0;
    private double integral = 0;
    private double setpoint = 0;

    public double maxOut = 1;
    public double minOut = -1;
    private double tolerance = 0;

    private ElapsedTime timer = new ElapsedTime();

    public PIDFTurret(double kP, double kI, double kD, double kF) {
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
        double timeChange = timer.seconds();

        // Protecție extremă: prevenim împărțirea la zero în loop-uri super-rapide
        if (timeChange <= 0) return 0;

        error = setpoint - currentValue;
        double derivative = (error - previousError) / timeChange;
        previousError = error;

        // Anti-Windup: Integrăm eroarea doar dacă suntem aproape de țintă (ex: 400 ticks distanță)
        if (Math.abs(error) < 400) {
            integral += error * timeChange;
        } else {
            integral = 0;
        }

        // Limităm termenul Integral la maxim 25% din putere pentru a nu exploda accelerația
        if (Math.abs(integral * kI) > 0.25) {
            integral = Math.signum(integral) * (0.25 / kI);
        }

        // ====================================================================
        // MAGIA FLYWHEEL-ULUI: Feedforward aplicat pe SETPOINT (Țintă)!
        // Oferă motorului de 6000 RPM curentul constant necesar pentru a-și menține
        // turația, chiar și când eroarea este 0.
        // ====================================================================
        double feedforward = kF * setpoint;

        // Suma puterilor: Proporțional + Integral + Derivativ + Forța de bază (F)
        double output = (kP * error) + (kI * integral) + (kD * derivative) + feedforward;

        timer.reset();

        // Returnăm valoarea tăiată la limitele fizice ale Control Hub-ului (-1.0 la 1.0)
        return Range.clip(output, minOut, maxOut);
    }

    public void reset() {
        integral = 0;
        previousError = 0;
        timer.reset();
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public boolean targetReached() {
        return Math.abs(error) < tolerance;
    }
}