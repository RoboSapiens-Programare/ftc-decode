package org.firstinspires.ftc.teamcode.Robot.Utils;

import android.os.Environment;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class NNLogging {
    private BufferedWriter writer;
    private VoltageSensor batterySensor;
    private boolean isReady = false;
    private int totalBallsShotCounter = 0;

    private static final double GOAL_X = 12;
    private static final double GOAL_Y = 134;

    // Inner class to cache the exact conditions when the trigger was pulled
    public static class ShotSnapshot {
        public double x, y, heading;
        public double velX, velY, omega;
        public double voltage;
        public boolean isValid = false;
    }

    private final ShotSnapshot pendingSnapshot = new ShotSnapshot();

    public void init(HardwareMap hwMap) {
        try {
            batterySensor = hwMap.voltageSensor.iterator().next();
        } catch (Exception e) {
            System.err.println("NNLogging Warning: No voltage sensor detected.");
        }

        try {
            File directory = new File(Environment.getExternalStorageDirectory(), "FIRST/data");
            if (!directory.exists()) {
                directory.mkdirs();
            }

            File file = new File(directory, "ftc_shot_data.csv");
            boolean fileExists = file.exists();

            writer = new BufferedWriter(new FileWriter(file, true));

            if (!fileExists) {
                writer.write("target_dist,angle_error,vel_x,vel_y,omega,voltage,balls_scored\n");
                writer.flush();
            }

            isReady = true;
        } catch (IOException e) {
            System.err.println("NNLogging Error: Failed to open CSV file -> " + e.getMessage());
            isReady = false;
        }
    }

    /**
     * Captures the physical parameters of the robot at the exact moment of launch. Call this when
     * your outtake mechanism physically launches the balls.
     */
    public void takeSnapshot(
            double robotX, double robotY, double heading, double velX, double velY, double omega) {
        if (!isReady) return;

        pendingSnapshot.x = robotX;
        pendingSnapshot.y = robotY;
        pendingSnapshot.heading = heading;
        pendingSnapshot.velX = velX;
        pendingSnapshot.velY = velY;
        pendingSnapshot.omega = omega;
        pendingSnapshot.voltage = (batterySensor != null) ? batterySensor.getVoltage() : 0.0;
        pendingSnapshot.isValid = true;
    }

    /**
     * Permanently writes the cached snapshot to disk along with the human-observed score. Call this
     * when you press the D-pad to input the results.
     */
    public void commitSnapshot(int ballsScored) {
        if (!isReady || writer == null || !pendingSnapshot.isValid) return;

        // 1. Compute spatial vectors from the cached historical coordinates
        double deltaX = GOAL_X - pendingSnapshot.x;
        double deltaY = GOAL_Y - pendingSnapshot.y;
        double targetDist = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        double absoluteAngleToGoal = Math.atan2(deltaY, deltaX);
        double angleError = absoluteAngleToGoal - pendingSnapshot.heading;

        while (angleError > Math.PI) angleError -= 2 * Math.PI;
        while (angleError < -Math.PI) angleError += 2 * Math.PI;

        totalBallsShotCounter += ballsScored;

        try {
            String line =
                    String.format(
                            "%.3f,%.4f,%.3f,%.3f,%.3f,%.2f,%d\n",
                            targetDist,
                            angleError,
                            pendingSnapshot.velX,
                            pendingSnapshot.velY,
                            pendingSnapshot.omega,
                            pendingSnapshot.voltage,
                            ballsScored);
            writer.write(line);
            writer.flush();

            // Invalidate snapshot until next shot is physically taken
            pendingSnapshot.isValid = false;
        } catch (IOException e) {
            System.err.println("NNLogging Error: Failed writing line -> " + e.getMessage());
        }
    }

    public boolean hasPendingSnapshot() {
        return pendingSnapshot.isValid;
    }

    public int getTotalBallsScored() {
        return totalBallsShotCounter;
    }

    public void close() {
        if (writer != null) {
            try {
                writer.flush();
                writer.close();
                isReady = false;
            } catch (IOException e) {
                // Fail silently
            }
        }
    }
}
