package org.firstinspires.ftc.teamcode.Robot.Utils;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;
import org.tensorflow.lite.Interpreter;

public class ShootAssist {

    private Interpreter tflite;
    private boolean isModelLoaded = false;

    // Pre-allocated memory containers to eliminate garbage collection overhead in iterative loops
    private final float[][] inputArray = new float[1][6];

    // CRITICAL UPDATE: Changed array dimensions to match your new 4-class Softmax layer [0, 1, 2, 3
    // balls]
    private final float[][] outputArray = new float[1][4];

    // HARDCODED SCALING CONSTANTS (Ensure these match your newly generated balanced-dataset
    // printouts!)
    private final double[] MEANS =
            new double[] {
                2.503, // target_dist mean
                0.001, // angle_error mean
                3.512, // vel_x mean
                -0.004, // vel_y mean
                0.015, // omega mean
                12.485 // voltage mean
            };

    private final double[] STDS =
            new double[] {
                0.871, // target_dist std dev
                0.235, // angle_error std dev
                0.865, // vel_x std dev
                0.582, // vel_y std dev
                1.164, // omega std dev
                0.579 // voltage std dev
            };

    public void init(String modelName) {
        try {
            MappedByteBuffer modelBuffer = loadModelFile(modelName);
            Interpreter.Options options = new Interpreter.Options();

            // Single execution thread to keep Control Hub cycle times optimized
            options.setNumThreads(1);

            tflite = new Interpreter(modelBuffer, options);
            isModelLoaded = true;
        } catch (Exception e) {
            isModelLoaded = false;
            System.err.println(
                    "ShootAssist Error: Could not load TFLite model -> " + e.getMessage());
        }
    }

    /**
     * Runs inference and returns the full confidence array from the model.
     *
     * @return A float array containing confidence values for index [0]=0 balls, [1]=1 ball, [2]=2
     *     balls, [3]=3 balls.
     */
    public float[] predictProbabilities(
            double targetDist,
            double angleError,
            double velX,
            double velY,
            double omega,
            double voltage) {
        if (!isModelLoaded || tflite == null) {
            return new float[] {0.0f, 0.0f, 0.0f, 0.0f};
        }

        // Apply standard scaler transformation
        inputArray[0][0] = (float) ((targetDist - MEANS[0]) / STDS[0]);
        inputArray[0][1] = (float) ((angleError - MEANS[1]) / STDS[1]);
        inputArray[0][2] = (float) ((velX - MEANS[2]) / STDS[2]);
        inputArray[0][3] = (float) ((velY - MEANS[3]) / STDS[3]);
        inputArray[0][4] = (float) ((omega - MEANS[4]) / STDS[4]);
        inputArray[0][5] = (float) ((voltage - MEANS[5]) / STDS[5]);

        // Run inference
        tflite.run(inputArray, outputArray);

        // Return the copy of internal pre-allocated array to keep the original safe from external
        // mutations
        return outputArray[0].clone();
    }

    /**
     * Resolves the highest probability index, exactly mirroring Python's np.argmax().
     *
     * @return An integer representing the most likely outcome category (0, 1, 2, or 3 scored
     *     balls).
     */
    public int predictBallCount(
            double targetDist,
            double angleError,
            double velX,
            double velY,
            double omega,
            double voltage) {
        float[] probabilities =
                predictProbabilities(targetDist, angleError, velX, velY, omega, voltage);

        int bestClass = 0;
        float maxProb = probabilities[0];

        for (int i = 1; i < probabilities.length; i++) {
            if (probabilities[i] > maxProb) {
                maxProb = probabilities[i];
                bestClass = i;
            }
        }
        return bestClass;
    }

    /**
     * Calculates the statistical "expected value" of the shot. Useful for automated cross-field
     * alignment optimization!
     *
     * @return A smooth calculated float from 0.0 to 3.0 representing the weighted average of
     *     expected balls.
     */
    public float predictExpectedBalls(
            double targetDist,
            double angleError,
            double velX,
            double velY,
            double omega,
            double voltage) {
        float[] probabilities =
                predictProbabilities(targetDist, angleError, velX, velY, omega, voltage);

        // Expected value formula: E[X] = x_0*P(x_0) + x_1*P(x_1) + ...
        return (0f * probabilities[0])
                + (1f * probabilities[1])
                + (2f * probabilities[2])
                + (3f * probabilities[3]);
    }

    public void close() {
        if (tflite != null) {
            tflite.close();
            isModelLoaded = false;
        }
    }

    public boolean isReady() {
        return isModelLoaded;
    }

    private MappedByteBuffer loadModelFile(String modelName) throws IOException {
        File modelFile = new File("/sdcard/FIRST/" + modelName);

        if (!modelFile.exists()) {
            throw new IOException(
                    "Could not find TFLite model at: "
                            + modelFile.getAbsolutePath()
                            + ". Did you forget to run 'adb push'?");
        }

        try (FileInputStream inputStream = new FileInputStream(modelFile);
                FileChannel fileChannel = inputStream.getChannel()) {
            return fileChannel.map(FileChannel.MapMode.READ_ONLY, 0, modelFile.length());
        }
    }

    public void debugModelLoading() {
        if (!isModelLoaded || tflite == null) {
            System.err.println("❌ DIAGNOSTIC: Model is NOT loaded in memory!");
            return;
        }

        System.out.println("🤖 --- TFLITE DIAGNOSTIC RUN ---");

        // Test Case: Perfect alignment, zero velocity, optimal voltage.
        // This profile should naturally output a high probability for 3 balls.
        inputArray[0][0] = 0.0f; // Target distance (Mean-scaled)
        inputArray[0][1] = 0.0f; // Angle error (Mean-scaled)
        inputArray[0][2] = 0.0f; // vel_x (Mean-scaled)
        inputArray[0][3] = 0.0f; // vel_y (Mean-scaled)
        inputArray[0][4] = 0.0f; // omega (Mean-scaled)
        inputArray[0][5] = 0.0f; // voltage (Mean-scaled)

        try {
            // Clear previous outputs
            for (int i = 0; i < 4; i++) outputArray[0][i] = -1.0f;

            // Force native execution
            tflite.run(inputArray, outputArray);

            System.out.println("✅ Native execution successful!");
            System.out.printf(
                    "Raw Softmax Probabilities: [0b: %.4f, 1b: %.4f, 2b: %.4f, 3b: %.4f]\n",
                    outputArray[0][0], outputArray[0][1], outputArray[0][2], outputArray[0][3]);

        } catch (Exception e) {
            System.err.println("❌ CRASH DURING INFERENCE: " + e.getMessage());
            e.printStackTrace();
        }
        System.out.println("--------------------------------");
    }
}
