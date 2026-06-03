package org.firstinspires.ftc.teamcode.Robot.Utils;

import android.content.res.AssetFileDescriptor;
import com.qualcomm.robotcore.hardware.HardwareMap;

// CHANGE THIS: Use the standard runtime library bundled in the SDK
import org.tensorflow.lite.Interpreter;

import java.io.FileInputStream;
import java.io.IOException;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;

public class ShootAssist {

    private Interpreter tflite;
    private boolean isModelLoaded = false;

    // Pre-allocated memory blocks to prevent loop-time lag from Garbage Collection
    private final float[][] inputArray = new float[1][3];
    private final float[][] outputArray = new float[1][1];

    /**
     * Initializes the TensorFlow Lite interpreter with your trained model.
     */
    public void init(HardwareMap hwMap, String modelName) {
        try {
            MappedByteBuffer modelBuffer = loadModelFile(hwMap, modelName);
            Interpreter.Options options = new Interpreter.Options();

            // Restrict to a single execution thread to preserve CPU cycles for your main thread/drivetrain loops
            options.setNumThreads(1);

            tflite = new Interpreter(modelBuffer, options);
            isModelLoaded = true;
        } catch (Exception e) {
            isModelLoaded = false;
            System.err.println("ShootAssist Error: Could not load TFLite model -> " + e.getMessage());
        }
    }

    /**
     * Predicts the likelihood of making a shot from the robot's current pose.
     */
    public float predictSuccess(double x, double y, double heading) {
        if (!isModelLoaded || tflite == null) {
            return 0.0f;
        }

        // Mutate the pre-allocated array space directly to prevent garbage collection spikes mid-match
        inputArray[0][0] = (float) x;
        inputArray[0][1] = (float) y;
        inputArray[0][2] = (float) heading;

        // Run inference natively using our pre-allocated containers
        tflite.run(inputArray, outputArray);

        // Return the prediction probability matrix value
        return outputArray[0][0];
    }

    /**
     * Safely disposes of native memory handles when the OpMode stops.
     */
    public void close() {
        if (tflite != null) {
            tflite.close();
            isModelLoaded = false;
        }
    }

    public boolean isReady() {
        return isModelLoaded;
    }

    private MappedByteBuffer loadModelFile(HardwareMap hwMap, String modelName) throws IOException {
        AssetFileDescriptor fileDescriptor = hwMap.appContext.getAssets().openFd(modelName);
        FileInputStream inputStream = new FileInputStream(fileDescriptor.getFileDescriptor());
        FileChannel fileChannel = inputStream.getChannel();
        long startOffset = fileDescriptor.getStartOffset();
        long declaredLength = fileDescriptor.getDeclaredLength();
        return fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength);
    }
}