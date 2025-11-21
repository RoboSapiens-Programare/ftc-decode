package org.firstinspires.ftc.teamcode.Auto.Samples;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.List;

//@Disabled
@TeleOp(name = "Concept: Motif Detection", group = "Concept")
public class MotifDetectionSample extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        robot.turret.enableCamera();
    }

    @Override
    public void loop() {
        List<AprilTagDetection> result = robot.turret.tagProcessor.getDetections();

        if (!result.isEmpty()) {
            for (AprilTagDetection tag : result) {
                if (tag.id == 21 || tag.id == 22 || tag.id == 23) {
                    robot.revolver.greenPosition = tag.id - 21;
                }
            }
        }
    }
}