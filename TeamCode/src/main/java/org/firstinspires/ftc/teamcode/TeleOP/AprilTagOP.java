package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "AprilTagOp", group = "TeleOp")
public class AprilTagOP extends OpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private DcMotorEx pivotMotor;


    @Override
    public void init() {
        pivotMotor = hardwareMap.get(DcMotorEx.class, "CamPivot");
        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("AprilTag Op Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        moveToTag();
    }

    public void moveToTag()
    {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        double CENTER_X = 320;
        double tolerance = 20;

        if (!detections.isEmpty()) {
            // In this case it goes to the first april tag detected
            // TODO: sort by tag ids corresponding to goals
            AprilTagDetection tag = detections.get(0);
            double error = tag.center.x - CENTER_X;

            // TODO: check this crap logic that makes no sense
            if (Math.abs(error) > tolerance) {

                double power = error / CENTER_X;
                power = Math.max(-0.5, Math.min(0.5, power));

                pivotMotor.setPower(power);
            } else {
                pivotMotor.setPower(0);
            }
        } else {
            pivotMotor.setPower(0);
        }

    }

    public void spewTelemetry()
    {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        if (detections.isEmpty()) {
            telemetry.addLine("No tags detected");
        } else {
            for (AprilTagDetection tag : detections) {
                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("Center", "(%.2f, %.2f)", tag.center.x, tag.center.y);
                telemetry.addData("Pose (X,Y,Z)", "(%.2f, %.2f, %.2f)",
                        tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z);
                telemetry.addData("Yaw/Pitch/Roll", "(%.2f, %.2f, %.2f)",
                        tag.ftcPose.yaw, tag.ftcPose.pitch, tag.ftcPose.roll);
                telemetry.addLine("------");
            }
        }
        telemetry.update();
    }
}
