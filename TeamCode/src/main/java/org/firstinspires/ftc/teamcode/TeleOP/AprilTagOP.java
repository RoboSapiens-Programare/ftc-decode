package org.firstinspires.ftc.teamcode.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;

import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

import kotlin.text.MatchGroup;

@Configurable
@Config
@TeleOp(name = "AprilTagOp", group = "TeleOp")
public class AprilTagOP extends OpMode {

//    private VisionPortal visionPortal;
    private OpenCvWebcam camera;
    private AprilTagDetectionPipeline aprilTagDetectionPipeline;
    private CRServo pivotMotor;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();

    //PID Variables
    public static double Kp = 0.0087;
    public static double Ki = 0.00000000002;
    public static double Kd = 0.000005;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int direction = 1;
    double center_x = 160;

    double integralSum = 0;

    double lastError = 0;

    public boolean tagFound = false;

    ElapsedTime timer = new ElapsedTime();


    @Override
    public void init() {


        pivotMotor = hardwareMap.get(CRServo.class, "CamPivot");

        // visionPortal = new VisionPortal.Builder()
        //         .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
        //         .addProcessor(aprilTag)
        //         .enableLiveView(true)
        //         .setCameraResolution(new Size(640, 480))
        //         .setAutoStartStreamOnBuild(true)
        //         .setShowStatsOverlay(true)
        //         .build();


        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(camera, 30);

        camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        telemetry.addLine("AprilTag Op Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        moveToTag();
    }

    public void moveToTag() {
        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getLatestDetections();

        tagFound = false;

        AprilTagDetection tag = null;

        if (!detections.isEmpty()) {
            // In this case it goes to the first april tag detected
            // TODO: sort by tag ids corresponding to goals

            // testing tag array logic

            for (int i = 0; i < detections.size(); i++) {
                tag = detections.get(i);

                if (tag.id == 6) {
                    tagFound = true;
                    break;
                } else {
                    pivotMotor.setPower(0);
                }
            }
        }



        if(tagFound) {
            direction *= -1;

            double pos_x = tag.center.x;
            // calculate the error
            double error = -center_x + pos_x;
            if (Math.abs(error) > 20*Kp) {
                // rate of change of the error
                double derivative = (error - lastError) / timer.seconds();

                // sum of all error over time
                integralSum = integralSum + (error * timer.seconds());



                double out = ((Kp * Math.log(Math.abs(error)+1) * error / Math.abs(error)) + (Ki * integralSum) + (Kd * derivative))*1;

                pivotMotor.setPower(out);

                lastError = error;

                // reset the timer for next time
                timer.reset();
                telemetry.addData("Error: ", error);
                telemetry.addData("Error tiganie: ", Kp * 0);
                telemetry.addData("power to motor", out);
                telemetry.update();
            }
            else pivotMotor.setPower(0);
        } else {
            pivotMotor.setPower(0.05 * direction);
        }
        spewTelemetry();
    }

    public void spewTelemetry() {
        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getLatestDetections();

        if (detections.isEmpty()) {
            telemetry.addLine("No tags detected");
        } else {
            for (AprilTagDetection tag : detections) {
                Orientation rot = Orientation.getOrientation(tag.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("Center", "(%.2f, %.2f)", tag.center.x, tag.center.y);
                telemetry.addData("Pose (X,Y,Z)", "(%.2f, %.2f, %.2f)",
                        tag.pose.x, tag.pose.y, tag.pose.z  );
                telemetry.addData("Yaw/Pitch/Roll", "(%.2f, %.2f, %.2f)",
                        rot.firstAngle, rot.secondAngle, rot.thirdAngle);
                telemetry.addLine("------");
            }
        }
        telemetry.update();
    }
}
