package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.teamcode.Robot.Utils.PIDFController;
import android.util.Size;


import org.firstinspires.ftc.teamcode.Robot.uV;
import java.util.List;

public class Turret {
    public DcMotorEx turretMotor;
    public CRServo turretRotationServo;
    private float turretRotation;

    private final int TAGID = 20;
    public final int frameWidth = 640;
    public double currentPos = 0;
    public boolean tracking = true;

    private AprilTagProcessor tagProcessor;

    // PID values for turret
    // WHEN TUNING USE ZIEGLER-NICHOLS METHOD
    // IT WAS MADE FOR THIS
    // LITERALLY FOR THIS
    public double Kp = 0.000495;
    public double Ki = 0.00099;
    public double Kd = 0.000165;
    public static double Kf = 0.065; // Power to overcome inertia and friction



    public PIDFController pidfController = new PIDFController(Kp, Ki, Kd, Kf);

    private VisionPortal vision;

    public Turret(HardwareMap hwMap) {
        turretMotor = hwMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretRotationServo = hwMap.get(CRServo.class, "turretRotationServo");

        tagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        VisionPortal.Builder vBuilder = new VisionPortal.Builder();

        vBuilder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        vBuilder.addProcessor(tagProcessor);
        vBuilder.setCameraResolution(new Size(frameWidth, 480));
        vBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        vision = vBuilder.build();

        vision.resumeStreaming();

        FtcDashboard.getInstance().startCameraStream(vision, 30);

        pidfController.setSetpoint(frameWidth/2);
        pidfController.setTolerance(0);
    }

    public void setAngle(float rotation) {
        turretRotation = rotation;
        turretRotationServo.setPower(rotation);
    }
    public float getTurretRotation() {
        return turretRotation;
    }
    public void setPower(float power) {
        turretMotor.setPower(power);
    }
    public void startMotor() {
        turretMotor.setPower(uV.outtakePower);
    }

    public void stopMotor() {
        turretMotor.setPower(0);
    }

    public void enableTracking() {
        tracking = true;

        if (vision != null) {
            vision.resumeStreaming();   // restarts the webcam stream
        }
    }

    public void disableTracking() {
        tracking = false;

        if (vision != null) {
            vision.stopStreaming();     // fully stops camera pipeline
        }
    }

    public void toggleTracking() {
        if (tracking) disableTracking();
        else enableTracking();
    }

    public void update() {
        if (!tracking)
            return;

        List<AprilTagDetection> result = tagProcessor.getDetections();

        if (!result.isEmpty()) {
            for (AprilTagDetection tag : result) {
                if (tag.id == TAGID) {
                    // telemetry.addData("TAG OUT", tag.center.x);
                    currentPos = tag.center.x;
                    turretRotationServo.setPower(pidfController.updatePID(tag.center.x));
                }
            }
        } else {
            // no more search fuck off driver 2
        }
    }
}
