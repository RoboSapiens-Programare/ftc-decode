package org.firstinspires.ftc.teamcode.Robot.Subsystems;

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
    private final int frameWidth = 640;
    public boolean tracking = true;

    private AprilTagProcessor tagProcessor;

    // PID values for turret
    // WHEN TUNING USE ZIEGLER-NICHOLS METHOD
    // IT WAS MADE FOR THIS
    // LITERALLY FOR THIS
    public static double Kp = 0.0;
    public static double Kd = 0.0;
    public static double Ki = 0.0;
    public static double Kf = 0.0; // Power to overcome inertia and friction

    private PIDFController pidfController = new PIDFController(Kp, Ki, Kd, Kf);

    private VisionPortal vision;

    private ElapsedTime searchTimer = new ElapsedTime();
    private int searchSign = 1;
    private int searchStep = 0;

    public Turret(HardwareMap hwMap) {
        turretMotor = hwMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretRotationServo = hwMap.get(CRServo.class, "turretRotationServo");

        tagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        VisionPortal.Builder vBuilder = new VisionPortal.Builder();

        vBuilder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        vBuilder.addProcessor(tagProcessor);
        vBuilder.setCameraResolution(new Size(frameWidth, 720));
        vBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        vision = vBuilder.build();

        vision.resumeStreaming();

        pidfController.setSetpoint(frameWidth/2);
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
                    turretMotor.setPower(pidfController.updatePID(tag.center.x));
                    searchStep = 0;
                }
            }
        } else {
            // motor at almost 60RPM
            // 5:1 gear ratio
            // 12 RPM on turret
            // 5 sec per full rotation
            // i want to go just...90 deg in each direction
            // results in 90 * 5 / 360 = 5/4
            // needed 180 after first iter

            if (searchStep == 0) {
                ++searchStep;
                searchTimer.reset();
            }

            if (searchTimer.milliseconds() > 1250 * (1 / uV.searchSpeedMultiplier) && searchStep == 1) {
                searchSign *= -1;
                searchTimer.reset();
                ++searchStep;
            }

            if (searchTimer.milliseconds() > 2500 * (1 / uV.searchSpeedMultiplier) && searchStep == 2) {
                searchSign *= -1;
                searchTimer.reset();
            }

            turretMotor.setPower(uV.searchSpeedMultiplier * searchSign);
        }
    }
}
