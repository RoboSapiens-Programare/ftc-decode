package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.Utils.PIDFController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@SuppressWarnings("FieldCanBeLocal")
@Config
public class Turret {
  public DcMotorEx turretMotor;
  public CRServo turretRotationServo;
  private float turretRotation;
  public boolean found = false;

  public enum TargetObelisk {
    RED,
    BLUE
  };

  public TargetObelisk targetObelisk = TargetObelisk.BLUE;

  public final int frameWidth = 640;
  public double currentPos = 0;
  public boolean tracking = true;

  public AprilTagProcessor tagProcessor;

  // PID values for turret
  // WHEN TUNING USE ZIEGLER-NICHOLS METHOD
  // IT WAS MADE FOR THIS
  // LITERALLY FOR THIS

  // old value that was tested with
  //    public double Kp = 0.000495;

  // new value that wasn't tested
  public double Kp = 0.000505;
  public double Ki = 0.001;
  public double Kd = 0.000165;
  public static double Kf = 0.065; // Power to overcome inertia and friction

  public static double shootKp = 1800 * 0.2;
  public static double shootKi = 1800 * 0.4 / (4.0 / 10);
  public static double shootKd = 0.066 * 1800 * 4 / 10;
  public static double shootKf = 9;
  public static double velo = 0;

  public PIDFController pidfController = new PIDFController(Kp, Ki, Kd, Kf);
  public PIDFController velocitypidfController = new PIDFController(Kp, Ki, Kd, Kf);

  private VisionPortal vision;
  public double targetVelocity;

  public Turret(HardwareMap hwMap) {
    turretMotor = hwMap.get(DcMotorEx.class, "turretMotor");
    turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    turretRotationServo = hwMap.get(CRServo.class, "turretRotationServo");

    tagProcessor = AprilTagProcessor.easyCreateWithDefaults();
    VisionPortal.Builder vBuilder = new VisionPortal.Builder();

    vBuilder.setCamera(hwMap.get(WebcamName.class, "webcamTurret"));
    vBuilder.addProcessor(tagProcessor);
    vBuilder.setCameraResolution(new Size(frameWidth, 480));
    vBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

    vision = vBuilder.build();

    vision.resumeStreaming();

    FtcDashboard.getInstance().startCameraStream(vision, 30);

    pidfController.setSetpoint(frameWidth / 2.0);
    pidfController.setTolerance(0);
  }

  public void toggleTracking() {
    tracking = !tracking;
  }

  public void enableCamera() {
    if (vision != null) {
      vision.resumeStreaming(); // restarts the webcam stream
    }
  }

  public void disableCamera() {
    if (vision != null) {
      vision.stopStreaming(); // fully stops camera pipeline
    }
  }

  public void update() {
    turretMotor.setVelocityPIDFCoefficients(shootKp, shootKi, shootKd, shootKf);

    if (!tracking) {
      found = false;
      return;
    }

    List<AprilTagDetection> result = tagProcessor.getDetections();

    if (!result.isEmpty()) {
      for (AprilTagDetection tag : result) {
        if (tag.id == (targetObelisk == TargetObelisk.RED ? 24 : 20)) {
          FtcDashboard.getInstance().getTelemetry().addData("tag center", tag.center.x);
          found = true;
          currentPos = tag.center.x;
          // -50 is the physical offset, currently aims too much to the right, compensates 50 to the
          // left
          turretRotationServo.setPower(pidfController.updatePID(tag.center.x - 50));

          double dist =
              tag.ftcPose.x * tag.ftcPose.x
                  + tag.ftcPose.y * tag.ftcPose.y
                  + tag.ftcPose.z * tag.ftcPose.z;
          dist = Math.sqrt(dist);

          // TODO: check if this varies maechanically based on battery voltage and than change the
          // power based on it

          //                    turretMotor.setPower(0.586767 + 0.0025*(dist - 57));

          //                    if (dist >= 73.5) {
          //                        turretMotor.setVelocity((dist - 73.5) * 125 / 23.5 + 1100);
          //                        FtcDashboard.getInstance().getTelemetry().addData("s", (dist -
          // 73.5) * 125 / 23.5 + 1100);
          //                    } else {
          //                        turretMotor.setVelocity((dist - 73.5) * 140 / 23.5 + 960);
          //                        FtcDashboard.getInstance().getTelemetry().addData("s", (dist -
          // 73.5) * 140 / 23.5 + 960);
          //
          //                    }

          turretMotor.setVelocity(((dist - 52.3) * 375 / 33.15 + 1000));
          targetVelocity = ((dist - 52.3) * 375 / 33.15 + 1000);

          //                    FtcDashboard.getInstance().getTelemetry().addData("v",
          // turretMotor.getVelocity());
          //                    FtcDashboard.getInstance().getTelemetry().addData("distance", dist);
          //                    FtcDashboard.getInstance().getTelemetry().addData("distance power",
          // dist*0.01);
          //                    FtcDashboard.getInstance().getTelemetry().addData("log distance
          // power", 0.586767 + 0.0025*(dist - 57));
        }
      }
    } else {
      // no more search fuck off driver 2
      turretRotationServo.setPower(0);
      found = false;
    }
  }
}
