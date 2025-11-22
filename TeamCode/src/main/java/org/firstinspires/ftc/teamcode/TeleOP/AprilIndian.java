package org.firstinspires.ftc.teamcode.TeleOP;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "TURRET TEST")
public class AprilIndian extends OpMode {

  private DcMotor turretMotor;

  private final int TAGID = 20;
  private final int frameWidth = 640;

  private AprilTagProcessor tagProcessor;

  @Override
  public void init() {

    // FtcDashboard.getInstance().startCameraStream(camera, 12);

    turretMotor = hardwareMap.get(DcMotor.class, "CamPivot");

    tagProcessor = AprilTagProcessor.easyCreateWithDefaults();
    VisionPortal.Builder vBuilder = new VisionPortal.Builder();

    vBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
    vBuilder.addProcessor(tagProcessor);
    vBuilder.setCameraResolution(new Size(frameWidth, 720));
    vBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

    VisionPortal vision = vBuilder.build();

    vision.resumeStreaming();
  }

  @Override
  public void loop() {

    List<AprilTagDetection> result = tagProcessor.getDetections();

    if (!result.isEmpty()) {
      for (AprilTagDetection tag : result) {
        if (tag.id == TAGID) {
          telemetry.addData("TAG OUT", tag.center.x);
          turretMotor.setPower(
              (tag.center.x - ((double) frameWidth / 2)) / ((double) frameWidth / 2) * 0.225);
        }
      }
    } else {
      telemetry.addData("TAG OUT", "NONE");
      turretMotor.setPower(0.05);
    }
  }
}
