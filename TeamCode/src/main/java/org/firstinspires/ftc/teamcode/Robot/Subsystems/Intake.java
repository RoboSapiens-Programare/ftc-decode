package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.Utils.ColorEnum;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@SuppressWarnings("FieldCanBeLocal")
public class Intake extends Subsystem {
    public DcMotorEx intakeMotor;
    private Revolver revolver;
    private final ElapsedTime cooldown = new ElapsedTime();
    private final ElapsedTime cooldown2 = new ElapsedTime();

    private boolean doCooldown = false;

    public PredominantColorProcessor colorSensor;

    public VisionPortal portal;

    public Intake(HardwareMap hwMap, Revolver revolver) {
        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");

        this.revolver = revolver;

        colorSensor =
                new PredominantColorProcessor.Builder()
                        .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                        .setSwatches(
                                PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                                PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                                PredominantColorProcessor.Swatch.RED,
                                PredominantColorProcessor.Swatch.BLUE,
                                PredominantColorProcessor.Swatch.YELLOW,
                                PredominantColorProcessor.Swatch.BLACK,
                                PredominantColorProcessor.Swatch.WHITE)
                        .build();

        portal =
                new VisionPortal.Builder()
                        .addProcessor(colorSensor)
                        .setCameraResolution(new Size(320, 240))
                        .setCamera(hwMap.get(WebcamName.class, "intakeCam"))
                        .enableLiveView(false)
                        .build();
    }

    public void disableCamera() {
        // if (portal.getCameraState() == VisionPortal.CameraState.STREAMING)
        // portal.stopStreaming();
        // colorSensor.setEnabled(false);
        portal.setProcessorEnabled(colorSensor, false); // disable
    }

    public void enableCamera() {
        // if (portal.getCameraState() != VisionPortal.CameraState.STREAMING)
        // portal.resumeStreaming();
        portal.setProcessorEnabled(colorSensor, true); // disable

        doCooldown = true;
        cooldown.reset();
    }

    @Override
    public void update() {
        // check both are equal in order to ignore false positives

        if (revolver.getBallCount() >= 3) return;
        if (doCooldown && cooldown.milliseconds() > 1000) {
            cooldown.reset();
            doCooldown = false;
        }
        if (!revolver.isReady() || doCooldown) return;

        PredominantColorProcessor.Result result = colorSensor.getAnalysis();

        FtcDashboard.getInstance().getTelemetry().addData("color", result.closestSwatch);

        if (result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN) {
            revolver.setSlotColor(revolver.getTargetSlot(), ColorEnum.GREEN);
        } else if (result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) {
            revolver.setSlotColor(revolver.getTargetSlot(), ColorEnum.PURPLE);
        }
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }
}
