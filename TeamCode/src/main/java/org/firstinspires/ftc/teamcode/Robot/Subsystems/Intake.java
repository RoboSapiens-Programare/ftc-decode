package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import android.graphics.Color;
import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.Utils.ColorEnum;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@SuppressWarnings("FieldCanBeLocal")
public class Intake extends Subsystem {
    private DcMotorEx intakeMotor;
    private Spindexer spindexer;
    private final ElapsedTime cooldown = new ElapsedTime();

    private PredominantColorProcessor colorSensor;
    private ColorEnum lastGuess = ColorEnum.UNDEFINED;
    private boolean execOnce = true;

    private VisionPortal portal;

    public Intake(HardwareMap hwMap, Spindexer revolver) {
        intakeMotor = hwMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.spindexer = revolver;

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
                        .setCamera(hwMap.get(WebcamName.class, "IntakeCam"))
                        .enableLiveView(false)
                        .build();

//        FtcDashboard.getInstance().startCameraStream(portal, 30);
    }

    @Override
    public void update() {
        // check both are equal in order to ignore false positives

//        if (spindexer.getBallCount() >= 3) return;
        PredominantColorProcessor.Result result = colorSensor.getAnalysis();
        FtcDashboard.getInstance().getTelemetry().addData("hue",result.HSV[0]);
        FtcDashboard.getInstance().getTelemetry().addData("sat",result.HSV[1]);
        FtcDashboard.getInstance().getTelemetry().addData("val",result.HSV[2]);

        if (execOnce) {
            if (result.HSV[0] >= 70 && result.HSV[0] <= 95 && result.HSV[1] > 90) {
                lastGuess = ColorEnum.GREEN;
                execOnce = false;
                return;
            } else if (result.HSV[0] >= 120 && result.HSV[0] <= 170 && result.HSV[1] > 90) {
                lastGuess = ColorEnum.PURPLE;
                execOnce = false;
                return;
            }
        }

        if (cooldown.milliseconds() < 500 && !execOnce) {
            return;
        }

        cooldown.reset();

        if (result.HSV[0] >= 70 && result.HSV[0] <= 95 && result.HSV[1] > 90 && lastGuess==ColorEnum.GREEN) {
            spindexer.setSlotColor(
                    spindexer.getTargetSlot(),
                    ColorEnum.GREEN);

        } else if (result.HSV[0] >= 120 && result.HSV[0] <= 170 && result.HSV[1] > 90 && lastGuess == ColorEnum.PURPLE) {
            spindexer.setSlotColor(
                    spindexer.getTargetSlot(),
                    ColorEnum.PURPLE);
        } else {
            lastGuess = ColorEnum.UNDEFINED;
        }

        execOnce = true;

    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }
}
