package org.firstinspires.ftc.teamcode.Auto.Calibration;

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.util.LinkedList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@SuppressWarnings("FieldCanBeLocal")
@Autonomous(name = "Color Sensor", group = "2. Calibration")
public class ColorSensorCalib extends OpMode {
    private PredominantColorProcessor colorSensor;
    private VisionPortal portal;

    private List<Integer> hueList = new LinkedList<>();
    private int min = Integer.MAX_VALUE;
    private int max = Integer.MIN_VALUE;

    @Override
    public void init() {

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
                        .setCamera(hardwareMap.get(WebcamName.class, "IntakeCam"))
                        .enableLiveView(false)
                        .build();

        FtcDashboard.getInstance().startCameraStream(portal, 30);
    }

    @Override
    public void loop() {
        PredominantColorProcessor.Result result = colorSensor.getAnalysis();
        FtcDashboard.getInstance().getTelemetry().addData("hue", result.HSV[0]);
        FtcDashboard.getInstance().getTelemetry().addData("hue list", hueList);
        FtcDashboard.getInstance().getTelemetry().addData("hue min", min);
        FtcDashboard.getInstance().getTelemetry().addData("hue max", max);

        FtcDashboard.getInstance().getTelemetry().addData("hue", result.HSV[0]);
        FtcDashboard.getInstance().getTelemetry().addData("sat", result.HSV[1]);
        FtcDashboard.getInstance().getTelemetry().addData("val", result.HSV[2]);

        FtcDashboard.getInstance().getTelemetry().update();

        if (result.HSV[0] < min) {
            min = result.HSV[0];
        }
        if (result.HSV[0] > max) {
            max = result.HSV[0];
        }

        for (int hue : hueList) {
            if (hue == result.HSV[0]) {
                return;
            }
        }

        hueList.add(result.HSV[0]);
    }
}
