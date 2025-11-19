package org.firstinspires.ftc.teamcode.Auto.Samples;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="Color Detection Sample", group="1. Autonomous Samples")
public class ColorDetectionSample extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private ColorSensor sensor;

    enum ColorEnum {
        GREEN,
        PURPLE,
        UNDEFINED
    };

    private ColorEnum getColor(ColorSensor sensor) {
        int argb = sensor.argb();
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        float[] hsv = {0, 0, 0};
        Color.RGBToHSV(r, g, b, hsv);
        float h = hsv[0]; // convert hue to degrees

        dashboardTelemetry.addData("hue", hsv[0]);

        // works only at a specific distance, change when remounting sensor :D

        if (h >= 170 && h <= 255 && argb > 300000000) {
            return ColorEnum.PURPLE;
        } else if (h >= 120 && h <= 170 && argb > 800000000) {
            return ColorEnum.GREEN;
        } else {
            return ColorEnum.UNDEFINED;
        }
    }

    @Override
    public void init() {
        sensor = hardwareMap.get(ColorSensor.class, "colorSensor");
    }

    @Override
    public void loop() {

        dashboardTelemetry.addData("sensor", getColor(sensor));

        dashboardTelemetry.addData("argb", sensor.argb());

        dashboardTelemetry.addData("sensor rgb", String.format("%d %d %d", sensor.red(), sensor.green(), sensor.blue()));

        dashboardTelemetry.update();

    }
}
