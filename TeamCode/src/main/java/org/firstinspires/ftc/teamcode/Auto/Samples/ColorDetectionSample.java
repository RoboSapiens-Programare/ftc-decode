package org.firstinspires.ftc.teamcode.Auto.Samples;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="Color Detection Sample", group="1. Autonomous Samples")

//TODO MODIFICAT DIN LAPTOP CA NU STIU VALORILE
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

        // IMPORTANT
        // PLEASE USE THESE VALUES AS A BASIS FOR THE NEXT COLOR SENSOR "CALIBRATION"

        /*
        averages of the sensor argb values on 19/11

        1644167168 nothing in front

        ^^ observe how this value is 1 digit more than the others

        201326592 green hole
        150994944 purple hole
        -285015291 purple flush
        -117307130 green flush

         */

        if (h >= 180 && argb < 369762048) {
            return ColorEnum.PURPLE;
        } else if (h <= 180 && argb < 369762048 ) {
            return ColorEnum.GREEN;
        } else if (argb < 1044167168)
        {   // defaults to green if it just sees something in front of it
            // this happens if it's a purple hole because you can not differentiate
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
