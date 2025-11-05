package org.firstinspires.ftc.teamcode.Auto.Samples;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name="Color Detection Sample", group="1. Autonomous Samples")
public class ColorDetectionSample extends OpMode {
    private ColorSensor sensor1, sensor2;

    enum ColorEnum {
        GREEN,
        PURPLE,
        UNDEFINED
    };

    private ColorEnum getColor(ColorSensor sensor) {
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        float[] hsv = {0, 0, 0};
        Color.RGBToHSV(r, g, b, hsv);
        float h = hsv[0]; // convert hue to degrees

        telemetry.addData("hue", hsv[0]);

        if (h >= 180 && h <= 220) {
            return ColorEnum.PURPLE;
        } else if (h >= 90 && h <= 160) {
            return ColorEnum.GREEN;
        } else {
            return ColorEnum.UNDEFINED;
        }
    }

    @Override
    public void init() {
        sensor1 = hardwareMap.get(ColorSensor.class, "colorSensorLeft");
        sensor2 = hardwareMap.get(ColorSensor.class, "colorSensorRight");
    }

    @Override
    public void loop() {
        sensor1.enableLed(true);
        sensor2.enableLed(false);

        telemetry.addData("sensor1", getColor(sensor1));
        telemetry.addData("sensor2", getColor(sensor2));

        telemetry.addData("sensor1 rgb", String.format("%d %d %d", sensor1.red(), sensor1.green(), sensor1.blue()));
        telemetry.addData("sensor2 rgb", String.format("%d %d %d", sensor2.red(), sensor2.green(), sensor2.blue()));

        telemetry.update();

    }
}
