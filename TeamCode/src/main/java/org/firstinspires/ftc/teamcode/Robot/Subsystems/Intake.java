package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.uV;

public class Intake {
    public DcMotorEx intakeMotor;
    private ColorSensor colorSensorLeft, colorSensorRight;
    private Revolver r;

    enum ColorEnum {
        GREEN,
        PURPLE,
        UNDEFINED
    };


    public Intake(HardwareMap hwMap, Revolver revolver) {
        r = revolver;

        colorSensorLeft = hardwareMap.get(ColorSensor.class, "colorSensorLeft");
        colorSensorRight = hardwareMap.get(ColorSensor.class, "colorSensorRight");
    }


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

    public void update() {
        // check both are equal in order to ignore false positives
        if (getColor(colorSensorLeft) != getColor(colorSensorRight)) {
            return;
        }

        switch (getColor(colorSensorLeft)) {
            case ColorEnum.GREEN:
                handleGreen();
                break;
        
            case ColorEnum.PURPLE:
                handlePurple();
                break;

            default:
                break;
        }
    }

    public void handleGreen() {
        // placeholder
    }

    public void handlePurple() {
        // placeholder
    }

    public void setPower(float power) {
        intakeMotor.setPower(power);
    }
    
    public void startMotor() {
        intakeMotor.setPower(uV.intakePower);
    }

    public void stopMotor() {
        intakeMotor.setPower(0);
    }
}
