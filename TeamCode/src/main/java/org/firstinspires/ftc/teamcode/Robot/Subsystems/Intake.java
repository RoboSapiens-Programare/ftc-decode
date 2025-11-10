package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import android.graphics.Color;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Robot.uV;

public class Intake {
    public DcMotorEx intakeMotor;
    private final ColorSensor colorSensorLeft;
    private final ColorSensor colorSensorRight;

    enum ColorEnum {
        GREEN,
        PURPLE,
        UNDEFINED
    };


    public Intake(HardwareMap hwMap, Revolver revolver) {

        intakeMotor =  hwMap.get(DcMotorEx.class, "intakeMotor");

        colorSensorLeft = hwMap.get(ColorSensor.class, "colorSensorLeft");
        colorSensorRight = hwMap.get(ColorSensor.class, "colorSensorRight");
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
            case GREEN:
                handleGreen();
                break;
        
            case PURPLE:
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
