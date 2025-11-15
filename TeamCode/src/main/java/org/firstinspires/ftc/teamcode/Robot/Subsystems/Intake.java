package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Utils.ColorEnum;
import org.firstinspires.ftc.teamcode.Robot.uV;

public class Intake {
    public DcMotorEx intakeMotor;
    private final ColorSensor colorSensor;
    private Revolver revolver;
    private final ElapsedTime cooldown = new ElapsedTime();

    public Intake(HardwareMap hwMap, Revolver revolver) {
        intakeMotor =  hwMap.get(DcMotorEx.class, "intakeMotor");

        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");

        this.revolver = revolver;
    }

    private ColorEnum getColor(ColorSensor sensor) {
        int argb = sensor.argb();
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        float[] hsv = {0, 0, 0};
        Color.RGBToHSV(r, g, b, hsv);
        float h = hsv[0]; // convert hue to degrees

        // works only at a specific distance, change when remounting sensor :D

        if (h >= 200 && h <= 255 && argb > 300000000) {
            return ColorEnum.PURPLE;
        } else if (h >= 120 && h <= 200 && argb > 300000000) {
            return ColorEnum.GREEN;
        } else {
            return ColorEnum.UNDEFINED;
        }
    }

    public void update() {
        // check both are equal in order to ignore false positives

        if (revolver.getBallCount() >= 3)
            return;
        FtcDashboard.getInstance().getTelemetry().addData("colorSensor: ", getColor(colorSensor));
        ColorEnum col = getColor(colorSensor);
        if (col == ColorEnum.UNDEFINED) {
            return;
        }

        if (cooldown.milliseconds() < 500) {
            return;
        }

        cooldown.reset();

        revolver.setSlotColor(revolver.getTargetSlot(), col);

//        byte b = revolver.getFreeSlot();
//        if (b == 5)
//            return;
//
//        revolver.setTargetSlot(b);

    }

    public void handleGreen() {
        byte b = revolver.getFreeSlot();
        if (b != 5) {
            revolver.setSlotColor(b, ColorEnum.GREEN);
        }
    }

    public void handlePurple() {
        byte b = revolver.getFreeSlot();
        if (b != 5) {
            revolver.setSlotColor(b, ColorEnum.PURPLE);
        }
    }

    public void setPower(float power) {
        intakeMotor.setPower(power);
    }
    
    public void startMotor() {
//        intakeMotor.setPower(uV.intakePower);
    }

    public void stopMotor() {
        intakeMotor.setPower(0);
    }
}
