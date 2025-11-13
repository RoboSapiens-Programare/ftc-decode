package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
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
//    private final ColorSensor colorSensorLeft;
    private final ColorSensor colorSensorRight;
    private Revolver revolver;
    private final ElapsedTime cooldown = new ElapsedTime();



    public Intake(HardwareMap hwMap, Revolver revolver) {

        intakeMotor =  hwMap.get(DcMotorEx.class, "intakeMotor");

//        colorSensorLeft = hwMap.get(ColorSensor.class, "colorSensorLeft");
        colorSensorRight = hwMap.get(ColorSensor.class, "colorSensor");

        this.revolver = revolver;
    }


    private ColorEnum getColor(ColorSensor sensor) {
        int r = sensor.red() / 4;
        int g = sensor.green() / 4;
        int b = sensor.blue() / 4;

        float[] hsv = {0, 0, 0};
        Color.RGBToHSV(r, g, b, hsv);
        float h = hsv[0]; // convert hue to degrees

//        telemetry.addData("hue", hsv[0]);

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
//        ColorEnum col = getColor(colorSensorLeft);

//        FtcDashboard.getInstance().getTelemetry().addData("left", getColor(colorSensorLeft));

        if (revolver.getBallCount() >= 3)
            return;
        FtcDashboard.getInstance().getTelemetry().addData("right", getColor(colorSensorRight));
//        FtcDashboard.getInstance().getTelemetry().addData("should add? ", col == getColor(colorSensorRight) && col != ColorEnum.UNDEFINED);
        ColorEnum col = getColor(colorSensorRight);
        if (col == ColorEnum.UNDEFINED) {
            return;
        }

        if (cooldown.milliseconds() < 500) {
            return;
        }

        cooldown.reset();

        revolver.setSlotColor(revolver.getTargetSlot(), col);

        byte b = revolver.getFreeSlot();
        if (b == 5)
            return;

        revolver.setTargetSlot(b);

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
        intakeMotor.setPower(uV.intakePower);
    }

    public void stopMotor() {
        intakeMotor.setPower(0);
    }
}
