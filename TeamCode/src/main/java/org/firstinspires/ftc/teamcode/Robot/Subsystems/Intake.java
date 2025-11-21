package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.Utils.ColorEnum;
import org.firstinspires.ftc.teamcode.Robot.uV;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

public class Intake {
    public DcMotorEx intakeMotor;
//    private final ColorSensor colorSensor;
    private Revolver revolver;
    private final ElapsedTime cooldown = new ElapsedTime();

    private PredominantColorProcessor colorSensor;

    private VisionPortal portal;

    public Intake(HardwareMap hwMap, Revolver revolver) {
        intakeMotor =  hwMap.get(DcMotorEx.class, "intakeMotor");

        this.revolver = revolver;

        colorSensor = new PredominantColorProcessor.Builder()
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

        portal = new VisionPortal.Builder()
                .addProcessor(colorSensor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hwMap.get(WebcamName.class, "IntakeCam"))
                .enableLiveView(false)
                .build();
    }

    public void update() {
        // check both are equal in order to ignore false positives

        if (revolver.getBallCount() >= 3)
            return;

        PredominantColorProcessor.Result result = colorSensor.getAnalysis();


        if (result.closestSwatch != PredominantColorProcessor.Swatch.ARTIFACT_GREEN && result.closestSwatch != PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) {
            return;
        }

        if (cooldown.milliseconds() < 500) {
            return;
        }

        cooldown.reset();

        revolver.setSlotColor(
                revolver.getTargetSlot(),
                result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN ? ColorEnum.GREEN : ColorEnum.PURPLE
        );

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
        intakeMotor.setPower(uV.intakePower);
    }

    public void stopMotor() {
        intakeMotor.setPower(0);
    }


}
