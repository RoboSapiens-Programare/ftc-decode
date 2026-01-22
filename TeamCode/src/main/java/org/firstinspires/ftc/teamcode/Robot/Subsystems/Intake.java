package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import android.graphics.Color;
import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.Utils.ColorEnum;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@SuppressWarnings("FieldCanBeLocal")
public class Intake extends Subsystem {
    private final DcMotorEx intakeMotor;
    private final CRServo rollerLeft;
    private final CRServo rollerRight;

    private final TouchSensor intakeSensor;
    private final Spindexer spindexer;
    private final ElapsedTime cooldown = new ElapsedTime();

    private final PredominantColorProcessor colorSensor;
    private VisionPortal portal;

    public Intake(HardwareMap hwMap, Spindexer revolver) {
        intakeMotor = hwMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rollerLeft = hwMap.get(CRServo.class, "rollerLeft");
        rollerRight = hwMap.get(CRServo.class, "rollerRight");
        rollerRight.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeSensor = hwMap.get(TouchSensor.class, "intakeSensor");

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

        FtcDashboard.getInstance().startCameraStream(portal, 30);
    }

    @Override
    public void update() {
        // check both are equal in order to ignore false positives

//        if (spindexer.getBallCount() >= 3) return;
        PredominantColorProcessor.Result result = colorSensor.getAnalysis();
        FtcDashboard.getInstance().getTelemetry().addData("hue",result.HSV[0]);
        FtcDashboard.getInstance().getTelemetry().addData("sat",result.HSV[1]);
        FtcDashboard.getInstance().getTelemetry().addData("val",result.HSV[2]);

        if (cooldown.milliseconds() > 200 && intakeSensor.isPressed() && spindexer.isReady()) {
            if (result.HSV[0] >= 70 && result.HSV[0] <= 95 && result.HSV[1] > 90) {
                spindexer.setSlotColor(
                        spindexer.getTargetSlot(),
                        ColorEnum.GREEN);
                cooldown.reset();


            } else if (result.HSV[0] >= 120 && result.HSV[0] <= 170 && result.HSV[1] > 90) {
                spindexer.setSlotColor(
                        spindexer.getTargetSlot(),
                        ColorEnum.PURPLE);
                cooldown.reset();

            }
        }

    }

    public void setPower(double power) {
        setPower(power, true);
    }

    public void setPower(double power, boolean roller) {
        intakeMotor.setPower(power);
        rollerRight.setPower(0.375 * (roller ? 1 : 0));
        rollerLeft.setPower((roller ? 1 : 0));
    }

    public void setRollerPower(double left, double right) {
        rollerLeft.setPower(left);
        rollerRight.setPower(right);
    }
}
