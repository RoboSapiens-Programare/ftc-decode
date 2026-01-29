package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
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
    private final ElapsedTime pipelineTimer = new ElapsedTime();
    private final ElapsedTime spindexerDelay = new ElapsedTime();

    private final PredominantColorProcessor colorSensor;

    private int pipelineIndex = 0;
    private boolean doOnce = true;
    private VisionPortal portal;

    private final Limelight3A ll;

    public Intake(HardwareMap hwMap, Spindexer revolver) {

        intakeMotor = hwMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

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

        ll = hwMap.get(Limelight3A.class, "limelight");

        ll.start();
        FtcDashboard.getInstance().startCameraStream(ll, 30);
        //        ll.

        ll.pipelineSwitch(pipelineIndex);

        //        FtcDashboard.getInstance().startCameraStream(portal, 30);
    }

    @Override
    public void update() {

//        if (spindexer.getBallCount() >= 3) return;

        if (doOnce) {
            pipelineIndex = (pipelineIndex + 1) % 2;
            ll.pipelineSwitch(pipelineIndex);

            pipelineTimer.reset();
            doOnce = false;
        }

        FtcDashboard.getInstance().getTelemetry().addData("pipeline timer", pipelineTimer.milliseconds());
        if (pipelineTimer.milliseconds() > 100) {
            LLResult result = ll.getLatestResult();
            FtcDashboard.getInstance().getTelemetry().addData("result", result.isValid());
            FtcDashboard.getInstance().getTelemetry().addData("pipeline", pipelineIndex);

            if (result.isValid() && !intakeSensor.isPressed()) {
                switch (pipelineIndex) {
                    case 0:
                        spindexer.setSlotColor(spindexer.getTargetSlot(), ColorEnum.PURPLE);
                        doOnce = true;
                        break;
                    case 1:
                        spindexer.setSlotColor(spindexer.getTargetSlot(), ColorEnum.GREEN);
                        doOnce = true;
                        break;
                }

            }

            if (pipelineTimer.milliseconds() > 200) {
                doOnce = true;
            }
        }




        FtcDashboard.getInstance().getTelemetry().addData("rev ready", spindexer.isReady());
        FtcDashboard.getInstance()
                .getTelemetry()
                .addData("intake sensor", intakeSensor.isPressed());
        FtcDashboard.getInstance().getTelemetry().addData("cooldown", cooldown.milliseconds());
        FtcDashboard.getInstance()
                .getTelemetry()
                .addData(
                        "intake detected",
                        cooldown.milliseconds() > 100
                                && !intakeSensor.isPressed()
                                && spindexer.isReady());
        FtcDashboard.getInstance().getTelemetry().addData("ballcount", spindexer.getBallCount());

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
