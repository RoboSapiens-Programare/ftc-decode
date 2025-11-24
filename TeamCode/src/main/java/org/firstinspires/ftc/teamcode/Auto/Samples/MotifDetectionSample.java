package org.firstinspires.ftc.teamcode.Robot.Utils;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.teamcode.Robot.Robot;

// @Disabled
@TeleOp(name = "Concept: Motif Detection", group = "Concept")
public class MotifDetectionSample extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        robot.turret.enableCamera();
    }

    @Override
    public void loop() {
        List<LLResultTypes.FiducialResult> fiducialResults =
                robot.turret.limelight.getLatestResult().getFiducialResults();

        if (!fiducialResults.isEmpty()) {
            for (LLResultTypes.FiducialResult tag : fiducialResults) {
                if (tag.getFiducialId() == 21
                        || tag.getFiducialId() == 22
                        || tag.getFiducialId() == 23) {
                    robot.revolver.greenPosition = tag.getFiducialId() - 21;
                }
            }
        }
    }
}
