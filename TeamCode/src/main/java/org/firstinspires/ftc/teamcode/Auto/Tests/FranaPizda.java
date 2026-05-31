package org.firstinspires.ftc.teamcode.Auto.Tests;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "frana mijoarca")
public class FranaPizda extends OpMode {

    private Robot robot;
    private DcMotorEx leftBack;

    private ElapsedTime brakeTimer = new ElapsedTime();
    private boolean isBraking = false;

    // ADJUST THESE CONSTANTS
    private final double STOP_THRESHOLD = 2.0; // Ticks per second (near zero)
    private final double COOLDOWN_MS = 140.0; // Time to wait once stopped

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        // Cast to DcMotorEx to get getVelocity()
        leftBack = (DcMotorEx) hardwareMap.get(DcMotorEx.class, "leftRear");

        Robot.follower = Constants.createFollower(hardwareMap);
        Robot.follower.startTeleOpDrive(true);
        Robot.follower.setStartingPose(new Pose(0, 0));
    }

    @Override
    public void loop() {
        // 1. Get real-time velocity from the encoder
        double currentVelocity = Math.abs(leftBack.getVelocity());

        // 2. Check if the motor has physically stopped
        if (currentVelocity <= STOP_THRESHOLD) {
            if (!isBraking) {
                // This triggers the moment the encoder reports ~0
                brakeTimer.reset();
                isBraking = true;
            }
        } else {
            // Motor is clearly moving
            isBraking = false;
        }

        // 3. Prevent direction flipping during the "Short Circuit" phase
        // If we just stopped, wait for COOLDOWN_MS before accepting new stick inputs
        if (isBraking && brakeTimer.milliseconds() < COOLDOWN_MS) {
            Robot.follower.setTeleOpDrive(0, 0, 0, true);
        } else {
            Robot.follower.setTeleOpDrive(
                    -gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        }

        Robot.follower.update();

        // Debugging
        telemetry.addData("LB Velocity", currentVelocity);
        telemetry.addData("Brake Status", isBraking ? "SETTLING" : "READY");
        telemetry.update();
    }
}
