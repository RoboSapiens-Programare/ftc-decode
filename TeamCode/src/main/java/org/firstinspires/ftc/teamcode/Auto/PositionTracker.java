package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "Position Tracker - LFT", group = "Autonomous - Tools")
@Config
public class PositionTracker extends OpMode {
    private int loopCount = 0;
    long lastTime = System.nanoTime();
    private double averagedFrequency = 50.0; // Seed it with an expected baseline (e.g., 50-60Hz)
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        robot.shooter.init();

        Robot.alliance = Robot.Alliance.RED;

        // Determine starting heading: prefer geometric heading when a path exists, otherwise fall
        // back to explicit startPoint values
        Robot.follower.setStartingPose(new Pose(84, -115, -Math.PI / 2 - 0.601));
    }

    @Override
    public void loop() {
        robot.resetCache();

        robot.update();

        // Log values to Panels and Driver Station
        updateTelemetry();
    }

    @Override
    public void stop() {
        Robot.transitionPose = Robot.follower.getPose();
        robot.shooter.togglePurpleObelisk();
    }

    private void updateTelemetry() {
        long currentTime = System.nanoTime();
        double loopTimeSeconds = (currentTime - lastTime) / 1_000_000_000.0;
        lastTime = currentTime;

        // Guard against division by zero on initialization anomalies
        if (loopTimeSeconds > 0) {
            double instantFrequency = 1.0 / loopTimeSeconds;

            // Exponential Moving Average Formula:
            // Alpha (0.05) determines responsiveness vs smoothness. Lower = smoother.
            double alpha = 0.05;
            averagedFrequency = (alpha * instantFrequency) + ((1.0 - alpha) * averagedFrequency);
        }

        // Limit the dashboard telemetry network updates to every 10 frames
        if (loopCount++ < 10) {
            return;
        }
        loopCount = 0;

        dashboardTelemetry.addData("Loop Hz (Avg)", Math.round(averagedFrequency));
        dashboardTelemetry.addData("X", Robot.follower.getPose().getX());
        dashboardTelemetry.addData("Y", Robot.follower.getPose().getY());
        dashboardTelemetry.addData("Heading", Robot.follower.getPose().getHeading());
        dashboardTelemetry.addData(
                "Pose",
                String.format(
                        "new Pose(%f, %f, %f)",
                        Robot.follower.getPose().getX(),
                        Robot.follower.getPose().getY(),
                        Robot.follower.getPose().getHeading()));

        telemetry.addData("Loop Hz (Avg)", Math.round(averagedFrequency));
        telemetry.addData("X", Robot.follower.getPose().getX());
        telemetry.addData("Y", Robot.follower.getPose().getY());
        telemetry.addData("Heading", Robot.follower.getPose().getHeading());

        dashboardTelemetry.update();
        telemetry.update();
    }
}
