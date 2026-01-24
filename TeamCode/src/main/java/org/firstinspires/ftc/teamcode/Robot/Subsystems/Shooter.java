package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Utils.PIDFController;

@SuppressWarnings("FieldCanBeLocal")
@Config
public class Shooter extends Subsystem {
    private final DcMotorEx turretMotorLeft;
    public final DcMotorEx turretMotorRight;

    // PID values for shooter
    // WHEN TUNING USE ZIEGLER-NICHOLS METHOD
    // IT WAS MADE FOR THIS
    // LITERALLY FOR THIS

    public static double shootKp = 0.07;
    public static double shootKi = 0.00002;
    public static double shootKd = 0.0000001;
    public static double shootKf = 0.013;

    private final PIDFController pidfController =
            new PIDFController(shootKp, shootKi, shootKd, shootKf);

    private final Pose blueObeliskPose = new Pose(12, 135);
    private final Pose redObeliskPose = new Pose(133, 135);

    public static double targetVelocity = 1300;

    public boolean isTracking = false;

    public boolean shooting = false;

    private boolean shouldFollowTrack = true;

    public Shooter(HardwareMap hwMap) {
        turretMotorRight = hwMap.get(DcMotorEx.class, "natasha");
        turretMotorLeft = hwMap.get(DcMotorEx.class, "starDestroyer");
        turretMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        turretMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidfController.setTolerance(20);
        pidfController.maxOut = 2;
        pidfController.minOut = -2;
    }

    public boolean isShootReady() {
        boolean aligned =
                Robot.follower.getPose().getHeading() >= getAngle() - 5 * 2 * Math.PI / 360
                        && Robot.follower.getPose().getHeading()
                                <= getAngle() + 5 * 2 * Math.PI / 360;

        return pidfController.targetReached() && aligned;
    }

    public double computeDistance() {
        Pose currentPose = Robot.follower.getPose();
        Pose targetObeliskPose =
                Robot.alliance == Robot.Alliance.RED ? redObeliskPose : blueObeliskPose;

        return currentPose.distanceFrom(targetObeliskPose);
    }

    private double computeVelocity() {
        double dist = computeDistance();
        if (dist > 100) {
            return 1450;
        }
        if (dist < 65) {
            return 1100;
        }
        return dist * 1.45 + 1065;
    }

    public double getAngle(double x, double y) {
        Pose targetObeliskPose =
                Robot.alliance == Robot.Alliance.RED ? redObeliskPose : blueObeliskPose;

        double dx = Math.abs(x - targetObeliskPose.getX());
        double dy = Math.abs(y - targetObeliskPose.getY());

        double alpha = Math.atan(dy / dx);

        return Robot.alliance == Robot.Alliance.RED ? alpha : Math.PI - alpha;
    }

    public double getAngle() {
        Pose currentPose = Robot.follower.getPose();

        return getAngle(currentPose.getX(), currentPose.getY());
    }

    public void track() {
        Path p =
                new Path(
                        new BezierLine(
                                Robot.follower.getPose(),
                                new Pose(
                                        Robot.follower.getPose().getX() + 1,
                                        Robot.follower.getPose().getY() + 1)));

        isTracking = true;

        p.setLinearHeadingInterpolation(
                Robot.follower.getHeading(),
                getAngle(Robot.follower.getPose().getX() + 1, Robot.follower.getPose().getY() + 1));

        Robot.follower.breakFollowing();
        Robot.follower.followPath(p);
    }

    @Override
    public void update() {
        // TODO: remove after PID tuning
        pidfController.kP = shootKp;
        pidfController.kI = shootKi;
        pidfController.kD = shootKd;
        pidfController.kF = shootKf;

        targetVelocity = computeVelocity();
        pidfController.setSetpoint(targetVelocity);

        if (shooting) {

            double pidOutput = pidfController.updatePID(turretMotorRight.getVelocity());
            ////
            FtcDashboard.getInstance().getTelemetry().addData("pid vel", pidOutput);
            turretMotorRight.setPower(pidOutput / 2);
            turretMotorLeft.setPower(pidOutput / 2);
        } else {
            turretMotorRight.setPower(0);
            turretMotorLeft.setPower(0);
        }

        if (isTracking && shouldFollowTrack) {
            Path p =
                    new Path(
                            new BezierLine(
                                    Robot.follower.getPose(),
                                    new Pose(
                                            Robot.follower.getPose().getX() + 1,
                                            Robot.follower.getPose().getY() + 1)));

            p.setConstantHeadingInterpolation(this.getAngle());

            Robot.follower.followPath(p, true);

            shouldFollowTrack = false;
        }
    }

    public void reset() {
        shouldFollowTrack = true;
    }
}
