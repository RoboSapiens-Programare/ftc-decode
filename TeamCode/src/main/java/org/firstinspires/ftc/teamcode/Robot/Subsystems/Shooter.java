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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Utils.PIDFController;

@SuppressWarnings("FieldCanBeLocal")
@Config
public class Shooter extends Subsystem {
    private final DcMotorEx turretMotorLeft;
    public final DcMotorEx turretMotorRight;

    private final Servo lobServo;

    // PID values for shooter

    public static double shootKp = 0.07;
    public static double shootKi = 0.00002;
    public static double shootKd = 0.0000001;
    public static double shootKf = 0.013;

    private final PIDFController pidfController =
            new PIDFController(shootKp, shootKi, shootKd, shootKf);

    private final Pose blueObeliskPose = new Pose(12, 134);
    private final Pose redObeliskPose = new Pose(133, 134);

    public static double targetVelocity = 1300;

    public boolean isTracking = false;

    public boolean shooting = false;

    private boolean shouldFollowTrack = true;

    public Shooter(HardwareMap hwMap) {
        lobServo = hwMap.get(Servo.class, "lobServo");

        turretMotorRight = hwMap.get(DcMotorEx.class, "turretMotorLeft");
        turretMotorLeft = hwMap.get(DcMotorEx.class, "turretMotorRight");
        turretMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        turretMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidfController.setTolerance(20);
        pidfController.maxOut = 2;
        pidfController.minOut = -2;
    }

    public boolean isShootReady() {
        double tolerance = Math.toRadians(2);
//        boolean aligned =

        // TODO: implement pivoting turret here

        FtcDashboard.getInstance().getTelemetry().addData("Angle delta", Math.toDegrees(Robot.follower.getHeading() - getAngle()));

//        return pidfController.targetReached() && aligned;
        return false;
    }

    public boolean velocityReached() {
        return pidfController.targetReached();
    }

    public double computeDistance() {
        Pose currentPose = Robot.follower.getPose();
        Pose targetObeliskPose =
                Robot.alliance == Robot.Alliance.RED ? redObeliskPose : blueObeliskPose;

        return currentPose.distanceFrom(targetObeliskPose);
    }

    private double computeLob() {
        // TODO: implement this

        double dist = computeDistance();
        if (dist > 100) {
            return 1;
        }
        if (dist < 65) {
            return 0;
        }

        // should output a servo value (0 -> 1)
        // modify with telemetry for best results and change formula
        return dist * Math.pow(1, -100);

    }

    private double computeVelocity() {
        double dist = computeDistance();
        if (dist > 100) {
            return 1400;
        }
        if (dist < 65) {
            return 1100;
        }
        return dist * 1.42 + 1060;
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
        // TODO: implement turret pivot
    }

    @Override
    public void update() {
        pidfController.kP = shootKp;
        pidfController.kI = shootKi;
        pidfController.kD = shootKd;
        pidfController.kF = shootKf;

        targetVelocity = computeVelocity();
        pidfController.setSetpoint(targetVelocity);

        if (shooting) {
            double pidOutput = pidfController.updatePID(turretMotorRight.getVelocity());

            FtcDashboard.getInstance().getTelemetry().addData("pid vel", pidOutput);
            turretMotorRight.setPower(pidOutput / 2);
            turretMotorLeft.setPower(pidOutput / 2);

            lobServo.setPosition(computeLob());
        } else {
            turretMotorRight.setPower(0);
            turretMotorLeft.setPower(0);
        }

        if (isTracking) {

            // TODO: implement

        }
    }

}
