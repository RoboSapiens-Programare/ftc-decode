package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Utils.PIDFController;

@SuppressWarnings("FieldCanBeLocal")
@Config
public class Turret extends Subsystem {
    public DcMotorEx turretMotor;
    public CRServo turretRotationServo;
    private float turretRotation;
    public TouchSensor leftLimit, rightLimit;
    public boolean found = false;

    public boolean tracking = true;

    public Limelight3A limelight;

    // PID values for turret
    // WHEN TUNING USE ZIEGLER-NICHOLS METHOD
    // IT WAS MADE FOR THIS
    // LITERALLY FOR THIS

    public static double Kp = 0.19;
    public static double Ki = 0;
    public static double Kd = 0.025;
    public static double Kf = 0; // Power to overcome inertia and friction

    private final double shootKp = 1800 * 0.2;
    private final double shootKi = 1800 * 0.4 / (4.0 / 10);
    private final double shootKd = 0.066 * 1800 * 4 / 10;
    private final double shootKf = 9;

    public static double velocityTolerance = 75;
    public double curr = 0;

    public PIDFController pidfController = new PIDFController(Kp, Ki, Kd, Kf);

    public double targetVelocity;

    public Turret(HardwareMap hwMap) {
        turretMotor = hwMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretRotationServo = hwMap.get(CRServo.class, "turretRotationServo");

        leftLimit = hwMap.get(TouchSensor.class, "limitaTuretaStanga");
        rightLimit = hwMap.get(TouchSensor.class, "limitaTuretaDreapta");

        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        FtcDashboard.getInstance().startCameraStream(limelight, 30);

        pidfController.setSetpoint(0);
        pidfController.setTolerance(0.3);
    }

    public void enableCamera() {
        limelight.start();
    }

    public void disableCamera() {
        limelight.stop();
    }

    public boolean isShootReady() {
        boolean velo = Math.abs(turretMotor.getVelocity() - targetVelocity) < velocityTolerance;
        return velo && found;
    }

    public double getDistance() {
        Pose robotPose = Robot.follower.getPose();
        Pose obeliskPose = new Pose(Robot.Alliance.BLUE == Robot.alliance ? 18 : 129, 132);

        double dx = robotPose.getX() - obeliskPose.getX();
        double dy = robotPose.getY() - obeliskPose.getY();

        return Math.sqrt(dx * dx + dy * dy);
    }

    public double computeVelocity() {
        targetVelocity = ((getDistance() - 47.3) * 375 / 33.15 + 900);
        // 1370
        return targetVelocity;
    }

    @Override
    public void update() {
        turretMotor.setVelocityPIDFCoefficients(shootKp, shootKi, shootKd, shootKf);

        if (!tracking) {
            found = false;
            return;
        }

        LLResult result = limelight.getLatestResult();

        if (result.isValid()) {
            if (result.getFiducialResults().get(0).getFiducialId()
                    == (Robot.alliance == Robot.Alliance.RED ? 24 : 20)) {
                // blue by default

                FtcDashboard.getInstance().getTelemetry().addData("tag offset", result.getTx());
                double pidOutput = pidfController.updatePID(result.getTx());
                found = true;
                curr = result.getTx();
                if (leftLimit.isPressed() && pidOutput < 0) {
                    turretRotationServo.setPower(0);
                } else if (rightLimit.isPressed() && pidOutput > 0) {
                    turretRotationServo.setPower(0);
                } else turretRotationServo.setPower(pidOutput);

                turretMotor.setVelocity(computeVelocity());
            }
        } else {
            turretRotationServo.setPower(0);
            found = false;
        }
    }
}
