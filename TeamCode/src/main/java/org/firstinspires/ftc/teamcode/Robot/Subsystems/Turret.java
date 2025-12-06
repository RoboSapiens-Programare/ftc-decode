package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
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

    public static double Kp = 0.3;
    public static double Ki = 0;
    public static double Kd = 0.03;
    public static double Kf = 0; // Power to overcome inertia and friction

    public static double shootKp = 400;
    public static double shootKi = 0;
    public static double shootKd = 100;
    public static double shootKf = 13;

    double limelightMountAngleDegrees = 15.0; 

    // TODO: change to cm

    double limelightLensHeightInches = 11.65; 

    double goalHeightInches = 29.33; 

    double targetOffsetAngle_Vertical = 0;

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

    public static double velocityTolerance = 75;
    public double curr = 0;

    public PIDFController pidfController = new PIDFController(Kp, Ki, Kd, Kf);

    public static double targetVelocity;

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
        tracking = true;
    }

    public void disableCamera() {
        limelight.stop();
        tracking = false;
    }

    public boolean isShootReady() {
        boolean velo = Math.abs(turretMotor.getVelocity() - targetVelocity) < velocityTolerance;
        return velo && found;
    }

    public double getDistance() {

        targetOffsetAngle_Vertical = limelight.getLatestResult().getTy();

        angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        return distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

    }


    public void move(double power) {

        if (leftLimit.isPressed() && power > 0) {
            turretRotationServo.setPower(0);
        } else if (rightLimit.isPressed() && power < 0) {
            turretRotationServo.setPower(0);
        } else {
            turretRotationServo.setPower(power);
        }
    }

    @Override
    public void update() {
        turretMotor.setVelocityPIDFCoefficients(shootKp, shootKi, shootKd, shootKf);

        if (!tracking) {
            found = false;
            turretRotationServo.setPower(0);
            return;
        }

        final LLResult result = limelight.getLatestResult();

        if (result.isValid()) {
            for (FiducialResult fiducial : result.getFiducialResults()) {
                if (fiducial.getFiducialId() == (Robot.alliance == Robot.Alliance.RED ? 24 : 20)) {
                    double pidOutput = pidfController.updatePID(result.getTx());
                    found = true;
                    curr = result.getTx();

                    //move(pidOutput);

                    // turretMotor.setVelocity((getDistance()- 47.3) * 375 / 33.15 + 800);
                    targetVelocity = getDistance() * 37/7 + 785.67;
                    turretMotor.setVelocity(targetVelocity);

                
                    // turretMotor.setPower(0.2);

                    break;
                }
            }
        } else {
            turretRotationServo.setPower(0);
            found = false;
        }
    }
}
