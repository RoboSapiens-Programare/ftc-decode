package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Utils.PIDFController;
import org.firstinspires.ftc.teamcode.Robot.uV;

@Config
public class Shooter extends Subsystem {

    public static double FIELD_ANGLE_OFFSET_DEG = -3.0;
    public static int LL_STALE_THRESHOLD = 5;
    public static double LL_THRESHOLD_DEG = 20.0;
    public static double ENCODER_GEAR_RATIO = 3.0;
    public static double TURRET_ANGLE_OFFSET_DEG = -3.0;
    public static double TURRET_SERVO_MIDPOINT = 0.5;
    public static double TURRET_MAX_ANGLE_DEG = 90.0;
    public static double TURRET_GEAR_RATIO = 1.3;
    public static double TICKS_PER_REV = 8192.0;
    public static double shootKp = 0.06;
    public static double shootKi = 0.00002;
    public static double shootKd = 0.0000001;
    public static double shootKf = 0.013;
    public static double velA = 0.05;
    public static double velB = 4.444;
    public static double velC = 1140.2;
    public static double lobA = 0.0;
    public static double lobB = -0.01364;
    public static double lobC = 1.0682;
    public static double BALL_SPEED_INCHES = 250.0;
    public static double TURRET_AIM_THRESHOLD_DEG = 2;
    public static double targetVelocity = 1300;

    // Hardware & State
    public final DcMotorEx turretMotorLeft;
    public final DcMotorEx turretMotorRight;
    public final Servo lobServo;
    public final Servo turretPivot;
    public Limelight3A ll;
    public final DcMotorEx turretEncoder;
    public double turretServoPos = 0;
    public double LL_TURRET_OFFSET_DEG = 0.0;
    public double turretErrorRad = 0.0;
    public boolean shooting = false;
    public boolean trackCurrent = false;
    public double llDistance = 0;
    public boolean shootingLobComp = false;
    public String trackState = "IDLE";
    public double turretOutput = 0.0;

    // Internal State
    private final Servo gate;
    private double overrideAngleRad = 0.0;
    private double commandedServoPos = TURRET_SERVO_MIDPOINT;
    private int llStaleCount = 0;
    private double lastTx = Double.MAX_VALUE;
    private boolean override = false;
    private double overrideAngle = 0;

    // PID Controllers
    private final PIDFController pidfController =
            new PIDFController(shootKp, shootKi, shootKd, shootKf);
    private final PIDFController odometryTrackingController =
            new PIDFController(uV.odometryKp, uV.odometryKi, uV.odometryKd, uV.odometryKf);
    private final PIDFController limelightTrackingController =
            new PIDFController(uV.limelightKp, uV.limelightKi, uV.limelightKd, uV.limelightKf);

    private final Pose blueObeliskPose = new Pose(12, 134);
    private final Pose redObeliskPose = new Pose(133, 134);

    public Shooter(HardwareMap hwMap) {
        lobServo = hwMap.get(Servo.class, "lobServo");

        turretMotorRight = hwMap.get(DcMotorEx.class, "turretMotorRight");
        turretMotorLeft = hwMap.get(DcMotorEx.class, "turretMotorLeft");
        turretMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        turretMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretEncoder = hwMap.get(DcMotorEx.class, "rollerOne");
        turretEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidfController.setTolerance(20);
        pidfController.maxOut = 1.0;
        pidfController.minOut = -1.0;

        ll = hwMap.get(Limelight3A.class, "limelight");
        ll.pipelineSwitch(2);
        limelightTrackingController.setSetpoint(0);
        limelightTrackingController.setTolerance(1);
        ll.start();

        FtcDashboard.getInstance().startCameraStream(ll, 30);

        odometryTrackingController.setSetpoint(0);
        odometryTrackingController.setTolerance(0);

        turretPivot = hwMap.get(Servo.class, "turretPivot");
        gate = hwMap.get(Servo.class, "gateServo");
    }

    public void openGate() {
        gate.setPosition(uV.gateOpen);
    }

    public void closeGate() {
        gate.setPosition(uV.gateClosed);
    }

    // Aiming
    public boolean velocityReached() {
        double actual = -turretMotorLeft.getVelocity();
        if (llDistance < 80) {
            return Math.abs(actual - targetVelocity) < 160;
        } else {
            return Math.abs(actual - targetVelocity) < 41;
        }
    }

    public boolean isAimed() {
        if (llDistance < 60) {
            return Math.abs(Math.toDegrees(turretErrorRad)) < TURRET_AIM_THRESHOLD_DEG;
        } else {
            return Math.abs(Math.toDegrees(turretErrorRad)) < TURRET_AIM_THRESHOLD_DEG + 2;
        }
    }

    public void resetShooterPID() {
        pidfController.reset();
    }

    // Trajectory
    private double computeLob(double distance) {
        double lob = (lobA * distance * distance) + (lobB * distance) + lobC;
        if (lob <= uV.lobMin) return uV.lobMin;
        return Math.min(lob, uV.lobMax);
    }

    private double computeVelocity(double distance) {
        double velocity = (velA * distance * distance) + (velB * distance) + velC;
        if (distance > 120) {
            return 1880;
        }
        if (velocity <= 0) return 0;
        if (velocity >= 2300) return 2300;
        return velocity;
    }

    private double[] computeVirtualGoal(double rx, double ry, double vx, double vy) {
        Pose target = (Robot.alliance == Robot.Alliance.RED) ? redObeliskPose : blueObeliskPose;
        double goalX = target.getX();
        double goalY = target.getY();

        double dx = goalX - rx;
        double dy = goalY - ry;
        double rawDist = Math.hypot(dx, dy);

        double tof = rawDist / BALL_SPEED_INCHES;
        double virtualX = 0;
        double virtualY = 0;
        for (int i = 0; i < 2; i++) {
            virtualX = goalX - vx * tof;
            virtualY = goalY - vy * tof;
            double virtualDist = Math.hypot(virtualX - rx, virtualY - ry);
            tof = virtualDist / BALL_SPEED_INCHES;
        }

        double targetAngleRad = Math.atan2(virtualY - ry, virtualX - rx);
        return new double[]{targetAngleRad, rawDist};
    }

    // Turret Conversion
    public double getTargetFieldAngleRadStatic() {
        Pose pose = Robot.follower.getPose();
        Pose target = (Robot.alliance == Robot.Alliance.RED) ? redObeliskPose : blueObeliskPose;
        double dx = target.getX() - pose.getX();
        double dy = target.getY() - pose.getY();
        return Math.atan2(dy, dx);
    }

    private double servoPositionFromTurretAngle(double turretAngleRad) {
        double turretDeg = Math.toDegrees(turretAngleRad);
        turretDeg = Math.max(-TURRET_MAX_ANGLE_DEG, Math.min(TURRET_MAX_ANGLE_DEG, turretDeg));
        double servoDeg = turretDeg / TURRET_GEAR_RATIO;
        return TURRET_SERVO_MIDPOINT + (servoDeg / TURRET_MAX_ANGLE_DEG) * 0.5;
    }

    private double turretAngleFromServoPosition(double servoPos) {
        double servoDeg = (servoPos - TURRET_SERVO_MIDPOINT) * TURRET_MAX_ANGLE_DEG / 0.5;
        return Math.toRadians(servoDeg * TURRET_GEAR_RATIO);
    }

    // Limelight Tracking
    private double trackWithLimelight() {
        LLResult result = ll.getLatestResult();
        if (result == null || !result.isValid()) {
            return Double.NaN;
        }

        int targetId = (Robot.alliance == Robot.Alliance.RED) ? 24 : 20;
        for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
            if (fiducial.getFiducialId() != targetId) continue;

            double tx = fiducial.getTargetXDegrees();

            if (tx != lastTx) {
                lastTx = tx;
                llStaleCount = 0;
            } else {
                llStaleCount++;
            }

            if (llStaleCount >= LL_STALE_THRESHOLD || Math.abs(tx) >= LL_THRESHOLD_DEG) {
                return Double.NaN;
            }

//            double correctedTx = tx + LL_TURRET_OFFSET_DEG;
            turretErrorRad = Math.toRadians(-tx);
            trackState = "LIMELIGHT tx=" + tx;

            double currentTurretRad = turretAngleFromServoPosition(commandedServoPos);
            return servoPositionFromTurretAngle(currentTurretRad + Math.toRadians(tx));
        }

        return Double.NaN;
    }

    // Odometry Tracking (fallback)
    private static double normalizeRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    private double trackWithOdometry(double fieldTargetAngleRad, double robotHeading) {
        double desiredAngleRad = fieldTargetAngleRad - robotHeading
                + Math.toRadians(TURRET_ANGLE_OFFSET_DEG);
        desiredAngleRad = normalizeRadians(desiredAngleRad);

        double limitRad = Math.toRadians(TURRET_MAX_ANGLE_DEG);
        desiredAngleRad = Math.max(-limitRad, Math.min(limitRad, desiredAngleRad));

        double currentTurretAngleRad = turretAngleFromServoPosition(commandedServoPos);
        turretErrorRad = normalizeRadians(desiredAngleRad - currentTurretAngleRad);

        trackState = "ODOMETRY";
        return servoPositionFromTurretAngle(desiredAngleRad);
    }

    // Main Tracking
    public double track() {
        Pose pose = Robot.follower.getPose();
        double vx = Robot.follower.getVelocity().getXComponent();
        double vy = Robot.follower.getVelocity().getYComponent();
        double heading = pose.getHeading();

        double vxField = vx * Math.cos(heading) - vy * Math.sin(heading);
        double vyField = vx * Math.sin(heading) + vy * Math.cos(heading);

        double[] sotm = computeVirtualGoal(pose.getX(), pose.getY(), vxField, vyField);
        double fieldTargetAngleRad = sotm[0];
        llDistance = sotm[1];

        double servoPos = trackWithLimelight();

        if (Double.isNaN(servoPos)) {
            servoPos = trackWithOdometry(fieldTargetAngleRad, heading);
        }

        servoPos = Math.max(0.0, Math.min(1.0, servoPos));
        if (!override) {
            turretPivot.setPosition(servoPos);
            commandedServoPos = servoPos;
        }

        turretServoPos = servoPos;
        return servoPos;
    }

    // Override Controls
    @Override
    public void reset() {
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void goToAngle(double rad) {
        override = true;

        double limitRad = Math.toRadians(1000.0);
        if (rad > limitRad) {
            rad = limitRad;
        } else if (rad < -limitRad) {
            rad = -limitRad;
        }

        overrideAngle = rad;
    }

    public void stopOverride() {
        override = false;
    }

    public void incremental(double rad) {
        if (!override) {
            overrideAngle = (turretEncoder.getCurrentPosition()
                    / (TICKS_PER_REV * ENCODER_GEAR_RATIO)) * (2.0 * Math.PI);
        }
        goToAngle(rad + overrideAngle);
    }

    public void lock() {
        goToAngle(overrideAngle);
    }

    // Update Loop
    @Override
    public void update() {
        pidfController.kP = shootKp;
        pidfController.kI = shootKi;
        pidfController.kD = shootKd;
        pidfController.kF = shootKf;

        if (override) {
            double pos = servoPositionFromTurretAngle(overrideAngleRad);
            pos = Math.max(0.0, Math.min(1.0, pos));
            turretPivot.setPosition(pos);
            commandedServoPos = pos;
        }

        if (shooting) {
            track();

            targetVelocity = computeVelocity(llDistance);
            pidfController.setSetpoint(targetVelocity);
            if (!shootingLobComp) {
                lobServo.setPosition(computeLob(llDistance));
            }

            double pidOutput = pidfController.updatePID(-turretMotorLeft.getVelocity());

            turretMotorRight.setPower(-pidOutput);
            turretMotorLeft.setPower(-pidOutput);

        } else {
            if (!override) {
                turretMotorRight.setPower(-0.2);
                turretMotorLeft.setPower(-0.2);
            }
        }
    }
}
