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

    // =========================================================
    // HARDWARE
    // =========================================================
    public final DcMotorEx turretMotorLeft;
    public final DcMotorEx turretMotorRight;
    public final Servo     lobServo;
    public final Servo     turretPivot;
    private final Servo    gate;
    public  final Limelight3A limelight;

    // =========================================================
    // TURNABLE CONSTANTS (live-tuneable via FTC Dashboard @Config)
    // =========================================================

    // Turret servo geometry
    public static double TURRET_ANGLE_OFFSET_DEG = -3.0;
    public static double TURRET_SERVO_MIDPOINT   = 0.5;
    public static double TURRET_MAX_ANGLE_DEG    = 90.0;
    public static double TURRET_GEAR_RATIO       = 1.3;

    // Flywheel PIDF gains
    public static double FLYWHEEL_KP = 0.06;
    public static double FLYWHEEL_KI = 0.00002;
    public static double FLYWHEEL_KD = 0.0000001;
    public static double FLYWHEEL_KF = 0.013;

    // Lob polynomial:  A * dist^2 + B * dist + C
    public static double LOB_A =  0.0;
    public static double LOB_B = -0.01364;
    public static double LOB_C =  1.0682;

    // Velocity polynomial:  A * dist^2 + B * dist + C
    public static double VEL_A =  0.0;
    public static double VEL_B =  4.444;
    public static double VEL_C =  1140.2;

    // Limelight tracking
    public static double LL_MAX_ERROR_DEG       = 20.0;
    public double       LL_TURRET_OFFSET_DEG    =  0.0;
    public static int   LL_STALE_FRAME_LIMIT    = 5;

    // SOTM (Shot-On-The-Move)
    public static double BALL_SPEED_INCHES_PER_SEC = 250.0;

    // Aim readiness thresholds
    public static double TURRET_AIM_THRESHOLD_DEG = 3.0;

    // =========================================================
    // OBJECT POOLS (avoid GC allocations in hot loop)
    // =========================================================
    private final double[] sotmResult = new double[2];

    // =========================================================
    // FLYWHEEL PID CONTROLLER
    // =========================================================
    private final PIDFController flywheelPID =
            new PIDFController(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD, FLYWHEEL_KF);

    // =========================================================
    // TARGET POSES (field-frame obelisk coordinates)
    // =========================================================
    private static final Pose BLUE_OBELISK = new Pose(12, 134);
    private static final Pose RED_OBELISK  = new Pose(133, 134);

    // =========================================================
    // INTERNAL STATE
    // =========================================================
    private boolean overrideActive = false;
    private double  overrideAngleRad = 0.0;
    private double  commandedServoPos = TURRET_SERVO_MIDPOINT;

    // Limelight hysteresis counters
    private int    llStaleFrames = 0;
    private double lastTxDeg     = Double.MAX_VALUE;

    // Exposed for external reads (TeleOp telemetry, Auto checks)
    public double  turretErrorRad     = 0.0;
    public double  llDistance         = 0.0;
    public boolean shooting           = false;
    public boolean shootingLobComp    = false;
    public double  turretServoPos     = 0.0;
    public double  targetVelocity     = 1300.0;
    public String  trackState         = "IDLE";

    // =========================================================
    // CONSTRUCTOR
    // =========================================================
    public Shooter(HardwareMap hwMap) {
        lobServo = hwMap.get(Servo.class, "lobServo");

        turretMotorRight = hwMap.get(DcMotorEx.class, "turretMotorRight");
        turretMotorLeft  = hwMap.get(DcMotorEx.class, "turretMotorLeft");
        turretMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        turretMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelPID.setTolerance(20);
        flywheelPID.maxOut =  1.0;
        flywheelPID.minOut = -1.0;

        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        limelight.start();
        FtcDashboard.getInstance().startCameraStream(limelight, 30);

        turretPivot = hwMap.get(Servo.class, "turretPivot");
        turretPivot.setDirection(Servo.Direction.REVERSE);

        gate = hwMap.get(Servo.class, "gateServo");
    }

    // =========================================================
    // FEEDER GATE
    // =========================================================
    public void openGate()  { gate.setPosition(uV.gateOpen);   }
    public void closeGate() { gate.setPosition(uV.gateClosed); }

    // =========================================================
    // FLYWHEEL READINESS
    // =========================================================
    public boolean velocityReached() {
        double speed = -turretMotorLeft.getVelocity();
        double tolerance = (llDistance < 80) ? 160.0 : 41.0;
        return Math.abs(speed - targetVelocity) < tolerance;
    }

    // =========================================================
    // TURRET AIM CHECK
    // =========================================================
    public boolean isAimed() {
        double threshold = (llDistance < 60)
                ? TURRET_AIM_THRESHOLD_DEG
                : TURRET_AIM_THRESHOLD_DEG + 2.0;
        return Math.abs(Math.toDegrees(turretErrorRad)) < threshold;
    }

    // =========================================================
    // PID RESET
    // =========================================================
    public void resetFlywheelPID() {
        flywheelPID.reset();
    }

    /** @deprecated Use {@link #resetFlywheelPID()} */
    @Deprecated
    public void resetShooterPID() { resetFlywheelPID(); }

    // =========================================================
    // LOB SERVO POSITION
    // =========================================================
    private double computeLobPosition(double distanceInches) {
        double position = (LOB_A * distanceInches + LOB_B) * distanceInches + LOB_C;
        if (position <= uV.lobMin) return uV.lobMin;
        if (position >= uV.lobMax) return uV.lobMax;
        return position;
    }

    // =========================================================
    // FLYWHEEL VELOCITY
    // =========================================================
    private double computeFlywheelVelocity(double distanceInches) {
        double velocity = (VEL_A * distanceInches + VEL_B) * distanceInches + VEL_C;
        if (distanceInches > 120) return 1880.0;
        if (velocity <= 0)        return 0.0;
        if (velocity >= 2300)     return 2300.0;
        return velocity;
    }

    // =========================================================
    // SERVO GEOMETRY
    // =========================================================
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

    // =========================================================
    // ANGLE NORMALIZATION
    // =========================================================
    private static double normalizeRadians(double angle) {
        while (angle >  Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    // =========================================================
    // CHASSIS TARGET ANGLE (field-static)
    // =========================================================
    public double getTargetFieldAngleRad() {
        Pose robot  = Robot.follower.getPose();
        Pose target = (Robot.alliance == Robot.Alliance.RED) ? RED_OBELISK : BLUE_OBELISK;
        return Math.atan2(target.getY() - robot.getY(), target.getX() - robot.getX());
    }

    /** @deprecated Use {@link #getTargetFieldAngleRad()} */
    @Deprecated
    public double getTargetFieldAngleRadStatic() { return getTargetFieldAngleRad(); }

    // =========================================================
    // SOTM — Shot-On-The-Move
    //
    // Computes the field-frame angle to the obelisk, corrected
    // for robot velocity during the ball's time-of-flight.
    //
    // Populates sotmResult[0] = target angle (radians)
    //            sotmResult[1] = raw distance (inches)
    // =========================================================
    private void computeSOTM(double robotX, double robotY,
                             double velX, double velY) {
        Pose targetPose = (Robot.alliance == Robot.Alliance.RED)
                ? RED_OBELISK : BLUE_OBELISK;

        double goalX = targetPose.getX();
        double goalY = targetPose.getY();

        double dx = goalX - robotX;
        double dy = goalY - robotY;
        double rawDist = Math.sqrt(dx * dx + dy * dy);

        // Iterative time-of-flight correction (two iterations is enough
        // when robot speed << ball speed).
        double timeOfFlight = rawDist / BALL_SPEED_INCHES_PER_SEC;
        double virtualX = goalX;
        double virtualY = goalY;
        for (int i = 0; i < 2; i++) {
            virtualX = goalX - velX * timeOfFlight;
            virtualY = goalY - velY * timeOfFlight;
            double vDist = Math.sqrt(
                    (virtualX - robotX) * (virtualX - robotX)
                  + (virtualY - robotY) * (virtualY - robotY));
            timeOfFlight = vDist / BALL_SPEED_INCHES_PER_SEC;
        }

        sotmResult[0] = Math.atan2(virtualY - robotY, virtualX - robotX);
        sotmResult[1] = rawDist;
    }

    // =========================================================
    // LIMELIGHT TRACKING (with stale-frame hysteresis)
    //
    // Returns the servo position if a valid target is found,
    // or NaN to signal "no valid LL data".
    // =========================================================
    private double trackWithLimelight() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return Double.NaN;
        }

        int targetId = (Robot.alliance == Robot.Alliance.RED) ? 24 : 20;
        for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
            if (fiducial.getFiducialId() != targetId) continue;

            double tx = fiducial.getTargetXDegrees();

            // Track whether we are getting new frames or a stale one
            if (tx != lastTxDeg) {
                lastTxDeg    = tx;
                llStaleFrames = 0;
            } else {
                llStaleFrames++;
            }

            // Reject stale frames and targets outside the FOV threshold
            if (llStaleFrames >= LL_STALE_FRAME_LIMIT
                    || Math.abs(tx) >= LL_MAX_ERROR_DEG) {
                return Double.NaN;
            }

            double correctedTx = tx + LL_TURRET_OFFSET_DEG;
            turretErrorRad = Math.toRadians(correctedTx);
            trackState = "LIMELIGHT tx=" + tx;

            double currentTurretRad = turretAngleFromServoPosition(commandedServoPos);
            return servoPositionFromTurretAngle(currentTurretRad + Math.toRadians(correctedTx));
        }

        return Double.NaN;
    }

    // =========================================================
    // ODOMETRY TRACKING (fallback when Limelight is unavailable)
    // =========================================================
    private double trackWithOdometry(double fieldTargetAngleRad,
                                     double robotHeading) {
        double desiredAngleRad = fieldTargetAngleRad - robotHeading
                + Math.toRadians(TURRET_ANGLE_OFFSET_DEG);
        desiredAngleRad = normalizeRadians(desiredAngleRad);

        // Clamp to physical turret range
        double limitRad = Math.toRadians(TURRET_MAX_ANGLE_DEG);
        desiredAngleRad = Math.max(-limitRad, Math.min(limitRad, desiredAngleRad));

        double currentTurretAngleRad = turretAngleFromServoPosition(commandedServoPos);
        turretErrorRad = normalizeRadians(desiredAngleRad - currentTurretAngleRad);

        trackState = "ODOMETRY";
        return servoPositionFromTurretAngle(desiredAngleRad);
    }

    // =========================================================
    // TRACK — main turret-aiming loop
    //
    // Priority:  Limelight (precise) → Odometry + SOTM (fallback)
    // =========================================================
    public double track() {
        // --- 1. Read robot state ---
        Pose   robot   = Robot.follower.getPose();
        double robotVx = Robot.follower.getVelocity().getXComponent();
        double robotVy = Robot.follower.getVelocity().getYComponent();
        double heading = robot.getHeading();

        // Rotate velocity from robot-relative to field-frame
        double fieldVx = robotVx * Math.cos(heading) - robotVy * Math.sin(heading);
        double fieldVy = robotVx * Math.sin(heading) + robotVy * Math.cos(heading);

        // --- 2. Compute SOTM field-target angle ---
        computeSOTM(robot.getX(), robot.getY(), fieldVx, fieldVy);
        double fieldTargetAngleRad = sotmResult[0];
        llDistance = sotmResult[1];

        // --- 3. Try Limelight first ---
        double servoPos = trackWithLimelight();

        // --- 4. Fall back to Odometry + SOTM ---
        if (Double.isNaN(servoPos)) {
            servoPos = trackWithOdometry(fieldTargetAngleRad, heading);
        }

        // --- 5. Apply to servo ---
        servoPos = Math.max(0.0, Math.min(1.0, servoPos));
        if (!overrideActive) {
            turretPivot.setPosition(servoPos);
            commandedServoPos = servoPos;
        }

        turretServoPos = servoPos;
        return servoPos;
    }

    // =========================================================
    // OVERRIDE SYSTEM
    // =========================================================
    public void goToAngle(double angleRad) {
        overrideActive = true;
        double limitRad = Math.toRadians(TURRET_MAX_ANGLE_DEG);
        overrideAngleRad = Math.max(-limitRad, Math.min(limitRad, angleRad));
    }

    public void stopOverride() {
        overrideActive = false;
    }

    public void incremental(double deltaRad) {
        if (!overrideActive) {
            overrideAngleRad = turretAngleFromServoPosition(commandedServoPos);
        }
        goToAngle(deltaRad + overrideAngleRad);
    }

    public void lock() {
        goToAngle(overrideAngleRad);
    }

    // =========================================================
    // SUBSYSTEM LIFECYCLE
    // =========================================================
    @Override
    public void reset() {
        commandedServoPos = TURRET_SERVO_MIDPOINT;
    }

    @Override
    public void update() {
        // Sync PID gains from Dashboard live-tuning
        flywheelPID.kP = FLYWHEEL_KP;
        flywheelPID.kI = FLYWHEEL_KI;
        flywheelPID.kD = FLYWHEEL_KD;
        flywheelPID.kF = FLYWHEEL_KF;

        // --- Override: direct angle control ---
        if (overrideActive) {
            double pos = servoPositionFromTurretAngle(overrideAngleRad);
            pos = Math.max(0.0, Math.min(1.0, pos));
            turretPivot.setPosition(pos);
            commandedServoPos = pos;
        }

        // --- Shooting: tracking + flywheel + lob ---
        if (shooting) {
            track();

            targetVelocity = computeFlywheelVelocity(llDistance);
            flywheelPID.setSetpoint(targetVelocity);

            if (!shootingLobComp) {
                lobServo.setPosition(computeLobPosition(llDistance));
            }

            double power = flywheelPID.updatePID(-turretMotorLeft.getVelocity());
            turretMotorRight.setPower(power);
            turretMotorLeft.setPower(power);

        } else if (!overrideActive) {
            // Idle: keep flywheel spinning slowly
            turretMotorRight.setPower(0.2);
            turretMotorLeft.setPower(0.2);
        }
    }
}
