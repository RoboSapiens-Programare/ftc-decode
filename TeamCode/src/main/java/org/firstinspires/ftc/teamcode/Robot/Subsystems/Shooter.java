package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Utils.PIDFController;
import org.firstinspires.ftc.teamcode.Robot.uV;

@SuppressWarnings("FieldCanBeLocal")
@Config
public class Shooter extends Subsystem {

    public static double FIELD_ANGLE_OFFSET_DEG = -3.0;
    public static double ENCODER_GEAR_RATIO = 3.0;

    // Motoare & Servouri
    public final DcMotorEx turretMotorLeft;
    public final DcMotorEx turretMotorRight;
    public final Servo lobServo;
    public final CRServo turretPivot;
    private final Servo gate;

    // Senzori
    public Limelight3A ll;
    public final DcMotorEx turretEncoder; // Encoderul REV Through Bore

    // =========================================================
    // CONFIGURARE HARDWARE
    // =========================================================
    public static double TICKS_PER_REV = 8192.0; // REV Through Bore are 8192 ticks/rotatie

    // =========================================================
    // PIDF VALORI FLYWHEEL
    // =========================================================
    public static double shootKp = 0.07;
    public static double shootKi = 0.00002;
    public static double shootKd = 0.0000001;
    public static double shootKf = 0.013;

    // =========================================================
    // POLINOAME PENTRU RPM SI LOB (Grad 2: A*x^2 + B*x + C)
    // Tuneaza aceste valori pe teren in functie de distanta "d"
    // =========================================================
    public static double velA = -0.07055;
    public static double velB = 21.83422;
    public static double velC = 253.79189;

    public static double lobA = 0.00071;
    public static double lobB = -0.15168;
    public static double lobC = 7.99541;

    // =========================================================
    // TRACKING & SOTM CONFIG (DECODE 2025-2026)
    // =========================================================
    public static double LL_THRESHOLD_DEG = 10.0;
    public static double BALL_SPEED_INCHES = 250.0;

    public double turretErrorRad = 0.0;
    public boolean shooting = false;
    public double llDistance = 0;
    public static double targetVelocity = 1300;

    // =========================================================
    // CONTROLLERE PID
    // =========================================================
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

        // MAPARE ENCODER REV
        turretEncoder = hwMap.get(DcMotorEx.class, "rollerOne");
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidfController.setTolerance(20);
        pidfController.maxOut = 2;
        pidfController.minOut = -2;

        ll = hwMap.get(Limelight3A.class, "limelight");
        ll.pipelineSwitch(2);
        limelightTrackingController.setSetpoint(0);
        limelightTrackingController.setTolerance(1);
        ll.start();

        FtcDashboard.getInstance().startCameraStream(ll, 30);

        odometryTrackingController.setSetpoint(0);
        odometryTrackingController.setTolerance(1);

        turretPivot = hwMap.get(CRServo.class, "turretPivot");
        gate = hwMap.get(Servo.class, "gateServo");
    }

    public void openGate() {
        gate.setPosition(uV.gateOpen);
    }

    public void closeGate() {
        gate.setPosition(uV.gateClosed);
    }

    private double computeLob(double distance) {
        double lob = (lobA * distance * distance) + (lobB * distance) + lobC;
        if (lob <= uV.lobMin) return uV.lobMin;
        if (lob >= uV.lobMax) return uV.lobMax;
        return lob;
    }

    private double computeVelocity(double distance) {
        double velocity = (velA * distance * distance) + (velB * distance) + velC;
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

        double virtualX = goalX - vx * tof;
        double virtualY = goalY - vy * tof;

        double virtualDist = Math.hypot(virtualX - rx, virtualY - ry);
        double targetAngleRad = Math.atan2(virtualY - ry, virtualX - rx);

        return new double[]{targetAngleRad, virtualDist};
    }

    public double track() {
        // =========================================================
        // 1. CALCULAM MEREU DISTANTA DIN ODOMETRIE (SOTM) INTAI
        // Odometria este 100% precisa pentru distanta, o ignoram pe cea din camera
        // =========================================================
        Pose pose = Robot.follower.getPose();
        double vx = Robot.follower.getVelocity().getXComponent();
        double vy = Robot.follower.getVelocity().getYComponent();

        double[] sotm = computeVirtualGoal(pose.getX(), pose.getY(), vx, vy);
        double fieldTargetAngleRad = sotm[0];

        // AICI STABILIM DISTANTA ABSOLUTA:
        llDistance = sotm[1];

        LLResult result = ll.getLatestResult();
        double output = 0.0;
        boolean useLimelight = false;

        // =========================================================
        // 2. DACA LIMELIGHT VEDE TINTA (Doar pentru UNGHI)
        // =========================================================
        if (result != null && result.isValid()) {
            int targetId = (Robot.alliance == Robot.Alliance.RED) ? 24 : 20;
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                if (fiducial.getFiducialId() == targetId) {
                    double tx = fiducial.getTargetXDegrees();

                    useLimelight = true;

                    limelightTrackingController.kP = uV.limelightKp;
                    limelightTrackingController.kI = uV.limelightKi;
                    limelightTrackingController.kD = uV.limelightKd;
                    limelightTrackingController.kF = uV.limelightKf;

                    output = limelightTrackingController.updatePID(tx);

                    // Am sters linia cu fiducial.getCameraPose... Z !
                    // llDistance ramane cel perfect de la odometrie de mai sus!

                    turretErrorRad = Math.toRadians(tx);

                    FtcDashboard.getInstance().getTelemetry().addData("Track-State", "LIMELIGHT (Angle Only)");
                    break;
                }
            }
        }

        // =========================================================
        // 3. DACA NU VEDE CAMERA (Odometrie + REV Encoder pt Unghi)
        // =========================================================
        if (!useLimelight) {
            double currentHeading = pose.getHeading();
            double currentTurretAngleRad = (turretEncoder.getCurrentPosition() / (TICKS_PER_REV * ENCODER_GEAR_RATIO)) * (2.0 * Math.PI);

            double desiredTurretAngleRad = fieldTargetAngleRad - currentHeading + Math.toRadians(FIELD_ANGLE_OFFSET_DEG);

            while (desiredTurretAngleRad > Math.PI) desiredTurretAngleRad -= 2.0 * Math.PI;
            while (desiredTurretAngleRad < -Math.PI) desiredTurretAngleRad += 2.0 * Math.PI;

            double limitRad = Math.toRadians(80.0);

            if (desiredTurretAngleRad > limitRad) {
                desiredTurretAngleRad = limitRad;
                FtcDashboard.getInstance().getTelemetry().addData("Turret Limit", "⚠️ MAX LEFT/RIGHT");
            } else if (desiredTurretAngleRad < -limitRad) {
                desiredTurretAngleRad = -limitRad;
                FtcDashboard.getInstance().getTelemetry().addData("Turret Limit", "⚠️ MAX LEFT/RIGHT");
            } else {
                FtcDashboard.getInstance().getTelemetry().addData("Turret Limit", "✅ SAFE");
            }

            double errorRad = desiredTurretAngleRad - currentTurretAngleRad;

            while (errorRad > Math.PI) errorRad -= 2.0 * Math.PI;
            while (errorRad < -Math.PI) errorRad += 2.0 * Math.PI;

            turretErrorRad = errorRad;

            odometryTrackingController.kP = uV.odometryKp;
            odometryTrackingController.kI = uV.odometryKi;
            odometryTrackingController.kD = uV.odometryKd;
            odometryTrackingController.kF = uV.odometryKf;

            output = odometryTrackingController.updatePID(-Math.toDegrees(errorRad));

            FtcDashboard.getInstance().getTelemetry().addData("Track-State", "ODOMETRY SOTM (Encoder)");
            FtcDashboard.getInstance().getTelemetry().addData("Turret Actual (Deg)", Math.toDegrees(currentTurretAngleRad));
            FtcDashboard.getInstance().getTelemetry().addData("Turret Desired (Deg)", Math.toDegrees(desiredTurretAngleRad));
        }

        turretPivot.setPower(output);

        FtcDashboard.getInstance().getTelemetry().addData("Turret Error (Deg)", Math.toDegrees(turretErrorRad));
        FtcDashboard.getInstance().getTelemetry().update();

        return output;
    }

    @Override
    public void update() {
        pidfController.kP = shootKp;
        pidfController.kI = shootKi;
        pidfController.kD = shootKd;
        pidfController.kF = shootKf;

        track();

        if (shooting) {
            targetVelocity = computeVelocity(llDistance);
            pidfController.setSetpoint(targetVelocity);
            lobServo.setPosition(computeLob(llDistance));

            double pidOutput = pidfController.updatePID(-turretMotorLeft.getVelocity());

            FtcDashboard.getInstance().getTelemetry().addData("Shooter Target RPM", targetVelocity);
            FtcDashboard.getInstance().getTelemetry().addData("Shooter PID Out", pidOutput);
            FtcDashboard.getInstance().getTelemetry().addData("Actual Distance", llDistance);

            turretMotorRight.setPower(pidOutput / 2);
            turretMotorLeft.setPower(pidOutput / 2);
        } else {
            turretMotorRight.setPower(0.2);
            turretMotorLeft.setPower(0.2);
        }
    }
}