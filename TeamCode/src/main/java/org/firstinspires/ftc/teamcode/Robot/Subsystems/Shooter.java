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
    public static double ENCODER_GEAR_RATIO     = 3.0;

    // =========================================================
    // Motoare & Servouri
    // =========================================================
    public final DcMotorEx turretMotorLeft;
    public final DcMotorEx turretMotorRight;
    public final Servo     lobServo;
    public final CRServo   turretPivot;
    private final Servo    gate;

    // =========================================================
    // Senzori
    // =========================================================
    public Limelight3A     ll;
    public final DcMotorEx turretEncoder;

    // =========================================================
    // CONFIGURARE HARDWARE
    // =========================================================
    public static double TICKS_PER_REV = 8192.0;

    // =========================================================
    // PIDF VALORI FLYWHEEL
    // =========================================================
    public static double shootKp = 0.06;
    public static double shootKi = 0.00002;
    public static double shootKd = 0.0000001;
    public static double shootKf = 0.013;

    // =========================================================
    // POLINOAME PENTRU RPM SI LOB (Grad 2: A*x^2 + B*x + C)
    // =========================================================
    public static double velA =  0.0;
    public static double velB =  4.444;
    public static double velC =  1140.2;
    public static double lobA =  0.0;
    public static double lobB = -0.01364;
    public static double lobC =  1.0682;
    // =========================================================
    // TRACKING & SOTM CONFIG
    // =========================================================
    public static double LL_THRESHOLD_DEG     = 20.0;
    public double LL_TURRET_OFFSET_DEG =  0.0;
    public static double BALL_SPEED_INCHES    = 250.0;
    public static double TURRET_AIM_THRESHOLD_DEG = 3;

    // =========================================================
    // HYSTERESIS LIMELIGHT
    // Robot ~100Hz, LL ~30fps → ~3 loop-uri intre frame-uri LL.
    // Fara hysteresis → switch constant LL<->Odometrie → reset
    // integral → oscilatie stanga-dreapta.
    // LL_STALE_THRESHOLD = 5 loop-uri = ~50ms buffer de siguranta.
    // =========================================================
    public static int LL_STALE_THRESHOLD = 5;
    private int       llStaleCount       = 0;
    private double    lastTx             = Double.MAX_VALUE;

    // =========================================================
    // STATE PUBLIC — citit din TeleOp pentru telemetry
    // =========================================================
    public double  turretErrorRad = 0.0;
    public boolean shooting       = false;
    public boolean trackCurrent = false;
    public double  llDistance     = 0;
    public String  trackState     = "IDLE";
    public double  turretOutput   = 0.0;
    public static double targetVelocity = 1300;

    // =========================================================
    // CONTROLLERE PID
    // =========================================================
    private final PIDFController pidfController =
            new PIDFController(shootKp, shootKi, shootKd, shootKf);

    //TODO DECOMENTAT
    private final PIDFController odometryTrackingController =
            new PIDFController(uV.odometryKp, uV.odometryKi, uV.odometryKd, uV.odometryKf);

    private final PIDFController limelightTrackingController =
            new PIDFController(uV.limelightKp, uV.limelightKi, uV.limelightKd, uV.limelightKf);

    private final Pose blueObeliskPose = new Pose(12, 134);
    private final Pose redObeliskPose  = new Pose(133, 134);

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

        turretEncoder = hwMap.get(DcMotorEx.class, "rollerOne");
        turretEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidfController.setTolerance(20);
        pidfController.maxOut =  1.0;
        pidfController.minOut = -1.0;

        ll = hwMap.get(Limelight3A.class, "limelight");
        ll.pipelineSwitch(2);
        limelightTrackingController.setSetpoint(0);
        limelightTrackingController.setTolerance(1);
        ll.start();

        FtcDashboard.getInstance().startCameraStream(ll, 30);

        odometryTrackingController.setSetpoint(0);
        odometryTrackingController.setTolerance(14);

        turretPivot = hwMap.get(CRServo.class, "turretPivot");

        turretPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        gate        = hwMap.get(Servo.class, "gateServo");
    }

    // =========================================================
    // GATE
    // =========================================================
    public void openGate()  { gate.setPosition(uV.gateOpen);   }
    public void closeGate() { gate.setPosition(uV.gateClosed); }

    // =========================================================
    // VELOCITY REACHED
    // =========================================================
    public boolean velocityReached() {
        double actual = -turretMotorLeft.getVelocity();
        if(llDistance < 80)
        {
//            if (actual - targetVelocity > -80 && actual - targetVelocity < 0)
//            {
//                return true;
//            }
                return (Math.abs(actual - targetVelocity ))<120;
            // actual e mai mare -> actual-target negativ
        } else {
            return Math.abs(actual - targetVelocity) < 41;
        }
    }

    // =========================================================
    // IS AIMED
    // =========================================================
    public boolean isAimed() {
        if (llDistance<60)
        {
            return Math.abs(Math.toDegrees(turretErrorRad)) < TURRET_AIM_THRESHOLD_DEG;
        }
        else {
            return Math.abs(Math.toDegrees(turretErrorRad)) < TURRET_AIM_THRESHOLD_DEG+2;
        }
//        return true;
    }

    // =========================================================
    // RESET PID FLYWHEEL
    // =========================================================
    public void resetShooterPID() {
        pidfController.reset();
    }

    // =========================================================
    // COMPUTE LOB & VELOCITY
    // =========================================================
    private double computeLob(double distance) {
        double lob = (lobA * distance * distance) + (lobB * distance) + lobC;
        if (lob <= uV.lobMin) return uV.lobMin;
        if (lob >= uV.lobMax) return uV.lobMax;
        return lob;
    }

    private double computeVelocity(double distance) {
        double velocity = (velA * distance * distance) + (velB * distance) + velC;
        if (distance>80)
        {
            return 1880;
        }
        if (velocity <= 0)    return 0;
        if (velocity >= 2300) return 2300;
        return velocity;
    }

    // =========================================================
    // SOTM — Shot on the Move
    // Returneaza: [targetAngleRad, rawDist]
    // =========================================================
    private double[] computeVirtualGoal(double rx, double ry, double vx, double vy) {
        Pose   target = (Robot.alliance == Robot.Alliance.RED) ? redObeliskPose : blueObeliskPose;
        double goalX  = target.getX();
        double goalY  = target.getY();

        double dx      = goalX - rx;
        double dy      = goalY - ry;
        double rawDist = Math.hypot(dx, dy);

        double tof      = rawDist / BALL_SPEED_INCHES;
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

    // =========================================================
    // TRACK — turret tracking principal
    // =========================================================
    public double track() {

        PIDFController odometryTrackingController =
                new PIDFController(uV.odometryKp, uV.odometryKi, uV.odometryKd, uV.odometryKf);

        PIDFController limelightTrackingController =
                new PIDFController(uV.limelightKp, uV.limelightKi, uV.limelightKd, uV.limelightKf);
        // -------------------------------------------------------
        // 1. POZITIE & VITEZA — rotire in field frame
        // -------------------------------------------------------
        Pose   pose    = Robot.follower.getPose();
        double vx      = Robot.follower.getVelocity().getXComponent();
        double vy      = Robot.follower.getVelocity().getYComponent();
        double heading = pose.getHeading();

        double vxField = vx * Math.cos(heading) - vy * Math.sin(heading);
        double vyField = vx * Math.sin(heading) + vy * Math.cos(heading);

        double[] sotm            = computeVirtualGoal(pose.getX(), pose.getY(), vxField, vyField);
        double   fieldTargetAngleRad = sotm[0];
        llDistance               = sotm[1];

        // -------------------------------------------------------
        // 2. LIMELIGHT cu hysteresis
        // Nu resetam integratorul la tranzitii — cauzeaza spike/oscilatie
        // -------------------------------------------------------
        LLResult result       = ll.getLatestResult();
        double   output       = 0.0;
        boolean  useLimelight = false;

        if (result != null && result.isValid()) {
            int targetId = (Robot.alliance == Robot.Alliance.RED) ? 24 : 20;

            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                if (fiducial.getFiducialId() == targetId) {
                    double tx = fiducial.getTargetXDegrees();

                    if (tx != lastTx) {
                        // Frame nou de la limelight — reset contor stale
                        lastTx       = tx;
                        llStaleCount = 0;
                    } else {
                        // Acelasi frame — incrementam contorul
                        llStaleCount++;
                    }

                    // LL activ daca frame recent SI tinta in frame
                    if (llStaleCount < LL_STALE_THRESHOLD && Math.abs(tx) < LL_THRESHOLD_DEG) {
                        useLimelight = true;

                        if (llDistance>80)
                        {
                            limelightTrackingController.kP = uV.limelightKpF;
                            limelightTrackingController.kI = uV.limelightKiF;
                            limelightTrackingController.kD = uV.limelightKdF;
                            limelightTrackingController.kF = uV.limelightKfF;
                        } else {
                            limelightTrackingController.kP = uV.limelightKp;
                            limelightTrackingController.kI = uV.limelightKi;
                            limelightTrackingController.kD = uV.limelightKd;
                            limelightTrackingController.kF = uV.limelightKf;
                        }

                        double txCorrected = tx + LL_TURRET_OFFSET_DEG;

                            output = limelightTrackingController.updatePID(txCorrected);

                        turretErrorRad = Math.toRadians(txCorrected);
                        turretOutput   = output;
                        trackState     = "LIMELIGHT tx=" + tx;
                    }
                    break;
                }
            }
        }



        // -------------------------------------------------------
        // 3. ODOMETRIE — fallback cand LL nu vede sau e stale
        // -------------------------------------------------------
        if (!useLimelight) {
            double currentHeading        = pose.getHeading();
            double currentTurretAngleRad = (turretEncoder.getCurrentPosition()
                    / (TICKS_PER_REV * ENCODER_GEAR_RATIO)) * (2.0 * Math.PI);

            double desiredTurretAngleRad = fieldTargetAngleRad - currentHeading
                    + Math.toRadians(FIELD_ANGLE_OFFSET_DEG);

            while (desiredTurretAngleRad >  Math.PI) desiredTurretAngleRad -= 2.0 * Math.PI;
            while (desiredTurretAngleRad < -Math.PI) desiredTurretAngleRad += 2.0 * Math.PI;

            double limitRad = Math.toRadians(80.0);
            if (desiredTurretAngleRad > limitRad) {
                desiredTurretAngleRad = limitRad;
            } else if (desiredTurretAngleRad < -limitRad) {
                desiredTurretAngleRad = -limitRad;
            }

            double errorRad = desiredTurretAngleRad - currentTurretAngleRad;

            while (errorRad >  Math.PI) errorRad -= 2.0 * Math.PI;
            while (errorRad < -Math.PI) errorRad += 2.0 * Math.PI;

            turretErrorRad = errorRad;

            if (llDistance>80)
            {
                odometryTrackingController.kP = uV.odometryKpF;
                odometryTrackingController.kI = uV.odometryKiF;
                odometryTrackingController.kD = uV.odometryKdF;
                odometryTrackingController.kF = uV.odometryKfF;
            } else {
                odometryTrackingController.kP = uV.odometryKp;
                odometryTrackingController.kI = uV.odometryKi;
                odometryTrackingController.kD = uV.odometryKd;
                odometryTrackingController.kF = uV.odometryKf;
            }



            // Semn pastrat exact ca in versiunea originala
                output       = odometryTrackingController.updatePID(-Math.toDegrees(errorRad));

            turretOutput = output; // actualizat DUPA calculul output
            trackState   = "ODOMETRY";
        }


        output = Math.max(-1.0, Math.min(1.0, output));

        turretPivot.setPower(output);

        return output;
    }

    // =========================================================
    // UPDATE — apelat in fiecare loop
    // FtcDashboard scos din update() si track() pentru a elimina
    // lag-ul la sasiu — telemetry e gestionat din TeleOp
    // =========================================================
    @Override
    public void update() {
        pidfController.kP = shootKp;
        pidfController.kI = shootKi;
        pidfController.kD = shootKd;
        pidfController.kF = shootKf;

        if (shooting) {
            track();


            targetVelocity = computeVelocity(llDistance);
//            targetVelocity = 0;
            pidfController.setSetpoint(targetVelocity);
            lobServo.setPosition(computeLob(llDistance));

            double pidOutput = pidfController.updatePID(-turretMotorLeft.getVelocity());

            turretMotorRight.setPower(pidOutput);
            turretMotorLeft.setPower(pidOutput);

        } else {
            turretMotorRight.setPower(0.2);
            turretMotorLeft.setPower(0.2);
            turretPivot.setPower(0);
        }
    }
}