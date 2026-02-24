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
    public final DcMotorEx turretMotorLeft;
    public final DcMotorEx turretMotorRight;

    private final Servo lobServo;
    private final CRServo turretPivot;

    private final Limelight3A ll;

    // PID values for shooter

    public static double shootKp = 0.07;
    public static double shootKi = 0.00002;
    public static double shootKd = 0.0000001;
    public static double shootKf = 0.013;

    private final PIDFController pidfController =
            new PIDFController(shootKp, shootKi, shootKd, shootKf);

    private final PIDFController odometryTrackingController = new PIDFController(uV.odometryKp, uV.odometryKi, uV.odometryKd, uV.odometryKf);
    private final PIDFController limelightTrackingController = new PIDFController(uV.limelightKp, uV.limelightKi, uV.limelightKd, uV.limelightKf);

    private final Pose blueObeliskPose = new Pose(12, 134);
    private final Pose redObeliskPose = new Pose(133, 134);

    public static double targetVelocity = 1300;

    public boolean isTracking = false;

    public boolean shooting = false;

    private boolean shouldFollowTrack = true;

    public Shooter(HardwareMap hwMap) {
        lobServo = hwMap.get(Servo.class, "lobServo");

        turretMotorRight = hwMap.get(DcMotorEx.class, "turretMotorRight");
        turretMotorLeft = hwMap.get(DcMotorEx.class, "turretMotorLeft");
        turretMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        turretMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidfController.setTolerance(20);
        pidfController.maxOut = 2;
        pidfController.minOut = -2;

        ll = hwMap.get(Limelight3A.class, "limelight");
        ll.pipelineSwitch(2);
        limelightTrackingController.setSetpoint(0);
        limelightTrackingController.setTolerance(0.3);

        odometryTrackingController.setTolerance(20);

        turretPivot = hwMap.get(CRServo.class, "turretPivot");
    }

    public boolean isShootReady() {
        double tolerance = Math.toRadians(2);
        //        boolean aligned =

        // TODO: implement pivoting turret here

        FtcDashboard.getInstance()
                .getTelemetry()
                .addData("Angle delta", Math.toDegrees(Robot.follower.getHeading() - getAngle()));

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

        return 0;

        // should output a servo value (0 -> 1)
        // modify with telemetry for best results and change formula
//        return dist * Math.pow(1, -100);
    }

    private double computeVelocity() {
//        double dist = computeDistance();
//        if (dist > 100) {
//            return 1400;
//        }
//        if (dist < 65) {
//            return 1100;
//        }
//        return dist * 1.42 + 1060;

        return 2700;
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
        LLResult result = ll.getLatestResult();

        boolean found = false;
        double output = 0;

        if (result.isValid()) {
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                if (fiducial.getFiducialId() == (Robot.alliance == Robot.Alliance.RED ? 24 : 20)) {
                    output = limelightTrackingController.updatePID(result.getTx());

                    found = true;
                }
            }
        }

        if (!found) {
            double delta = getAngle() - Robot.follower.getHeading();

            double deltaTicks = delta * 2048 / Math.PI;

            odometryTrackingController.setSetpoint(deltaTicks);

            output = odometryTrackingController.updatePID(turretMotorRight.getCurrentPosition());
        }

        turretPivot.setPower(output);

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
            double pidOutput = pidfController.updatePID(turretMotorLeft.getVelocity());

            FtcDashboard.getInstance().getTelemetry().addData("pid vel", pidOutput);
            turretMotorRight.setPower(pidOutput / 2);
            turretMotorLeft.setPower(pidOutput / 2);

            lobServo.setPosition(computeLob());
        } else {
            turretMotorRight.setPower(0);
            turretMotorLeft.setPower(0);
        }

        if (isTracking) {
            track();
        }
    }
}
