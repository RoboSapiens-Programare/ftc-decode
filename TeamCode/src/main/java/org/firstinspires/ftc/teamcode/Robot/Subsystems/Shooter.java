package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Utils.PIDFController;
import org.firstinspires.ftc.teamcode.Robot.uV;
import org.jetbrains.annotations.Contract;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Config
public class Shooter extends Subsystem {

    private static final double WHEEL_RADIUS_INCHES = 1.4567; // 74mm diameter / 2
    private static final double RAW_MOTOR_TICKS = 28.0;      // 1:1 kit bypasses gearbox completely
    private static final double EXTERNAL_GEAR_RATIO = 1.0;   // Change if you use external gears/pulleys

    private static double TICKS_TO_INCHES = (2.0 * Math.PI * WHEEL_RADIUS_INCHES) / (RAW_MOTOR_TICKS * EXTERNAL_GEAR_RATIO);
    public static double TURRET_SERVO_MIDPOINT = 0.5;
    public static double TURRET_MAX_ANGLE_DEG = 90.0;
    public static double TURRET_GEAR_RATIO = 1.3;
    public static double shootKp = 0.06;
    public static double shootKi = 0.00002;
    public static double shootKd = 0.0000001;
    public static double shootKf = 0.013;
    public static double TURRET_AIM_THRESHOLD_DEG = 2;
    public static double targetVelocity = 1300;
    public static double targetLob = 0;

    // Hardware & State
    public final CachingDcMotorEx turretMotorLeft;
    public final CachingDcMotorEx turretMotorRight;
    public final CachingServo lobServo;
    public final CachingServo turretPivot;
    public double turretServoPos = 0;
    public double LL_TURRET_OFFSET_DEG = 0.0;
    public double turretErrorRad = 0.0;
    public boolean shooting = false;
    public double distance = 0;
    public boolean shootingLobComp = true;

    public boolean targetSelected = false;
    public static Pose targetGoal;

    public String trackState = "IDLE";
    public double turretOutput = 0.0;

    // Internal State
    private final CachingServo gate;
    private double overrideAngleRad = 0.0;
    private double commandedServoPos = TURRET_SERVO_MIDPOINT;
    private boolean override = false;
    private double overrideAngle = 0;

    // PID Controllers
    private final PIDFController pidfController =
            new PIDFController(shootKp, shootKi, shootKd, shootKf);

    private final Pose blueObeliskPose = new Pose(12, 134);
    private final Pose redObeliskPose = new Pose(133, 134);

    public Shooter(HardwareMap hwMap) {
        lobServo = new CachingServo(hwMap.get(Servo.class, "lobServo"));

        turretMotorRight = new CachingDcMotorEx(hwMap.get(DcMotorEx.class, "turretMotorRight"));
        turretMotorLeft = new CachingDcMotorEx(hwMap.get(DcMotorEx.class, "turretMotorLeft"));
        turretMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        turretMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretPivot = new CachingServo(hwMap.get(Servo.class, "turretPivot"));
        gate = new CachingServo(hwMap.get(Servo.class, "gateServo"));
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
        if (distance < 80) {
            return Math.abs(actual - targetVelocity) < 160;
        } else {
            return Math.abs(actual - targetVelocity) < 41;
        }
    }

    public boolean isAimed() {
        if (distance < 60) {
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
//        double lob = (lobA * distance * distance) + (lobB * distance) + lobC;
        double lob = -0.0000526853 * distance * distance * distance + 0.0101482 * distance * distance -0.646392 * distance + 13.95284;
//        if (lob <= uV.lobMin) return uV.lobMin;
//        return Math.min(lob, uV.lobMax);
        return Math.max(Math.min(lob, 1), 0.2);
    }

    private double lobToAngle(double lobPos) {
        return 90-(-35 * lobPos + 51.75);
    }

    private double computeVelocity(double distance) {
//        double velocity = (velA * distance * distance) + (velB * distance) + velC;
        double velocity = 0.0167354 * distance * distance * distance -2.95692 * distance * distance + 176.80479 * distance - 2242.28866;
        if (distance > 120) {
            return 1880;
        }
        return Math.max(Math.min(velocity, 2300), 500);
    }

    private double computeVirtualGoal(double rx, double ry, double vx, double vy) {
        double goalX = targetGoal.getX();
        double goalY = targetGoal.getY();

        double dx = goalX - rx;
        double dy = goalY - ry;
        distance = Math.hypot(dx, dy);

        targetVelocity = computeVelocity(distance);
        pidfController.setSetpoint(targetVelocity);

        double vel = targetVelocity * TICKS_TO_INCHES * Math.cos(Math.toRadians(lobToAngle(targetLob)));

        double tof = distance / vel;
        double virtualX = 0;
        double virtualY = 0;
        for (int i = 0; i < 2; i++) {
            virtualX = goalX - vx * tof;
            virtualY = goalY - vy * tof;
            double virtualDist = Math.hypot(virtualX - rx, virtualY - ry);
            tof = virtualDist / vel;
        }

        double virtualDist = Math.hypot(virtualX - rx, virtualY - ry);
        targetLob = computeLob(virtualDist);


        double v = Math.abs(vx) + Math.abs(vy);

        FtcDashboard.getInstance().getTelemetry().addData("vx", vx);
        FtcDashboard.getInstance().getTelemetry().addData("vy", vy);
        FtcDashboard.getInstance().getTelemetry().addData("v", v);

//        return Math.atan2(virtualY - ry, virtualX - rx) * ((Math.abs(vx) + Math.abs(vy) > 4 ) ? 1.1 : 1);
//        return Math.atan2(virtualY - ry, virtualX - rx) * ((Math.abs(vx) + Math.abs(vy) > 4 ) ? 1.1*Math.signum(vx)*(vx-vy>0 ? -1 : 1) : 1);

//        return Math.atan2(virtualY - ry, virtualX - rx) * ((Math.abs(vx) + Math.abs(vy) > 40 ) ? (vx>0 && vy>0 ? 1.1 : 0.9) : 1);

        return Math.atan2(virtualY - ry, virtualX - rx) * ((v > 1 ) ? ((vx>0 && vy>0) ? 1+(v/((double) 400 /35 * distance)) : 1-(v/((double) 400 /35*distance))) : 1);
    }

    private void mySOTM() {
        Pose robotPose = Robot.follower.getPose();
        Pose target = (Robot.alliance == Robot.Alliance.RED) ? redObeliskPose : blueObeliskPose;

        // 2. Calculate the delta components
        double deltaX = target.getX() - robotPose.getX();
        double deltaY = target.getY() - robotPose.getY();

        // 3. Instantiate the vector
        // Option A: If your Vector class takes (X, Y) components
        Vector robotToGoalVector = new Vector(deltaX, deltaY);

        // constants
        double g = 32.174 * 12;
        double x = robotToGoalVector.getMagnitude() - uV.passThroughPointRadius;
        double y = uV.scoreHeight;
        double a = uV.scoreAngle;

        // calculate initial launch components
        double hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math.tan(a)), 0.2, 0.7);

        double flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));

        // get robot velocity and convert it into parallel and perpendicular components
        Vector robotVelocity = Robot.follower.getVelocity();

        double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.getTheta();

        double parallelComponent = -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

        // velocity compensation variables
        double vz = flywheelSpeed * Math.sin(hoodAngle);
        double time = x / (flywheelSpeed * Math.cos(hoodAngle));
        double ivr = x / time + parallelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr = nvr * time;

        // recalculate launch components
        hoodAngle = MathFunctions.clamp(Math.atan(vz / nvr), uV.lobMin, uV.lobMax);

        flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - y)));

        // update turret
        double turretVelCompOffset = Math.atan(perpendicularComponent / ivr);
        double turretAngle = Robot.follower.getHeading() - robotToGoalVector.getTheta() + turretVelCompOffset;

        if (turretAngle > 180) {
            turretAngle -= 360;
        }

        trackState = "ODOMETRY";
        turretPivot.setPosition(servoPositionFromTurretAngle(turretAngle));

        lobServo.setPosition(hoodAngle);

        // Return the final vector calculations back to the robot controller loop

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
//        double turretDeg = Math.toDegrees(turretAngleRad);
//        turretDeg = Math.max(-TURRET_MAX_ANGLE_DEG, Math.min(TURRET_MAX_ANGLE_DEG, turretDeg));
        return TURRET_SERVO_MIDPOINT + (turretAngleRad * TURRET_GEAR_RATIO / Math.PI) * 0.5;
    }

    private double turretAngleFromServoPosition(double servoPos) {
        double servoDeg = (servoPos - TURRET_SERVO_MIDPOINT) * TURRET_MAX_ANGLE_DEG / 0.5;
        return Math.toRadians(servoDeg * TURRET_GEAR_RATIO);
    }


    // Main Tracking
    public void track() {
        Pose pose = Robot.follower.getPose();
        double heading = pose.getHeading();

        Vector followerVelocity = Robot.follower.getVelocity();

        double targetAngleRad = computeVirtualGoal(pose.getX(), pose.getY(), followerVelocity.getXComponent(), followerVelocity.getYComponent());

        FtcDashboard.getInstance().getTelemetry().addData("turret heading", targetAngleRad);


        double desiredAngleRad = AngleUnit.normalizeRadians(targetAngleRad - heading);

        double servoPos = servoPositionFromTurretAngle(desiredAngleRad);

        servoPos = Math.max(0, Math.min(1, servoPos));

        if (!override) {
            turretPivot.setPosition(servoPos);
            commandedServoPos = servoPos;
        }

        turretServoPos = servoPos;
    }

    // Override Controls
    @Override
    public void reset() {
//        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void init() {
        turretPivot.setPosition(0.5);

        targetSelected = false;
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

    public void lock() {
        goToAngle(overrideAngle);
    }

    // Update Loop
    @Override
    public void update() {
//        pidfController.kP = shootKp;
//        pidfController.kI = shootKi;
//        pidfController.kD = shootKd;
//        pidfController.kF = shootKf;

        if (!targetSelected) {
            targetGoal = Robot.alliance == Robot.Alliance.RED ? redObeliskPose : blueObeliskPose;
            targetSelected = true;
        }

        if (override) {
            double pos = servoPositionFromTurretAngle(overrideAngleRad);
            pos = Math.max(0.0, Math.min(1.0, pos));
            turretPivot.setPosition(pos);
            commandedServoPos = pos;
        }

        if (shooting) {
            track();

            if (shootingLobComp) {
                lobServo.setPosition(targetLob);
            }

            double pidOutput = pidfController.updatePID(-turretMotorLeft.getVelocity());

            turretMotorRight.setPower(-pidOutput);
            turretMotorLeft.setPower(-pidOutput);

        } else if (!override) {
            turretMotorRight.setPower(-0.2);
            turretMotorLeft.setPower(-0.2);
        }
    }
}
