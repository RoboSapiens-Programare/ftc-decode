package org.firstinspires.ftc.teamcode.Robot.Subsystems;

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
import com.qualcomm.robotcore.util.ElapsedTime;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Utils.PIDFController;
import org.firstinspires.ftc.teamcode.Robot.uV;

@Config
public class Shooter extends Subsystem {

    private static final double WHEEL_RADIUS_INCHES = 1.4567; // 74mm diameter / 2
    private static final double RAW_MOTOR_TICKS = 28.0; // 1:1 kit bypasses gearbox completely
    private static final double EXTERNAL_GEAR_RATIO =
            1.0; // Change if you use external gears/pulleys

    private static final double TICKS_TO_INCHES =
            (2.0 * Math.PI * WHEEL_RADIUS_INCHES) / (RAW_MOTOR_TICKS * EXTERNAL_GEAR_RATIO);
    public static double TURRET_SERVO_MIDPOINT = 0.5;
    public static double TURRET_MAX_ANGLE_DEG = 90.0;
    public static double TURRET_GEAR_RATIO = 1.2;
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
    public double desiredAngleRad = 0;

    private final ElapsedTime trackingTimer = new ElapsedTime();

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
//        if (distance < 60) {
//            return Math.abs(Math.toDegrees(turretErrorRad)) < TURRET_AIM_THRESHOLD_DEG;
//        } else {
//            return Math.abs(Math.toDegrees(turretErrorRad)) < TURRET_AIM_THRESHOLD_DEG + 2;
//        }
        return true;
    }

    public void resetShooterPID() {
        pidfController.reset();
    }

    // Trajectory
    private double computeLob(double distance) {
        //        double lob = (lobA * distance * distance) + (lobB * distance) + lobC;
        double lob =
                -0.0000526853 * distance * distance * distance
                        + 0.0101482 * distance * distance
                        - 0.646392 * distance
                        + 13.95284;
        //        if (lob <= uV.lobMin) return uV.lobMin;
        //        return Math.min(lob, uV.lobMax);
        return Math.max(Math.min(lob, 1), 0.2);
    }

    private double lobToAngle(double lobPos) {
        return 90 - (-35 * lobPos + 51.75);
    }

    private double computeVelocity(double distance) {
        // 1. Guard clause for close-range target arrival
        if (distance <= 20.0) {
            return 500.0; // Or whatever your target holding/stop velocity is
        }

        double velocity = -0.0006780758 * Math.pow(distance, 3) + 0.1700239691 * Math.pow(distance, 2) + -5.9676194360 * distance + 1305.8793678170;
        return Math.max(Math.min(velocity, 2300.0), 500.0);

        // 2. Guard clause for long-range max velocity
        //        if (distance > 120.0) {
        //            return 1880.0;
        //        }

        // 3. Main cubic regression curve
        //        double velocity =
        //                0.0165354 * distance * distance * distance
        //                        - 2.95692 * distance * distance
        //                        + 176.80479 * distance
        //                        - 2182.28866;

        // 4. Inject the aggressive acceleration boost for smaller distances
        //        if (distance < 60.0) {
        //            velocity += 85.0 * Math.exp(-0.06 * (distance - 20.0));
        //        }

        // 5. Final safety clamp
        //        return Math.max(Math.min(velocity, 2300.0), 500.0);
    }

    private double computeVirtualGoal(double rx, double ry, double vx, double vy) {
        double goalX = targetGoal.getX();
        double goalY = targetGoal.getY();

        double dx = goalX - rx;
        double dy = goalY - ry;
        distance = Math.hypot(dx, dy);

        targetVelocity = computeVelocity(distance);

        // Calculate actual projectile horizontal velocity component
        double vel =
                targetVelocity * TICKS_TO_INCHES * Math.cos(Math.toRadians(lobToAngle(targetLob)));
        double tof = distance / vel;

        double virtualX = goalX;
        double virtualY = goalY;

        // Core predictive targeting loop (Time of Flight convergence)
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

        // --- SCALING ADJUSTMENT FIX ---
        // If you need to manually over-compensate or tweak the lead because of friction/latency,
        // apply an offset modifier directly to the virtual coordinates before atan2:
        if (v > 1.0) {
            // Example: Dynamically tweak the lead position based on velocity and distance
            double gain = 1.0 + (v / ((400.0 / 35.0) * distance));

            // Push the virtual point further out along the vector of movement
            virtualX = goalX - (vx * tof * gain);
            virtualY = goalY - (vy * tof * gain);
        }

        // Return the pure, untampered heading angle to the compensated target position
        return Math.atan2(virtualY - ry, virtualX - rx);
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

        double flywheelSpeed =
                Math.sqrt(
                        g
                                * x
                                * x
                                / (2
                                        * Math.pow(Math.cos(hoodAngle), 2)
                                        * (x * Math.tan(hoodAngle) - y)));

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

        flywheelSpeed =
                Math.sqrt(
                        g
                                * ndr
                                * ndr
                                / (2
                                        * Math.pow(Math.cos(hoodAngle), 2)
                                        * (ndr * Math.tan(hoodAngle) - y)));

        // update turret
        double turretVelCompOffset = Math.atan(perpendicularComponent / ivr);
        double turretAngle =
                Robot.follower.getHeading() - robotToGoalVector.getTheta() + turretVelCompOffset;

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
        //        turretDeg = Math.max(-TURRET_MAX_ANGLE_DEG, Math.min(TURRET_MAX_ANGLE_DEG,
        // turretDeg));
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

        double targetAngleRad = 0;

        if (shooting) {
            Vector followerVelocity = Robot.follower.getVelocity();
            targetAngleRad =
                    computeVirtualGoal(
                            pose.getX(),
                            pose.getY(),
                            followerVelocity.getXComponent(),
                            followerVelocity.getYComponent());
        } else {
            distance = Math.hypot(targetGoal.getY() - pose.getY(), targetGoal.getX() - pose.getX());
            targetAngleRad =
                    Math.atan2(targetGoal.getY() - pose.getY(), targetGoal.getX() - pose.getX());
            targetLob = computeLob(distance);
            targetVelocity = computeVelocity(distance);
        }

        desiredAngleRad = AngleUnit.normalizeRadians(targetAngleRad - heading);
        desiredAngleRad = Math.max(-Math.PI * 2 / 5, Math.min(desiredAngleRad, Math.PI * 2 / 5));
        double servoPos = servoPositionFromTurretAngle(desiredAngleRad);

        servoPos = Math.max(0, Math.min(1, servoPos));

        turretErrorRad = desiredAngleRad - turretAngleFromServoPosition(commandedServoPos);

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

        if (!shooting && trackingTimer.milliseconds() > 300) {
            trackingTimer.reset();

            track();
        }

        if (shooting) {
            track();

            if (shootingLobComp) {
                lobServo.setPosition(targetLob);
            }

            pidfController.setSetpoint(targetVelocity);
            double pidOutput = pidfController.updatePID(-turretMotorLeft.getVelocity());

            turretMotorRight.setPower(-pidOutput);
            turretMotorLeft.setPower(-pidOutput);
        } else {

            turretMotorRight.setPower(-0.2);
            turretMotorLeft.setPower(-0.2);
        }
    }
}
