package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
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

/*  THE GREAT TODO
*       - tune pid
*           - procedure:
*               - first determine the max rpm for each motor using the motor test class ( must be modified )
*               - then, by using ziegler-nichols (check google drive for `SFANTA ENCICLOPEDIE`) tune
*
* */

@SuppressWarnings("FieldCanBeLocal")
@Config
public class FixedTurret extends Subsystem {
    public DcMotorEx turretMotorLeft;
    public DcMotorEx turretMotorRight;

    // PID values for turret
    // WHEN TUNING USE ZIEGLER-NICHOLS METHOD
    // IT WAS MADE FOR THIS
    // LITERALLY FOR THIS

    public static double shootKp = 0;
    public static double shootKi = 0;
    public static double shootKd = 0;
    public static double shootKf = 0;

    private final PIDFController pidfController = new PIDFController(shootKp, shootKi, shootKd, shootKf);

    public static double leftMotorMaxRPM = 0;
    public static double rightMotorMaxRPM = 0;

    private final Pose blueObeliskPose = new Pose();
    private final Pose redObeliskPose = new Pose();

    public static double velocityTolerance = 75;
    public double curr = 0;

    public static double targetVelocity = 0;

    public boolean track = false;
    private boolean shouldFollowTrack = true;

    public FixedTurret(HardwareMap hwMap) {
        turretMotorLeft = hwMap.get(DcMotorEx.class, "turretMotorLeft");
        turretMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turretMotorRight = hwMap.get(DcMotorEx.class, "turretMotorRight");
        turretMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean isShootReady() {
        return Math.abs(turretMotorLeft.getVelocity() - targetVelocity) < velocityTolerance;
    }

    public double computeDistance() {
        Pose currentPose = Robot.follower.getPose();
        Pose targetObeliskPose = Robot.alliance == Robot.Alliance.RED ? redObeliskPose : blueObeliskPose;

        return currentPose.distanceFrom(targetObeliskPose);
    }

    private double computeVelocity() {
        return computeDistance() * 37 / 7 + 785.67;
    }

    public double getAngle() {
        Pose currentPose = Robot.follower.getPose();
        Pose targetObeliskPose = Robot.alliance == Robot.Alliance.RED ? redObeliskPose : blueObeliskPose;

        double dx = Math.abs(currentPose.getX() - targetObeliskPose.getX());
        double dy = Math.abs(currentPose.getY() - targetObeliskPose.getY());

        double alpha = Math.atan(dy / dx);

        return Robot.alliance == Robot.Alliance.RED ? alpha : Math.PI - alpha;
    }

    @Override
    public void update() {
        pidfController.kP = shootKp;
        pidfController.kI = shootKi;
        pidfController.kD = shootKd;
        pidfController.kF = shootKf;

        if (track && shouldFollowTrack) {
            Path p = new Path(
                    new BezierLine(
                        Robot.follower.getPose(),
                        Robot.follower.getPose()
                    )
            );

            p.setHeadingInterpolation( HeadingInterpolator.linear(
                    Robot.follower.getHeading(),
                    this.getAngle()
            ) );

            Robot.follower.followPath(p, true);

            shouldFollowTrack = false;
        }

        double pidOutput = pidfController.updatePID(computeVelocity());
        turretMotorLeft.setPower(pidOutput * (leftMotorMaxRPM / (leftMotorMaxRPM + rightMotorMaxRPM)));
        turretMotorRight.setPower(pidOutput * (rightMotorMaxRPM / (leftMotorMaxRPM + rightMotorMaxRPM)));
    }

    public void reset() {
        shouldFollowTrack = true;
    }
}