package org.firstinspires.ftc.teamcode.Auto.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Utils.PIDFController;
import org.firstinspires.ftc.teamcode.Robot.uV;

@Config
@Autonomous(name = "PID Tuner", group = "2. Tuners")
public class PidTuner extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private DcMotorEx motor;
    private DcMotorEx rightMotor, leftMotor;

    public double Kp = -0.0006;
    public double Ki = -0.00001;
    public double Kd = -0.000053;
    public double Kf = 0;
    public int target = 0;
    public int tolerance = 30;
    private final PIDFController pidfController = new PIDFController(Kp, Ki, Kd, Kf);

    public double shootKp = 0.075;
    public double shootKi = 0.00002;
    public double shootKd = 0.0000001;
    public double shootKf = 0.013;

    private final PIDFController shootController =
            new PIDFController(shootKp, shootKi, shootKd, shootKf);
    public int targetVelocity = 1300;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "spindexer");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        rightMotor = hardwareMap.get(DcMotorEx.class, "natasha");
        leftMotor = hardwareMap.get(DcMotorEx.class, "starDestroyer");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //        motor.setMode(DcMotor.RunMode.);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shootController.setTolerance(20);
        shootController.maxOut = 2;
        shootController.minOut = -2;
    }

    @Override
    public void loop() {
        pidfController.kP = Kp;
        pidfController.kI = Ki;
        pidfController.kD = Kd;
        pidfController.kF = Kf;

        shootController.kP = shootKp;
        shootController.kI = shootKi;
        shootController.kD = shootKd;
        shootController.kF = shootKf;

        pidfController.setSetpoint(target);
        pidfController.setTolerance(tolerance);

        double pidOutput = pidfController.updatePID(motor.getCurrentPosition());

        motor.setPower(pidOutput * uV.revolverPowerMultiplier);

        shootController.setSetpoint(targetVelocity);
        double shoot = shootController.updatePID(rightMotor.getVelocity());

        rightMotor.setPower(shoot / 2);
        leftMotor.setPower(shoot / 2);

        if (shootController.targetReached()) {
            target -= 8192 / 3;
        }

        dashboardTelemetry.addData("current", motor.getCurrentPosition());
        dashboardTelemetry.addData("target", target);
        dashboardTelemetry.addData("pid", pidOutput);
        dashboardTelemetry.addData("velo", rightMotor.getVelocity());
        dashboardTelemetry.addData("shoot", shoot);

        dashboardTelemetry.update();
    }
}
