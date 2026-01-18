package org.firstinspires.ftc.teamcode.Auto.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Utils.PIDFController;

@Config
@Autonomous(name = "Dual Motor Tuner", group = "2. Tuners")
public class DualMotorTuner extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;

    private DcMotorEx spindexer;

    private ElapsedTime timer = new ElapsedTime();

    private final PIDFController pidfController =
            new PIDFController(shootKp, shootKi, shootKd, shootKf);

    public static double shootKp = 0.07;
    public static double shootKi = 0.00002;
    public static double shootKd = 0.0000001;
    public static double shootKf = 0.013;
    public static int targetVelocity = 0;

    @Override
    public void init() {
        rightMotor = hardwareMap.get(DcMotorEx.class, "natasha");
        leftMotor = hardwareMap.get(DcMotorEx.class, "starDestroyer");
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidfController.setTolerance(20);
        pidfController.maxOut = 2;
        pidfController.minOut = -2;
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        pidfController.kP = shootKp;
        pidfController.kI = shootKi;
        pidfController.kD = shootKd;
        pidfController.kF = shootKf;

        pidfController.setSetpoint(targetVelocity);
        double pidOutput = pidfController.updatePID(rightMotor.getVelocity());
        ////
        rightMotor.setPower(pidOutput / 2);
        leftMotor.setPower(pidOutput / 2);

//        rightMotor.setPower(0.5);
//        leftMotor.setPower(0.5);

//        if (timer.seconds() > 6) {
//            spindexer.setPower(0.5);
//        }

        dashboardTelemetry.addData("current", rightMotor.getVelocity());
        dashboardTelemetry.addData("target", targetVelocity);
        dashboardTelemetry.addData("pid", pidOutput);

        dashboardTelemetry.update();
    }
}
