package org.firstinspires.ftc.teamcode.Auto.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.Robot.Utils.PIDFController;

@Autonomous(name = "Motor test", group = "1. Auto Tests")
public class MotorTest extends OpMode {
    private DcMotorEx motor;
    private TouchSensor limitSwitch;

    private final PIDFController pidfController = new PIDFController(Kp, Ki, Kd, Kf);

    public static double Kp = -0.0006;
    public static double Ki = -0.00001;
    public static double Kd = -0.000053;
    public static double Kf = 0;
    public static int target = 0;
    public static int tolerance = 30;

    private boolean doOnce = true;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "intake");
        limitSwitch = hardwareMap.get(TouchSensor.class, "spindexerLimitSwitch");

        pidfController.setTolerance(tolerance);
    }

    @Override
    public void loop() {
        //        while (!limitSwitch.isPressed() && doOnce) {
        //            motor.setPower(0.2);
        //        }
        //
        //        if (doOnce) {
        //            doOnce = false;
        //
        //            motor.setPower(0);
        //
        //            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //        } else {
        //            pidfController.kP = Kp;
        //            pidfController.kI = Ki;
        //            pidfController.kD = Kd;
        //            pidfController.kF = Kf;
        //
        //            pidfController.setSetpoint(uV.homingOffset);
        //
        //            motor.setPower(pidfController.updatePID(motor.getCurrentPosition()));
        //        }

        motor.setPower(-1);
    }
}
