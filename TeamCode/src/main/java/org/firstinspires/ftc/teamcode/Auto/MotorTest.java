package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.uV;

@Autonomous(name = "Motor test", group = "1. Auto Tests")
public class MotorTest extends OpMode { ;

    private DcMotorEx intakeMotor;

    private final ElapsedTime timer = new ElapsedTime();
    private int step = 0;


    private void updateFollower(double power, double gx, double gy, double gr) {
        double y = -gy*power; // Remember, this is reversed!
        double x = gx*power; // this is strafing
        double rx = gr*power;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
    }

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        timer.reset();

    }

    @Override
    public  void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        intakeMotor.setPower(uV.intakePower);
    }
}
