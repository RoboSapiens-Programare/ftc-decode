package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.uV;

@Autonomous(name = "Full test", group = "1. Auto Tests")
public class IclesTest extends OpMode { ;

    private DcMotorEx intakeMotor;
    private DcMotorEx outtakeMotor1;
    private DcMotorEx outtakeMotor2;

    private Servo servoCamLeft;
    private Servo servoCamRight;

    private CRServo servoGeckoLeft;
    private CRServo servoGeckoRight;

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

        outtakeMotor1 = hardwareMap.get(DcMotorEx.class, "outtakeMotor1");
        outtakeMotor1.setDirection(DcMotorSimple.Direction.FORWARD);

        outtakeMotor2 = hardwareMap.get(DcMotorEx.class, "outtakeMotor2");
        outtakeMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        servoCamLeft = hardwareMap.get(Servo.class, "servoCamLeft");
        servoCamRight = hardwareMap.get(Servo.class, "servoCamRight");
        servoCamRight.setDirection(Servo.Direction.REVERSE);

        servoGeckoLeft = hardwareMap.get(CRServo.class, "servoGeckoLeft");
        servoGeckoRight = hardwareMap.get(CRServo.class, "servoGeckoRight");
        servoGeckoRight.setDirection(CRServo.Direction.REVERSE);

        timer.reset();
        servoCamLeft.setPosition(.3);
        servoCamRight.setPosition(.3);

    }

    @Override
    public  void start() {
        timer.reset();
        servoCamLeft.setPosition(uV.camLeversDown);
        servoCamRight.setPosition(uV.camLeversDown);
    }

    @Override
    public void loop() {
        intakeMotor.setPower(uV.intakePower);
        outtakeMotor1.setPower(uV.outtakePower);
        outtakeMotor2.setPower(uV.outtakePower);

        if (step == 0) {
            servoGeckoLeft.setPower(uV.geckoLiftPower);
            servoGeckoRight.setPower(uV.geckoLiftPower);

            ++step;
        }

        if (timer.milliseconds() > 3000 && step == 1) {
            servoCamLeft.setPosition(uV.camLeversUp);
            servoCamRight.setPosition(uV.camLeversUp);

            ++step;
        }

        if (timer.milliseconds() > 4500 && step == 2) {
            servoGeckoLeft.setPower(uV.geckoLiftPower);
            servoGeckoRight.setPower(uV.geckoLiftPower);

            ++step;
        }

        if (timer.milliseconds() > 5500 && step == 3) {
            servoCamLeft.setPosition(uV.camLeversDown);
            servoCamRight.setPosition(uV.camLeversDown);

            step = 0;
            timer.reset();
        }
    }
}
