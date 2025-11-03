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
    private DcMotorEx outtakeMotor;

    private Servo servoCamLeft;
    private Servo servoCamRight;

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;

    private final ElapsedTime timer = new ElapsedTime();
    private int step = 0;
    private boolean lift = false;


    private void updateFollower(double power) {
        double y = -gamepad1.left_stick_y*power; // Remember, this is reversed!
        double x = gamepad1.left_stick_x*power; // this is strafing
        double rx = gamepad1.right_stick_x*power;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator;
        double leftRearPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightRearPower = (y + x - rx) / denominator;

        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);
    }

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor1");
        outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        servoCamLeft = hardwareMap.get(Servo.class, "servoCamLeft");
        servoCamRight = hardwareMap.get(Servo.class, "servoCamRight");
        servoCamRight.setDirection(Servo.Direction.REVERSE);

        timer.reset();
        servoCamLeft.setPosition(uV.camLeversDown1);
        servoCamRight.setPosition(uV.camLeversDown2);

    }

    @Override
    public  void start() {
        timer.reset();
        servoCamLeft.setPosition(uV.camLeversDown1);
        servoCamRight.setPosition(uV.camLeversDown2);
    }

    @Override
    public void loop() {
        intakeMotor.setPower(uV.intakePower);
        outtakeMotor.setPower(uV.outtakePower);

       if (timer.seconds() > 2)
       {
           lift = true;
       }



        if (lift) {
            if (step == 0) {
                servoCamLeft.setPosition(uV.camLeversUp1);
                servoCamRight.setPosition(uV.camLeversUp2);
                ++step;
            }

            if (step == 1 && timer.milliseconds() > 5000) {
                servoCamLeft.setPosition(uV.camLeversDown1);
                servoCamRight.setPosition(uV.camLeversDown2);

                step = 0;
                lift = false;
            }
        }

        //updateFollower(1);
    }
}
