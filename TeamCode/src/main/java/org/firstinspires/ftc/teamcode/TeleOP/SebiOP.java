package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.uV;

@Autonomous(name = "Drive Sebastian", group = "0. TeleOp")
public class SebiOP extends OpMode { ;

    private DcMotorEx intakeMotor;

    private DcMotorEx outtakeMotor;

    private Servo uppies;

    private CRServo revolverSpin;


    private DcMotorEx leftFront, leftRear, rightFront, rightRear;

    private final ElapsedTime lifttimer = new ElapsedTime();
    private final ElapsedTime launchtimer = new ElapsedTime();
    private final ElapsedTime loadtimer = new ElapsedTime();
    private int step = 0;
    private boolean lift = false;
    private boolean load = false;
    private boolean launch = false;

    private double clamp(double x, double min, double max) {
        return Math.min(Math.max(x, min), max);
    }


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

        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        uppies = hardwareMap.get(Servo.class, "uppies");

        revolverSpin = hardwareMap.get(CRServo.class, "revolverSpin");



        lifttimer.reset();
        launchtimer.reset();
        loadtimer.reset();

    }

    @Override
    public  void start() {
        lifttimer.reset();
    }

    @Override
    public void loop() {


        if (gamepad1.left_trigger > 0) {
            revolverSpin.setPower(0.3);
        } else if (gamepad1.right_trigger > 0) {
            revolverSpin.setPower(-0.3);
        } else {
            revolverSpin.setPower(0);
        }


        if (gamepad1.dpad_up)
        {
            uppies.setPosition(uV.liftUp);
        } else if (gamepad1.dpad_down) {
            uppies.setPosition(uV.uppiesDown);
        } else {
            uppies.setPosition(uV.uppiesDown);
        }


//        if (gamepad1.circle)
//        {
//            lift = true;
//        }

        if (gamepad1.cross)
        {
            load = true;
        }


        if (gamepad1.square)
        {
            launch = true;
        }


        if (load) {
            intakeMotor.setPower(uV.intakePower);
            if (loadtimer.seconds() > 2)
            {
                load = false;
                loadtimer.reset();
            }
        }

        if (launch) {
            outtakeMotor.setPower(uV.outtakePower);
            if (launchtimer.seconds() > 2)
            {
                launch = false;
                launchtimer.reset();
            }
        }


//        if (lift) {
//            if (step == 0) {
//                uppies.setPosition(uV.liftUp);
//                ++step;
//            }
//
//            if (step == 1 && lifttimer.milliseconds() > 1000) {
//                uppies.setPositio0n(uV.uppiesDown);
//                step = 0;
//                lift = false;
//            }
//        }


        //updateFollower(1);
    }
}
