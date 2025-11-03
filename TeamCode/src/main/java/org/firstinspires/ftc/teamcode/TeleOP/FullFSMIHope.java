package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Revolver;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.uV;

@TeleOp(name = "Drive", group = "0. TeleOp")
public class FullFSMIHope extends OpMode { ;

    private DcMotorEx intakeMotor;
    private DcMotorEx outtakeMotor;

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;

    private State state = State.DRIVE;

    private Robot robot;

    private final ElapsedTime inputTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();


    enum State {
        INTAKE,
        OUTTAKE,
        DRIVE
    };

    // change state and reset timers
    private void changeState(State newState) {
        state = newState;
        inputTimer.reset();
        stateTimer.reset();
    }

    // clamp value between min and max
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

    private void handleIntake() {
        intakeMotor.setPower(uV.intakePower);
        robot.revolver.setMode(Revolver.Mode.INTAKE);

        // power off intake and switch back to drive state
        if (gamepad1.left_bumper) {
            intakeMotor.setPower(0);

            changeState(State.DRIVE);
        }
    }

    private void handleOuttake() {
        outtakeMotor.setPower(uV.outtakePower);
        robot.revolver.setMode(Revolver.Mode.OUTTAKE);

        // shoot ball
        // wait 500 ms to let motor speed up
        if (gamepad1.right_trigger > .5 && stateTimer.milliseconds() > 200) {
            robot.revolver.load();
        } else {
            robot.revolver.retract();
        }

        // go to next slot
        if (gamepad1.dpad_left && inputTimer.milliseconds() > 300) {
            robot.revolver.nextSlot();

            inputTimer.reset();
        }

        // go to previous slot
        if (gamepad1.dpad_right && inputTimer.milliseconds() > 300) {
            robot.revolver.prevSlot();

            inputTimer.reset();
        }

        // go back to drive mode
        if (gamepad1.square) {
            outtakeMotor.setPower(0);

            changeState(State.DRIVE);
        }
    }

    private void handleDrive() {
        // leave motor in semi-active state to reduce ramp up time
        outtakeMotor.setPower(.3 * uV.outtakePower);

        // switch to intake state
        if (gamepad1.cross) {
            changeState(State.INTAKE);
        }

        // switch to outtake state
        if (gamepad1.square) {
            changeState(State.OUTTAKE);
        }
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.revolver.setMode(Revolver.Mode.INTAKE);
    }

    @Override
    public  void start() {
        // start in drive mode
        changeState(State.DRIVE);
    }

    @Override
    public void loop() {

        // FSM :>

        switch (state) {
            case INTAKE:
                handleIntake();
                break;

            case OUTTAKE:
                handleOuttake();
                break;

            case DRIVE:
                handleDrive();
                break;
        }

//        updateFollower(1);

        robot.revolver.update();

        telemetry.addData("current position", robot.revolver.getCurrentPosition());
        telemetry.addData("target position", robot.revolver.getTargetPosition());
        telemetry.addData("state", state);
        telemetry.update();
    }}
