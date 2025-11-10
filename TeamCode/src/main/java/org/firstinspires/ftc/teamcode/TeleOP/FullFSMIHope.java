package org.firstinspires.ftc.teamcode.TeleOP;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Revolver;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.uV;

@TeleOp(name = "Drive", group = "0. TeleOp")
public class FullFSMIHope extends OpMode { ;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private DcMotorEx intakeMotor;
    Thread t1;


//    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    private State state = State.INTAKE;

    private Robot robot;

    private final ElapsedTime inputTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();


    enum State {
        INTAKE,
        OUTTAKE
    };

    // change state and reset timers
    private void changeState(State newState) {
        state = newState;

        inputTimer.reset();
        stateTimer.reset();

        // go to pos 0 always
        robot.revolver.setTargetSlot((byte) 0);
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

        robot.revolver.leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);
    }

    private void handleIntake() {
        robot.turret.turretMotor.setPower(0.5 * uV.outtakePower);

        robot.intake.startMotor();
        robot.revolver.mode = Revolver.Mode.INTAKE;

        // go to previous slot
        if (gamepad1.dpad_left && inputTimer.milliseconds() > 300) {
            robot.revolver.prevSlot();

            inputTimer.reset();
        }

        // go to next slot
        if (gamepad1.dpad_right && inputTimer.milliseconds() > 300) {
            robot.revolver.nextSlot();

            inputTimer.reset();
        }

        // power off intake and switch to outtake state
        if (gamepad1.cross && stateTimer.milliseconds() > 400) {
            robot.intake.stopMotor();
            changeState(State.OUTTAKE);
        }
    }

    private void handleOuttake() {
        robot.turret.startMotor();
        robot.revolver.mode = Revolver.Mode.OUTTAKE;

        // shoot ball
        // wait at least 300 ms to let motor speed up
        if (gamepad1.right_trigger > 0.5 && stateTimer.milliseconds() > 300) {
            robot.revolver.load();
        } else {
            robot.revolver.retract();
        }

        // go to previous slot
        if (gamepad1.dpad_left && inputTimer.milliseconds() > 300) {
            robot.revolver.prevSlot();

            inputTimer.reset();
        }

        // go to next slot
        if (gamepad1.dpad_right && inputTimer.milliseconds() > 300) {
            robot.revolver.nextSlot();
            
            inputTimer.reset();
        }

        // go to intake state
        if (gamepad1.cross && stateTimer.milliseconds() > 400) {
            changeState(State.INTAKE);
        }

    }


    @Override
    public void init() {
        robot = new Robot(hardwareMap);
//        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
//        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public  void start() {
        changeState(State.INTAKE);
        robot.revolver.mode = Revolver.Mode.INTAKE;

        t1 = new Thread(robot.revolver);

        t1.start();

    }

    @Override
    public void loop() {

        switch (state) {
            case INTAKE:
                handleIntake();
                break;

            case OUTTAKE:
                handleOuttake();
                break;

        }


//        updateFollower(1);


        dashboardTelemetry.addData("target position", robot.revolver.getTargetSlot());
        dashboardTelemetry.addData("state", state);
        dashboardTelemetry.addData("target pos: ", Revolver.target);
        dashboardTelemetry.addData("current pos: ", robot.revolver.leftFront.getCurrentPosition());
        dashboardTelemetry.addData("input timer", inputTimer.milliseconds());

        dashboardTelemetry.addData("dpad l", gamepad1.dpad_left);
        dashboardTelemetry.addData("dpad r", gamepad1.dpad_right);

        dashboardTelemetry.addData("distance to walk", robot.revolver.distanceToWalk);

        telemetry.addData("target position", robot.revolver.getTargetSlot());
        telemetry.addData("state", state);
        telemetry.update();
        dashboardTelemetry.update();
    }

    @Override
    public void stop() {
        // stop thread so it resets between restarts
        try {
            t1.interrupt();
        } catch (RuntimeException e) {
            // pass
        }
    }
}
