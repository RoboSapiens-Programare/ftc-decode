package org.firstinspires.ftc.teamcode.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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



    private DcMotorEx leftFront, leftRear, rightFront, rightRear;

    private State STATE = State.INTAKE;

    private Robot robot;

    private final ElapsedTime inputTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private boolean stateSingleton = true;


    enum State {
        INTAKE,
        OUTTAKE
    };

    // change state and reset timers
    private void changeState(State newState) {
        STATE = newState;
        inputTimer.reset();
        stateTimer.reset();

        stateSingleton = true;
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
        robot.turret.turretMotor.setPower(0.5 * uV.outtakePower);
        intakeMotor.setPower(uV.intakePower);
        robot.revolver.mode = Revolver.Mode.INTAKE;



        // power off intake and switch back to outtake state
        if (gamepad1.cross && stateTimer.milliseconds() > 200) {
            intakeMotor.setPower(0);
            changeState(State.OUTTAKE);
        }
    }

    private void handleOuttake() {
        robot.turret.startMotor();

        // set revolver in outtake mode
        robot.revolver.mode = Revolver.Mode.OUTTAKE;

        // shoot ball
        // wait 500 ms to let motor speed up
        if (gamepad1.right_trigger > 0.5 && stateTimer.milliseconds() > 200) {
            robot.revolver.load();
        } else {
            robot.revolver.retract();
        }

        // go to next slot
        if (gamepad1.dpad_left && inputTimer.milliseconds() > 300) {
            robot.revolver.prevSlot();
            inputTimer.reset();
        }

        // go to previous slot
        if (gamepad1.dpad_right && inputTimer.milliseconds() > 300) {
            robot.revolver.nextSlot();
            inputTimer.reset();
        }

        // go back to drive mode
        if (gamepad1.cross && stateTimer.milliseconds() > 200) {
            changeState(State.INTAKE);
        }
    }


    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);



        robot.revolver.mode = Revolver.Mode.INTAKE;
    }

    @Override
    public  void start() {
        // start in drive mode
        robot.revolver.setTargetSlot((byte) 0);
        robot.revolver.mode = Revolver.Mode.INTAKE;
        changeState(State.INTAKE);
    }

    @Override
    public void loop() {

        switch (STATE) {
            case INTAKE:
                handleIntake();
                break;

            case OUTTAKE:
                handleOuttake();
                break;
        }


//        updateFollower(1);


        dashboardTelemetry.addData("target position", robot.revolver.getTargetSlot());
        dashboardTelemetry.addData("state", STATE);

        dashboardTelemetry.addData( "encoder", robot.revolver.encoder.getVoltage());
        telemetry.addData( "encoder", robot.revolver.encoder.getVoltage());

        telemetry.addData("target position", robot.revolver.getTargetSlot());
        telemetry.addData("state", STATE);

        robot.revolver.telemetryDump();

        telemetry.update();
        dashboardTelemetry.update();
    }}
