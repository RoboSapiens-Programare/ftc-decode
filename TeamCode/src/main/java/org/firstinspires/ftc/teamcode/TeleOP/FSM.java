package org.firstinspires.ftc.teamcode.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Revolver;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Utils.ColorEnum;
import org.firstinspires.ftc.teamcode.Robot.uV;

@TeleOp(name = "TeleOp")
public class FSM extends OpMode {

    private Robot robot;

    enum State {
        INTAKE,
        OUTTAKE
    };

    enum SortingMode{
        AUTO,
        MANUAL
    };

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    private State state = State.INTAKE;
    private SortingMode sortingMode = SortingMode.AUTO;


    private final ElapsedTime inputTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime loadBallTimer = new ElapsedTime();
    private boolean singletonLoad = true;
    private boolean singletonShoot = false;


    // change state and reset timers
    private void changeState(State newState) {
        state = newState;

        inputTimer.reset();
        stateTimer.reset();
        loadBallTimer.reset();
        // go to pos 0 always
        robot.revolver.setTargetSlot((byte) 0);
    }


    

    private void handleIntake() {
        robot.turret.turretMotor.setPower(0.5 * uV.outtakePower);

        robot.intake.startMotor();
        robot.revolver.mode = Revolver.Mode.INTAKE;

        switch(sortingMode) {
            case AUTO:

                if(robot.revolver.isSlotFull(robot.revolver.getTargetSlot())){
                    if(robot.revolver.getFreeSlot() != -1) {
                        robot.revolver.setTargetSlot(robot.revolver.getFreeSlot());
                    }
                }
                else robot.revolver.setTargetSlot(robot.revolver.getTargetSlot());

                if(gamepad1.dpad_right || gamepad1.dpad_left){
                    sortingMode = SortingMode.MANUAL;
                }
                break;

            case MANUAL:
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

                if(gamepad1.right_bumper) {
                    sortingMode = SortingMode.AUTO;
                }
                break;
        }

        // power off intake and switch to outtake state
        if (gamepad1.cross && stateTimer.milliseconds() > 400) {
            robot.intake.stopMotor();
            changeState(State.OUTTAKE);
        }

        robot.intake.update();
    }

    private void handleOuttake() {
        robot.turret.startMotor();
        robot.revolver.mode = Revolver.Mode.OUTTAKE;
        robot.revolver.update();

        if (sortingMode == SortingMode.AUTO && !singletonShoot) {
            if (robot.revolver.getFullSlot() != -1) {
                robot.revolver.setTargetSlot(robot.revolver.getFullSlot());
                robot.intake.update();

            } else robot.revolver.setTargetSlot((byte) 0);

            if (gamepad1.dpad_right || gamepad1.dpad_left) {
                sortingMode = SortingMode.MANUAL;
            }
        } else if (sortingMode == SortingMode.MANUAL && !singletonShoot) {
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

            if (gamepad1.right_bumper) {
                sortingMode = SortingMode.AUTO;
            }
        }

        // shoot ball
        // wait at least 300 ms to let motor speed up
        if (gamepad1.right_trigger > 0.5 && stateTimer.milliseconds() > 300) {
            singletonShoot = true;
        }

        if (singletonShoot) {
            if(singletonLoad){
                robot.revolver.liftLoad();
                loadBallTimer.reset();
                singletonLoad = false;
            }

            if(loadBallTimer.milliseconds() > 1500 && !singletonLoad) {
                robot.revolver.setSlotColor(robot.revolver.getTargetSlot(), ColorEnum.UNDEFINED);
                robot.revolver.liftReset();

                singletonLoad = true;
                singletonShoot = false;
            }
        }

        robot.turret.setAngle((float) (gamepad2.left_stick_x));

        // go to intake state
        if (gamepad1.cross && stateTimer.milliseconds() > 400) {
            changeState(State.INTAKE);
        }

    }


    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        robot.revolver.liftReset();
    }

    @Override
    public  void start() {
        changeState(State.INTAKE);
        robot.revolver.mode = Revolver.Mode.INTAKE;
        robot.revolver.setTargetSlot((byte) 1);
        sortingMode = SortingMode.AUTO;
        robot.revolver.start();
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


        robot.drive.updateDrive(
            -gamepad1.left_stick_y,
            gamepad1.left_stick_x,
            gamepad1.right_stick_x
        );

        robot.revolver.update();


        dashboardTelemetry.addData("state", state);
        dashboardTelemetry.addData("target position", Revolver.target);
        dashboardTelemetry.addData("current position", robot.revolver.encoderRevolver.getCurrentPosition());
        dashboardTelemetry.addData("ball count", robot.revolver.getBallCount());

        dashboardTelemetry.update();
    }

    @Override
    public void stop() {
        // stop thread so it resets between restarts
        robot.threadKill();
    }
}
