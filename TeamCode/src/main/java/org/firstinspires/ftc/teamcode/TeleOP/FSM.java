package org.firstinspires.ftc.teamcode.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Revolver;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Robot.Utils.ColorEnum;

@Config
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
    private int shootStep = -1;

    private int motifPosition = 0;
    private boolean sortMotif = false;


    // change state and reset timers
    private void changeState(State newState) {
        state = newState;

        inputTimer.reset();
        stateTimer.reset();
        loadBallTimer.reset();
        // go to pos 0 always
        robot.revolver.setTargetSlot((byte) 0);
    }


    

    public void handleIntake() {
//        robot.turret.turretMotor.setPower(0.5 * uV.outtakePower);

        if(gamepad1.right_trigger > 0.5) {
            robot.intake.startMotor();
        }
        else robot.intake.stopMotor();

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
//            robot.turret.turretMotor.setPower(.5);
            changeState(State.OUTTAKE);
        }

        robot.turret.turretMotor.setPower(0);
        robot.intake.update();
    }

    public void handleOuttake() {
        robot.revolver.mode = Revolver.Mode.OUTTAKE;

        if (gamepad1.cross && stateTimer.milliseconds() > 400) {
            robot.intake.stopMotor();
            changeState(State.INTAKE);
        }

        if (sortingMode == SortingMode.AUTO) {

            if (sortMotif) {
                byte t = robot.revolver.getSlotByMotifPosition(motifPosition);
                FtcDashboard.getInstance().getTelemetry().addData("t", (int) t);
                if (t != -1) {
                    robot.revolver.setTargetSlot(t);
                } else robot.revolver.setTargetSlot((byte) 0);
            } else {
                if (robot.revolver.getFullSlot() != -1) {
                    robot.revolver.setTargetSlot(robot.revolver.getFullSlot());
                } else {
                    changeState(State.INTAKE);
                }
            }



            if (gamepad1.dpad_right || gamepad1.dpad_left) {
                sortingMode = SortingMode.MANUAL;
            }
        } else if (sortingMode == SortingMode.MANUAL) {
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

            if (gamepad1.cross) {
                sortingMode = SortingMode.AUTO;
            }
        }

        // shoot ball
        // wait at least 300 ms to let motor speed up
        if (gamepad1.right_trigger > 0.5 && stateTimer.milliseconds() > 300 && shootStep == -1) {
            shootStep = 0;
            loadBallTimer.reset();
        }

        if (shootStep >= 0) {
            if (loadBallTimer.milliseconds() > 1400 && shootStep == 0) {
                robot.revolver.liftLoad();

                ++shootStep;
            }

            if(loadBallTimer.milliseconds() > 1800 && shootStep == 1) {
                robot.revolver.setSlotColor(robot.revolver.getTargetSlot(), ColorEnum.UNDEFINED);
                robot.revolver.liftReset();



                ++shootStep;

            }

            if (loadBallTimer.milliseconds() > 2200 && shootStep == 2) {
                    if (gamepad1.left_trigger > 0.5) {
                        shootStep = 0;
                    } else {
                        shootStep = -1;
                    }
                    if (motifPosition == 2) {
                        motifPosition = 0;
                    } else {
                        ++motifPosition;
                    }
                }
            }



        // turret tracking
        // TODO: change to driver 2

//        if (!robot.turret.tracking) {
//            robot.turret.turretRotationServo.setPower(gamepad1.left_stick_x);
//        }


        // TODO: change to driver 2

        if (gamepad1.left_bumper && inputTimer.milliseconds() > 400) {
            robot.revolver.prevMotif();
            inputTimer.reset();

        } else if (gamepad1.left_bumper && inputTimer.milliseconds() > 400) {
            robot.revolver.nextMotif();
            inputTimer.reset();
        }

        FtcDashboard.getInstance().getTelemetry().addData("motifpos",motifPosition);
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
        robot.turret.enableCamera();

        // TODO: remove
        robot.turret.targetObelisk = Turret.TargetObelisk.BLUE;


        robot.revolver.setTargetSlot((byte) 0);
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
            gamepad1.left_stick_y,
            -gamepad1.left_stick_x,
            -gamepad1.right_stick_x
        );

        robot.revolver.update();
        robot.turret.update();

        // TODO: change to driver 2
//        robot.turret.turretRotationServo.setPower((float) (gamepad1.left_stick_x));

        if (inputTimer.milliseconds() > 500 && gamepad1.touchpad) {
            sortMotif = !sortMotif;
            inputTimer.reset();
        }


        dashboardTelemetry.addData("state", state);
        dashboardTelemetry.addData("target position", Revolver.target);
        dashboardTelemetry.addData("current position", robot.revolver.encoderRevolver.getCurrentPosition());

        dashboardTelemetry.addData("slot 0", robot.revolver.getSlotColor((byte) 0));
        dashboardTelemetry.addData("slot 1", robot.revolver.getSlotColor((byte) 1));
        dashboardTelemetry.addData("slot 2", robot.revolver.getSlotColor((byte) 2));
//        dashboardTelemetry.addData("color", robot.intake.);

        dashboardTelemetry.update();

        telemetry.addData("revolver target slot", robot.revolver.getTargetSlot());
    }

    @Override
    public void stop() {
    }
}
