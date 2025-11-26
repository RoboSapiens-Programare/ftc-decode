package org.firstinspires.ftc.teamcode.TeleOP;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Revolver;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Robot.Utils.ColorEnum;

// bag pula coaie il fac maine pe asta fmm de fsm

@Config
@TeleOp(name = "TeleOp")
public class FSM extends OpMode {
    private Robot robot;

    enum State {
        INTAKE,
        OUTTAKE
    };

    enum SortingMode {
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
    private boolean homeSpindexer = false;
    private boolean sortMotif = false;

    // Helper method to kill joystick drift
    private float applyDeadzone(float input) {
        // 0.1 is the threshold (10%). Adjust if your controller is older/looser.
        if (abs(input) < 0.05f) {
            return 0.0f;
        }
        return input;
    }

    // change state and reset timers
    private void changeState(State newState) {
        state = newState;

        inputTimer.reset();
        stateTimer.reset();
        loadBallTimer.reset();

        robot.intake.setPower(0);
    }

    private void manualSort() {

        // go to next slot
        if (gamepad2.dpad_right && inputTimer.milliseconds() > 300) {
            robot.spindexer.goToSlot(robot.spindexer.getTargetSlot() + 1);

            inputTimer.reset();
        }

        if (gamepad2.right_bumper) {
            sortingMode = SortingMode.AUTO;
        }
    }

    public void handleIntake() {
        robot.turret.tracking = false;
        robot.turret.turretMotor.setPower(0);
        robot.intake.update();

        if (gamepad1.right_trigger > 0.5) {
            robot.intake.setPower(1);
        } else if (gamepad1.left_trigger > 0.5) {
            robot.intake.setPower((float) -0.55);
        } else robot.intake.setPower(0);

        // sort
        switch (sortingMode) {
            case AUTO:
                if (robot.spindexer.isSlotFull(robot.spindexer.getTargetSlot())) {
                    if (robot.spindexer.getFreeSlot() != -1) {
                        robot.spindexer.setTargetSlot(robot.spindexer.getFreeSlot());
                    }
                } else robot.spindexer.setTargetSlot(robot.spindexer.getTargetSlot());

                if (gamepad2.dpad_right || gamepad2.dpad_left) {
                    sortingMode = SortingMode.MANUAL;
                }
                break;

            case MANUAL:
                manualSort();
                break;
        }

        // power off intake and switch to outtake state
        if (gamepad1.cross && stateTimer.milliseconds() > 400) {
            changeState(State.OUTTAKE);
        }

        // change color based on current slot
        if (robot.spindexer.getSlotColor(robot.spindexer.getTargetSlot()) == ColorEnum.GREEN) {
            gamepad2.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (robot.spindexer.getSlotColor(robot.spindexer.getTargetSlot())
                == ColorEnum.PURPLE) {
            gamepad2.setLedColor(155, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
        } else gamepad2.setLedColor(255, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);

        // homing
        if (gamepad2.left_bumper && !homeSpindexer && inputTimer.milliseconds() > 500) {
            homeSpindexer = true;
            inputTimer.reset();
        }

        if (homeSpindexer) {
            robot.spindexer.motor.setPower(applyDeadzone(gamepad2.right_stick_x) * 0.125);
        }

        if (gamepad2.left_bumper && homeSpindexer && inputTimer.milliseconds() > 500) {
            robot.spindexer.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            homeSpindexer = false;
            inputTimer.reset();
        }
    }

    public void handleOuttake() {
        robot.spindexer.mode = Spindexer.Mode.OUTTAKE;

        if (!robot.turret.tracking) {
            robot.turret.turretMotor.setPower(1);
        }

        if (gamepad1.cross && stateTimer.milliseconds() > 400) {
            robot.intake.setPower(0);
            changeState(State.INTAKE);
        }

        switch (sortingMode) {
            case AUTO:
                // TODO: implement motif sorting and test it
                if (sortMotif) {
                    int t = robot.spindexer.getMotifOffset();
                    FtcDashboard.getInstance().getTelemetry().addData("t", t);
                    if (t != -1) {
                        robot.spindexer.goToSlot(t);
                    } else robot.spindexer.goToSlot(0);
                } else {
                    if (robot.spindexer.getFullSlot() != -1) {
                        robot.spindexer.goToSlot(robot.spindexer.getFullSlot());
                    }
//                    TODO: change to default to intake state when spindexer works consistently
//                    else if (loadBallTimer.milliseconds() > 300) {
//                        changeState(State.INTAKE);
//                    }
                }

                if (gamepad2.dpad_right || gamepad2.dpad_left) {
                    sortingMode = SortingMode.MANUAL;
                }

                break;

            case MANUAL:
                manualSort();
                break;
        }

        // change controller color based on current slot color
        if (robot.spindexer.getSlotColor(robot.spindexer.getTargetSlot()) == ColorEnum.GREEN) {
            gamepad2.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (robot.spindexer.getSlotColor(robot.spindexer.getTargetSlot())
                == ColorEnum.PURPLE) {
            gamepad2.setLedColor(155, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
        } else gamepad2.setLedColor(255, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);

        // change controller color when turret aligned
        if (robot.turret.isShootReady()) {
            gamepad1.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);

        // shoot ball
        // wait at least 300 ms to let motor speed up
        if (gamepad1.right_trigger > 0.5 && stateTimer.milliseconds() > 300 && shootStep == -1) {
            shootStep = 0;
            loadBallTimer.reset();
        }

        // Shoot sequence
        // Load only if on shoot mode
        if (shootStep >= 0) {
            if (shootStep == 0 && robot.spindexer.isReady()) {
                loadBallTimer.reset();
                ++shootStep;
            }

            if (loadBallTimer.milliseconds() > 100 && shootStep == 1) {
                robot.spindexer.shootCurrentSlot();

                loadBallTimer.reset();
                ++shootStep;
            }

            if (loadBallTimer.milliseconds() > 400 && shootStep == 2) {
                robot.spindexer.setSlotColor(robot.spindexer.getTargetSlot(), ColorEnum.UNDEFINED);

                loadBallTimer.reset();
                ++shootStep;
            }

            if (loadBallTimer.milliseconds() > 800 && shootStep == 3) {
                // if button still pressed, continue shooting sequence
                if (gamepad1.left_trigger > 0.5) {

                    loadBallTimer.reset();
                    shootStep = 0;

                } else {
                    robot.turret.turretMotor.setPower(0);

                    loadBallTimer.reset();
                    shootStep = -1;
                }

                // go to next slot in motif
                if (motifPosition == 2) {
                    motifPosition = 0;
                } else {
                    ++motifPosition;
                }
            }
        }

        // go to motif
        if (gamepad2.triangle && inputTimer.milliseconds() > 400) {
            robot.spindexer.gotoMotif();
            inputTimer.reset();
        }
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.turret.tracking = false;
        robot.turret.enableCamera();
        inputTimer.reset();

        gamepad2.setLedColor(255, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);

        robot.drive.driverGamepad = gamepad1;
    }

    @Override
    public void init_loop() {
        if (gamepad1.options && inputTimer.milliseconds() > 400) {
            if (robot.turret.targetObelisk == Turret.TargetObelisk.RED) {
                robot.turret.targetObelisk = Turret.TargetObelisk.BLUE;
                gamepad1.setLedColor(0, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
            } else {
                robot.turret.targetObelisk = Turret.TargetObelisk.RED;
                gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            }
            inputTimer.reset();
        }
    }

    @Override
    public void start() {
        changeState(State.INTAKE);
        sortingMode = SortingMode.AUTO;

        gamepad1.setLedColor(255, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
        gamepad2.setLedColor(255, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);

        robot.spindexer.mode = Spindexer.Mode.INTAKE;

        robot.spindexer.setTargetSlot((byte) 0);
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

        robot.drive.update();

        if (!homeSpindexer) robot.spindexer.update();

        robot.turret.update();
        robot.turret.turretRotationServo.setPower((float) (-gamepad2.left_stick_x));

        if (gamepad2.touchpad) {
            robot.turret.tracking = true;
        } else robot.turret.tracking = false;

        dashboardTelemetry.addData("state", state);
        dashboardTelemetry.addData("target Slot", robot.spindexer.getTargetSlot());
        dashboardTelemetry.addData(
                "current position", robot.spindexer.motor.getCurrentPosition());

        dashboardTelemetry.addData("power rotation: ", robot.spindexer.motor.getPower());
        dashboardTelemetry.addData("tracking state: ", robot.turret.tracking);
        dashboardTelemetry.addData("shoot step: ", shootStep);

        dashboardTelemetry.addData("slot 0", robot.spindexer.getSlotColor((byte) 0));
        dashboardTelemetry.addData("slot 1", robot.spindexer.getSlotColor((byte) 1));
        dashboardTelemetry.addData("slot 2", robot.spindexer.getSlotColor((byte) 2));

        dashboardTelemetry.addData("motifpos", motifPosition);
        //        dashboardTelemetry.addData("color", robot.intake.);

        dashboardTelemetry.update();
    }

    @Override
    public void stop() {}
}
