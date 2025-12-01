package org.firstinspires.ftc.teamcode.TeleOP;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Revolver;
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
    private boolean homeRevolver = false;
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
        // go to previous slot
        if (gamepad2.dpad_left && inputTimer.milliseconds() > 300) {
            robot.revolver.prevSlot();

            inputTimer.reset();
        }

        // go to next slot
        if (gamepad2.dpad_right && inputTimer.milliseconds() > 300) {
            robot.revolver.nextSlot();

            inputTimer.reset();
        }

        if (gamepad2.right_bumper) {
            sortingMode = SortingMode.AUTO;
        }
    }

    public void handleIntake() {
        // always track when testing
        // TODO: re-enable
        //        robot.turret.tracking = false;
        robot.revolver.mode = Revolver.Mode.INTAKE;
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
                if (robot.revolver.isSlotFull(robot.revolver.getTargetSlot())) {
                    if (robot.revolver.getFreeSlot() != -1) {
                        robot.revolver.setTargetSlot(robot.revolver.getFreeSlot());
                    }
                } else robot.revolver.setTargetSlot(robot.revolver.getTargetSlot());

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
        if (robot.revolver.getSlotColor(robot.revolver.getTargetSlot()) == ColorEnum.GREEN) {
            gamepad2.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (robot.revolver.getSlotColor(robot.revolver.getTargetSlot())
                == ColorEnum.PURPLE) {
            gamepad2.setLedColor(155, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
        } else gamepad2.setLedColor(255, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);

        // home revolver in case of ... fuck up
        if (gamepad2.left_bumper && !homeRevolver && inputTimer.milliseconds() > 500) {
            homeRevolver = true;
            inputTimer.reset();
        }

        if (homeRevolver) {
            robot.revolver.revolverSpin.setPower(applyDeadzone(gamepad2.right_stick_x) * 0.125);
        }

        if (gamepad2.left_bumper && homeRevolver && inputTimer.milliseconds() > 500) {
//            robot.revolver.encoderRevolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            homeRevolver = false;
            inputTimer.reset();
        }
    }

    public void handleOuttake() {
        robot.revolver.mode = Revolver.Mode.OUTTAKE;

        if (!robot.turret.tracking) {
            // TODO : change to average spoolup value
            robot.turret.turretMotor.setPower(0);
        }

        if (gamepad1.cross && stateTimer.milliseconds() > 400) {
            robot.intake.setPower(0);
            changeState(State.INTAKE);
        }

        switch (sortingMode) {
            case AUTO:
                if (sortMotif) {
                    byte t = robot.revolver.getSlotByMotifPosition(motifPosition);
                    FtcDashboard.getInstance().getTelemetry().addData("t", (int) t);
                    if (t != -1) {
                        robot.revolver.setTargetSlot(t);
                    } else robot.revolver.setTargetSlot((byte) 0);
                } else {
                    if (robot.revolver.getFullSlot() != -1) {
                        robot.revolver.setTargetSlot(robot.revolver.getFullSlot());
                    } else if (loadBallTimer.milliseconds() > 300) {
                        changeState(State.INTAKE);
                    }
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
        if (robot.revolver.getSlotColor(robot.revolver.getTargetSlot()) == ColorEnum.GREEN) {
            gamepad2.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (robot.revolver.getSlotColor(robot.revolver.getTargetSlot())
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
            // make sure revolver is stable before starting load sequence
            if (shootStep == 0 && robot.revolver.isReady()) {
                loadBallTimer.reset();
                ++shootStep;
            }

            if (loadBallTimer.milliseconds() > 100 && shootStep == 1) {
                robot.revolver.liftLoad();

                loadBallTimer.reset();
                ++shootStep;
            }

            if (loadBallTimer.milliseconds() > 400 && shootStep == 2) {
                robot.revolver.setSlotColor(robot.revolver.getTargetSlot(), ColorEnum.UNDEFINED);
                robot.revolver.liftReset();

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

        // cycle through motifs
        if (gamepad2.triangle && inputTimer.milliseconds() > 400) {
            robot.revolver.prevMotif();
            inputTimer.reset();

        } else if (gamepad2.cross && inputTimer.milliseconds() > 400) {
            robot.revolver.nextMotif();
            inputTimer.reset();
        }
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.revolver.liftReset();
        // always tracking for testing
        robot.turret.tracking = true;
        robot.turret.enableCamera();
        inputTimer.reset();
        gamepad2.setLedColor(255, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);

        robot.drive.driverGamepad = gamepad1;

        // TODO: change to auto park pos
        Robot.follower.setStartingPose(new Pose(72, 72));
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

        robot.revolver.mode = Revolver.Mode.INTAKE;
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

        robot.drive.update();

        if (!homeRevolver) robot.revolver.update();

        robot.turret.update();
        robot.turret.turretRotationServo.setPower((float) (-gamepad2.left_stick_x));

        Robot.follower.update();

        // always track when testing
        // TODO: re-enable
//                if (gamepad2.touchpad) {
//                    robot.turret.tracking = true;
//                } else robot.turret.tracking = false;

        dashboardTelemetry.addData("state", state);
        dashboardTelemetry.addData("target position", robot.revolver.target);
//        dashboardTelemetry.addData(
//                "current position", robot.revolver.encoderRevolver.getCurrentPosition());

        dashboardTelemetry.addData("power rotation: ", robot.revolver.revolverSpin.getPower());
        dashboardTelemetry.addData("tracking state: ", robot.turret.tracking);
        dashboardTelemetry.addData("shoot step: ", shootStep);

        dashboardTelemetry.addData("slot 0", robot.revolver.getSlotColor((byte) 0));
        dashboardTelemetry.addData("slot 1", robot.revolver.getSlotColor((byte) 1));
        dashboardTelemetry.addData("slot 2", robot.revolver.getSlotColor((byte) 2));

        dashboardTelemetry.addData("motifpos", motifPosition);
        //        dashboardTelemetry.addData("color", robot.intake.);

        dashboardTelemetry.addData("dst", robot.turret.getDistance());
        dashboardTelemetry.addData("curr odo", Robot.follower.getPose());

        dashboardTelemetry.update();

        telemetry.addData("revolver target slot", robot.revolver.getTargetSlot());
    }

    @Override
    public void stop() {}
}
