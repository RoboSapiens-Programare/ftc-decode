package org.firstinspires.ftc.teamcode.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Robot.Utils.ColorEnum;
import org.firstinspires.ftc.teamcode.Robot.uV;

@Config
@TeleOp(name = "TeleOp dos")
public class TeleOPDoi extends OpMode {
    private Robot robot;

    enum State {
        INTAKE,
        OUTTAKE
    };

    enum SortingMode {
        MOTIF,
        NONE
    };

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    private State state = State.INTAKE;
    private SortingMode sortingMode = SortingMode.NONE;

    private final ElapsedTime initLoopTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime driverOneInputTimer = new ElapsedTime();
    private final ElapsedTime driverTwoInputTimer = new ElapsedTime();
    private final ElapsedTime shootTimer = new ElapsedTime();

    private void changeState(State newState) {
        state = newState;
    }

    public void handleIntake() {
        /* ----- STATE FUNCTIONS ----- */
        // power off intake and switch to outtake state
        if (gamepad1.cross && stateTimer.milliseconds() > uV.inputDelayMS) {
            robot.intake.setPower(0);

            robot.spindexer.mode = Spindexer.Mode.OUTTAKE;
            changeState(State.OUTTAKE);

            driverOneInputTimer.reset();
        }

        /* ----- MECHANICAL FUNCTIONS ----- */

        // power on the intake in either direction
        if (gamepad1.right_trigger > 0.2) {
            robot.intake.setPower(uV.intakePower);
        } else if (gamepad1.left_trigger > 0.2) {
            robot.intake.setPower(-uV.intakePower / 2);
        }

        // magnetic homing (driver 2)
        if (gamepad2.left_bumper && driverTwoInputTimer.milliseconds() > 200) {
            robot.spindexer.reset();

            driverTwoInputTimer.reset();
        }

        /* ----- SORTING SNIPPETS ----- */
        if (robot.spindexer.isSlotFull(robot.spindexer.getTargetSlot())) {
            if (robot.spindexer.getFreeSlot() != -1) {
                robot.spindexer.setTargetSlot(robot.spindexer.getFreeSlot());
            } else {
                // has 3 game elements (no free slot available) in spindexer, change to outtake
                robot.intake.setPower(0);

                changeState(State.OUTTAKE);
            }
        } else robot.spindexer.setTargetSlot(robot.spindexer.getTargetSlot());

        // change color based on current slot
        if (robot.spindexer.getSlotColor(robot.spindexer.getTargetSlot()) == ColorEnum.GREEN) {
            gamepad2.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (robot.spindexer.getSlotColor(robot.spindexer.getTargetSlot())
                == ColorEnum.PURPLE) {
            gamepad2.setLedColor(155, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
        } else gamepad2.setLedColor(255, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
    }

    public void handleOuttake() {
        /* ----- STATE FUNCTIONS ----- */
        // power off turret and change to outtake state
        if (gamepad1.cross && stateTimer.milliseconds() > uV.inputDelayMS) {
            if (robot.turret.track) {
                robot.turret.track = false;
                Robot.follower.breakFollowing();
            }

            robot.turret.reset();
            robot.spindexer.mode = Spindexer.Mode.INTAKE;

            changeState(State.INTAKE);
        }

        /* ----- MECHANICAL FUNCTIONS ----- */
        // turret tracking via chassis
        if (gamepad1.touchpad && driverOneInputTimer.milliseconds() > uV.inputDelayMS) {
            robot.turret.reset();
            robot.turret.track = !robot.turret.track;
        }

        // shooting
        // let 300ms between shots
        // allow continuous shooting
        if (gamepad1.right_trigger > 0.3 && shootTimer.milliseconds() > uV.inputDelayMS) {
            robot.spindexer.shootCurrentSlot();
            shootTimer.reset();
        }

        /* ----- SORTING SNIPPETS ----- */

        // change sort mode
        if (gamepad2.touchpad && driverTwoInputTimer.milliseconds() > uV.inputDelayMS) {
            if (sortingMode == SortingMode.NONE) {
                robot.spindexer.motifGoToStart();
                sortingMode = SortingMode.MOTIF;
            } else {
                robot.spindexer.goToShootStartPose(0);
                sortingMode = SortingMode.NONE;
            }

        }

        // change controller color based on current slot color
        if (robot.spindexer.getSlotColor(robot.spindexer.getTargetSlot()) == ColorEnum.GREEN) {
            gamepad2.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (robot.spindexer.getSlotColor(robot.spindexer.getTargetSlot())
                == ColorEnum.PURPLE) {
            gamepad2.setLedColor(155, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
        } else gamepad2.setLedColor(255, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);

        // change controller color when turret can shoot optimally
        if (robot.turret.isShootReady()) {
            gamepad1.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.turret.track = false;

        sortingMode = SortingMode.MOTIF;

        robot.spindexer.mode = Spindexer.Mode.INTAKE;
        robot.spindexer.setTargetSlot((byte) 0);

        initLoopTimer.reset();

        changeState(State.INTAKE);
    }

    @Override
    public void init_loop() {
        // blink driver 2 led
        if (((int) initLoopTimer.seconds()) % 2 == 0) {
            gamepad2.setLedColor(0, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad2.setLedColor(255, 255, 255, Gamepad.LED_DURATION_CONTINUOUS);
        }

        if (gamepad1.options && driverOneInputTimer.milliseconds() > uV.inputDelayMS) {
            if (Robot.alliance == Robot.Alliance.RED) {
                Robot.alliance = Robot.Alliance.BLUE;
                gamepad1.setLedColor(0, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
            } else {
                Robot.alliance = Robot.Alliance.RED;
                gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            }
            driverOneInputTimer.reset();
        }
    }

    @Override
    public void start() {
        gamepad1.setLedColor(255, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
        gamepad2.setLedColor(255, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);

        Robot.follower.startTeleopDrive();
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

        Robot.follower.update();

        if (!robot.turret.track)
            Robot.follower.setTeleOpDrive(
                    -gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x,
                    true
            );

        robot.turret.update();

        dashboardTelemetry.addData("state", state);
        dashboardTelemetry.addData("target Slot", robot.spindexer.getTargetSlot());
        dashboardTelemetry.addData(
                "current position", robot.spindexer.motor.getCurrentPosition());

        dashboardTelemetry.addData("power rotation: ", robot.spindexer.motor.getPower());

        dashboardTelemetry.addData("slot 0", robot.spindexer.getSlotColor((byte) 0));
        dashboardTelemetry.addData("slot 1", robot.spindexer.getSlotColor((byte) 1));
        dashboardTelemetry.addData("slot 2", robot.spindexer.getSlotColor((byte) 2));

        dashboardTelemetry.update();
    }
}
