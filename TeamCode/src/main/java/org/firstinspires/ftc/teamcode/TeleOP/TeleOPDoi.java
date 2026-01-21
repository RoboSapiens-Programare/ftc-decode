package org.firstinspires.ftc.teamcode.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Utils.ColorEnum;
import org.firstinspires.ftc.teamcode.Robot.uV;

@Config
@TeleOp(name = "TeleOp dos FIXED")
public class TeleOPDoi extends OpMode {
    private Robot robot;

    private boolean homingExecOnce = false;
    private boolean homingExecOnce2 = false;
    private ElapsedTime homingFixTimer = new ElapsedTime();

    enum State {
        INTAKE,
        OUTTAKE
    }

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    private final Pose startPose = new Pose(64, 56, Math.PI/2);

    private State state = State.INTAKE;

    private final ElapsedTime initLoopTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime driverOneInputTimer = new ElapsedTime();
    private final ElapsedTime driverTwoInputTimer = new ElapsedTime();
    private final ElapsedTime matchTimer = new ElapsedTime();
    private final ElapsedTime shootTimer = new ElapsedTime();

    private void changeState(State newState) {
        state = newState;
        stateTimer.reset();

//        robot.spindexer.setTargetSlot((byte) 0);
        robot.shooter.shooting = state == State.OUTTAKE;
    }

    public void handleIntake() {
        /* ----- STATE FUNCTIONS ----- */
        // power off intake and switch to outtake state
        if (gamepad1.cross && stateTimer.milliseconds() > uV.inputDelayMS) {
            robot.intake.setPower(0);

            changeState(State.OUTTAKE);

            driverOneInputTimer.reset();
        }

        /* ----- MECHANICAL FUNCTIONS ----- */

        // power on the intake in either direction
        if (gamepad1.right_trigger > 0.2) {
            robot.intake.setPower(uV.intakePower);
        } else if (gamepad1.left_trigger > 0.2) {
            robot.intake.setPower(-uV.intakePower);
        } else {
            robot.intake.setPower(0);
        }

        // magnetic homing (driver 2)
        if (gamepad2.circle && driverTwoInputTimer.milliseconds() > uV.inputDelayMS) {
            robot.spindexer.reset();
            robot.spindexer.home();

            driverTwoInputTimer.reset();
        }

        /* ----- SORTING SNIPPETS ----- */
        if (robot.spindexer.isSlotFull(robot.spindexer.getTargetSlot())) {
            byte slot = robot.spindexer.getFreeSlot();
            if (slot != -1) {
                robot.spindexer.goToSlot(slot);
            }
        }

        dashboardTelemetry.addData(
                "is it full?", robot.spindexer.isSlotFull(robot.spindexer.getTargetSlot()));
        dashboardTelemetry.addData("target", robot.spindexer.getTargetSlot());
        dashboardTelemetry.addData("first free", robot.spindexer.getFreeSlot());

        // change color based on current slot
        updateGamepadLEDForSlotColor();
    }

    public void handleOuttake() {
        /* ----- STATE FUNCTIONS ----- */
        // power off shooter and change to intake state
        if (gamepad1.cross && stateTimer.milliseconds() > uV.inputDelayMS) {
            if (robot.shooter.isTracking) {
                robot.shooter.isTracking = false;
                Robot.follower.breakFollowing();
            }

            robot.shooter.reset();

            robot.shooter.stop();

            changeState(State.INTAKE);
            stateTimer.reset();
        }

        /* ----- MECHANICAL FUNCTIONS ----- */
        // shooter tracking via chassis
        if (gamepad1.touchpad && driverOneInputTimer.milliseconds() > uV.inputDelayMS) {
            robot.shooter.track();

            driverOneInputTimer.reset();
        }

        // shooting with proper debouncing - wait for spindexer to be ready

        if (gamepad1.left_bumper && driverOneInputTimer.milliseconds() > uV.inputDelayMS) {
            Shooter.targetVelocity = 1400;
            driverTwoInputTimer.reset();
        }

        if (gamepad1.right_bumper && driverOneInputTimer.milliseconds() > uV.inputDelayMS) {
            Shooter.targetVelocity = 1100;
            driverTwoInputTimer.reset();
        }

        if (gamepad1.right_trigger > 0.1) {  // Rising edge detection
            robot.intake.setPower(1);
            if (robot.shooter.isShootReady() && robot.spindexer.isReady()) {
                robot.spindexer.shoot();

                /*
                int slot = robot.spindexer.getTargetSlot();
                robot.spindexer.setSlotColor(slot, ColorEnum.UNDEFINED);

                robot.spindexer.targetPosition -= 8192.0 / 3;
                robot.spindexer.targetSlot -= 1;
                if (robot.spindexer.targetSlot == -1) {
                    robot.spindexer.targetSlot = 2;
                }
                 */
            }
        } else {
            robot.spindexer.shootDirection = -1;
        }


        // Update gamepad LEDs
        updateGamepadLEDForSlotColor();

        // change controller color when shooter can shoot optimally
        if (robot.shooter.isShootReady() && robot.spindexer.isReady()) {
            gamepad1.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }

    private void updateGamepadLEDForSlotColor() {
        ColorEnum currentSlotColor = robot.spindexer.getSlotColor(robot.spindexer.getTargetSlot());

        if (currentSlotColor == ColorEnum.GREEN) {
            gamepad2.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (currentSlotColor == ColorEnum.PURPLE) {
            gamepad2.setLedColor(155, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad2.setLedColor(255, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.shooter.isTracking = false;


        robot.spindexer.goToSlot((byte) 0);

        initLoopTimer.reset();

        Robot.follower.setStartingPose(startPose);

        changeState(State.INTAKE);

        // TODO: remove in final version
        Robot.alliance = Robot.Alliance.BLUE;
//        robot.spindexer.home();
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

        robot.spindexer.home();
        matchTimer.reset();
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

            default:
                Robot.follower.update();
        }

        // CRITICAL FIX: Always update all subsystems
        Robot.follower.update();
        robot.spindexer.update();
        robot.intake.update();
        robot.shooter.update();  // ALWAYS update shooter, not just in OUTTAKE

        if (!robot.spindexer.homing && robot.spindexer.homingSingleton && robot.spindexer.getBallCount() == 3) {
            homingExecOnce = true;
            homingExecOnce2 = false;
            homingFixTimer.reset();
            robot.spindexer.homingSingleton = false;
        }

        if (homingExecOnce) {
            if (homingFixTimer.milliseconds() > 500 && !homingExecOnce2) {
                robot.spindexer.targetPosition += 8192.0 / 8;
                homingExecOnce2 = true;
            }
            if (homingFixTimer.milliseconds() > 1500 && homingExecOnce2) {
                robot.spindexer.targetPosition -= 8192.0 / 8;
                homingExecOnce2 = false;
                homingExecOnce = false;
            }
        }

        // Only allow manual drive when not tracking
        if ((Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1 || Math.abs(gamepad2.right_stick_y) > 0.1) && robot.shooter.isTracking) {
            Robot.follower.breakFollowing();
            Robot.follower.startTeleOpDrive();
            robot.shooter.isTracking = false;
        }

        if (!robot.shooter.isTracking) {
            Robot.follower.setTeleOpDrive(
                    -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        }

        if (gamepad1.dpad_left && driverOneInputTimer.milliseconds() > uV.inputDelayMS) {
            robot.spindexer.targetPosition -= 8192.0/8;

            driverOneInputTimer.reset();
        }

        if (gamepad1.dpad_right && driverOneInputTimer.milliseconds() > uV.inputDelayMS) {
            robot.spindexer.targetPosition += 8192.0/8;

            driverOneInputTimer.reset();
        }

        if (gamepad2.left_trigger > 0.1) {
            robot.intake.setRollerPower(1, -1);
        }

        if (gamepad2.right_trigger > 0.1) {
            robot.intake.setRollerPower(-1, 1);
        }

        // Telemetry
        telemetry.addData("0. Ball count", robot.spindexer.getBallCount());
        telemetry.addData("1.   slot 0", robot.spindexer.getSlotColor(0));
        telemetry.addData("2.   slot 1", robot.spindexer.getSlotColor(1));
        telemetry.addData("3.   slot 2", robot.spindexer.getSlotColor(2));
        telemetry.addData("4. State", state);
        telemetry.addData("5. Homing", robot.spindexer.homing);
        telemetry.addData("6. Match Time", matchTimer.seconds());
        telemetry.addData("7. Shoot mode", Shooter.targetVelocity == 1100 ? "close" : "far");

        telemetry.update();
    }
}