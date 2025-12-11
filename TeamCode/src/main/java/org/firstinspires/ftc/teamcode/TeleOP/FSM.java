package org.firstinspires.ftc.teamcode.TeleOP;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Revolver;
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

    private final ElapsedTime opModeTimer = new ElapsedTime();

    private final ElapsedTime inputTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime loadBallTimer = new ElapsedTime();
    private boolean singletonLoad = true;
    private boolean doOnceState = true;
    private int shootStep = -1;

    private Pose startPose;
    private Pose shootPose;

    private boolean autoShoot = false;

    private int motifPosition = 0;
    // TODO: change to false in final version
    private boolean sortMotif = true;

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
        robot.revolver.liftReset();

        motifPosition = 0;

        doOnceState = true;
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

        if (gamepad2.right_trigger > 0.5) {
            sortingMode = SortingMode.AUTO;
        }
    }

    public void handleIntake() {
        if (doOnceState) {
            robot.revolver.mode = Revolver.Mode.INTAKE;
            robot.turret.turretMotor.setPower(0);
            robot.turret.disableCamera();
            robot.intake.enableCamera();

            doOnceState = false;
        }

        if (!robot.revolver.homing) {
            robot.intake.update();
        }

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

        shootStep = -1;
    }

    public void handleOuttake() {
        if (doOnceState) {
            robot.revolver.mode = Revolver.Mode.OUTTAKE;
            robot.turret.enableCamera();
            robot.intake.disableCamera();
            doOnceState = false;
        }
        // go to shoot position automatically

        // TODO: pedro tuning

        if (robot.revolver.getBallCount() == 0) {
            changeState(State.INTAKE);
        }

        if (gamepad1.left_trigger > .5) {
            robot.turret.override = true;
        } else {
            robot.turret.override = false;
        }

        if (gamepad1.right_bumper && inputTimer.milliseconds() > 400 && !autoShoot) {
            autoShoot = true;

            Path toShootZone = new Path(new BezierLine(Robot.follower.getPose(), shootPose));
            toShootZone.setLinearHeadingInterpolation(
                    Robot.follower.getPose().getHeading(), shootPose.getHeading());
            Robot.follower.followPath(toShootZone);
            dashboardTelemetry.addData("pula", "urmeaza");

            inputTimer.reset();
        }

        if (!robot.turret.tracking) {
            robot.turret.turretMotor.setPower(0.02);
        }

        if (gamepad1.cross && stateTimer.milliseconds() > 400) {
            robot.intake.setPower(0);
            changeState(State.INTAKE);
        }

        if (gamepad2.dpad_up && inputTimer.milliseconds() > 400) {
            robot.revolver.liftLoad();
            inputTimer.reset();
        }

        if (gamepad2.dpad_down && inputTimer.milliseconds() > 400) {
            robot.revolver.liftReset();
            inputTimer.reset();
        }

        switch (sortingMode) {
            case AUTO:
                if (sortMotif) {
                    byte t = robot.revolver.getSlotByMotifPosition(motifPosition);
                    FtcDashboard.getInstance().getTelemetry().addData("t", (int)t);
                    if (t != -1) {
                        robot.revolver.setTargetSlot(t);
                    } else {
                        t = robot.revolver.getFullSlot();
                        robot.revolver.setTargetSlot(t != -1 ? t : 1);
                    }

                } else if (robot.revolver.getFullSlot() != -1) {
                    robot.revolver.setTargetSlot(robot.revolver.getFullSlot());
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
        //        if (gamepad1.right_trigger > 0.5 && stateTimer.milliseconds() > 300 && shootStep
        // == -1 && robot.turret.isShootReady()) {
        if (gamepad1.right_trigger > 0.5 && stateTimer.milliseconds() > 300 && shootStep == -1) {
            shootStep = 0;
        }

        // Shoot sequence
        // Load only if on shoot mode
        if (shootStep >= 0) {
            // make sure revolver is stable before starting load sequence
            if (shootStep == 0 && robot.revolver.isReady()) {
                loadBallTimer.reset();
                ++shootStep;
            }

            if (loadBallTimer.milliseconds() > 300 && shootStep == 1) {
                robot.revolver.liftLoad();

                loadBallTimer.reset();
                ++shootStep;
            }

            if (loadBallTimer.milliseconds() > 500 && shootStep == 2) {
                robot.revolver.liftReset();
                robot.revolver.setSlotColor(robot.revolver.getTargetSlot(), ColorEnum.UNDEFINED);

                loadBallTimer.reset();
                ++shootStep;
            }

            if (robot.revolver.mihaiLimit.isPressed() && shootStep == 3) {
                loadBallTimer.reset();
                shootStep = -1;

                if (motifPosition == 2) {
                    motifPosition = 0;
                } else
                    ++motifPosition;
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

        if (Robot.alliance == Robot.Alliance.RED) {
            startPose = new Pose(72, 72, Math.toRadians(0));
            shootPose = new Pose(72, 72, Math.toRadians(45));

        } else {
            startPose = new Pose(72, 72, Math.toRadians(0));
            shootPose = new Pose(72, 72, Math.toRadians(-45));
        }

        Robot.follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {
        if (Robot.alliance == Robot.Alliance.RED) {
            gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);

        } else {
            gamepad1.setLedColor(0, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
        }

        if (gamepad1.options && inputTimer.milliseconds() > 400) {
            if (Robot.alliance == Robot.Alliance.RED) {
                Robot.alliance = Robot.Alliance.BLUE;

            } else {
                Robot.alliance = Robot.Alliance.RED;
            }

            inputTimer.reset();
        }
    }

    @Override
    public void start() {
        changeState(State.INTAKE);
        robot.revolver.mode = Revolver.Mode.INTAKE;
        sortingMode = SortingMode.AUTO;

        gamepad1.setLedColor(255, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
        gamepad2.setLedColor(255, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
        robot.revolver.setTargetSlot((byte) 0);

        Robot.follower.startTeleOpDrive();

        robot.revolver.reset();
        robot.revolver.home();

        opModeTimer.reset();
    }

    @Override
    public void loop() {

        Robot.follower.update();

        robot.revolver.update();
        robot.turret.update();

        if (gamepad2.left_bumper && inputTimer.milliseconds() > 400) {
            robot.revolver.reset();
            robot.revolver.home();
            inputTimer.reset();
        }

        if (gamepad2.right_bumper && inputTimer.milliseconds() > 400) {
            robot.revolver.reset();
            inputTimer.reset();
        }

        // TODO: enable in teleop
        // if (opModeTimer.seconds() >= 90) {
            // sortMotif = true;
        // }

        switch (state) {
            case INTAKE:
                handleIntake();
                break;

            case OUTTAKE:
                handleOuttake();
                break;
        }

        if (autoShoot && (gamepad1.right_bumper || !Robot.follower.isBusy())) {
            autoShoot = false;
            Robot.follower.startTeleopDrive();
        }

        if (!autoShoot) {
            Robot.follower.setTeleOpDrive(
                    applyDeadzone(-gamepad1.left_stick_y),
                    applyDeadzone(-gamepad1.left_stick_x),
                    applyDeadzone(-gamepad1.right_stick_x),
                    true // Robot Centric
                    );
        }

        if (gamepad2.left_trigger > 0.5 && inputTimer.milliseconds() > 500) {
            robot.revolver.prevMotif();
            inputTimer.reset();

        } else if (gamepad2.right_trigger > 0.5 && inputTimer.milliseconds() > 500) {
            robot.revolver.nextMotif();
            inputTimer.reset();
        }

        robot.turret.move((float) (-gamepad2.left_stick_x) * 0.45);

        dashboardTelemetry.addData("current", robot.turret.turretMotor.getVelocity());
        dashboardTelemetry.addData("target", robot.turret.targetVelocity);
        dashboardTelemetry.addData("distance", robot.turret.getDistance());
        dashboardTelemetry.addData("target", robot.turret.targetVelocity);
        dashboardTelemetry.addData("pose", Robot.follower.getPose());
        dashboardTelemetry.addData("is turret ready", robot.turret.isShootReady());
        dashboardTelemetry.addData("alliance", (Robot.alliance));

        dashboardTelemetry.addData("slot 0", robot.revolver.colorList[0]);
        dashboardTelemetry.addData("slot 1", robot.revolver.colorList[1]);
        dashboardTelemetry.addData("slot 2", robot.revolver.colorList[2]);

        dashboardTelemetry.addData("left limit", (robot.turret.leftLimit.isPressed()));
        dashboardTelemetry.addData("right limit", (robot.turret.rightLimit.isPressed()));

        dashboardTelemetry.addData("green position", (robot.revolver.greenPosition));
        dashboardTelemetry.addData("motif position", (motifPosition));
        
        dashboardTelemetry.addData("Searching for tag", (Robot.alliance == Robot.Alliance.RED ? 24 : 20));
        dashboardTelemetry.update();

        StringBuilder pattern = new StringBuilder("PPP");
        pattern.setCharAt(greenPosition, 'G');
        telemetry.addData("Pattern", pattern.toString());
        telemetry.update();
    }

    @Override
    public void stop() {}
}
