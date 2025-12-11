package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Revolver;

@Autonomous(name = "PulAuto", group = "0. Autonomous")
@Configurable // Panels
public class PulAuto extends OpMode {
    public int ball = 0;
    public int shootStep = -2;
    public ElapsedTime timer = new ElapsedTime();
    public ElapsedTime inputTimer = new ElapsedTime();
    public Robot robot;
    public boolean singleton = true;
    public DcMotorEx lr, rr, lf, rf;

    public int direction = 1;
    public ElapsedTime homingTimer = new ElapsedTime();

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.revolver.liftReset();

        robot.turret.tracking = true;
        robot.turret.enableCamera();

        robot.revolver.mode = Revolver.Mode.OUTTAKE;

        lr = hardwareMap.get(DcMotorEx.class, "leftRear");
        rr = hardwareMap.get(DcMotorEx.class, "rightRear");
        lf = hardwareMap.get(DcMotorEx.class, "leftFront");
        rf = hardwareMap.get(DcMotorEx.class, "rightFront");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lr.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rr.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void init_loop() {
        telemetry.addData("is turret ready", robot.turret.isShootReady());
        telemetry.update();

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
        robot.revolver.reset();
        robot.revolver.home();

        timer.reset();
    }

    @Override
    public void loop() {
        robot.turret.update();
        robot.revolver.update();

        telemetry.addData("is turret ready", robot.turret.isShootReady());
        telemetry.addData("alliance", (Robot.alliance));
        telemetry.addData("Searching for tag", (Robot.alliance == Robot.Alliance.RED ? 24 : 20));

        telemetry.update();

        if (singleton && timer.milliseconds() > 3000) {
            singleton = false;
            timer.reset();

            lr.setPower(0);
            rr.setPower(0);
            lf.setPower(0);
            rf.setPower(0);
            shootStep = -1;

        } else if (singleton) {
            lr.setPower(-.5);
            rr.setPower(-.5);
            lf.setPower(-.5);
            rf.setPower(-.5);
        }

        if (!(singleton && timer.milliseconds() > 3000)) return;

        if (robot.revolver.homing) {
            return;
        }

        if (ball == 3) {
            return;
        }

        if (!robot.turret.found) {
            if (homingTimer.milliseconds() >= 4500) {
                direction *= -1;
                homingTimer.reset();
            }

            robot.turret.move(direction * 0.67);
            return;
        } else {
            homingTimer.reset();
        }
        // shooting
        if (shootStep == -1) {
            robot.revolver.setTargetSlot((byte) ball);

            ++shootStep;
        }

        if (robot.revolver.isReady() && shootStep == 0) {
            robot.revolver.liftLoad();

            timer.reset();
            ++shootStep;
        }

        if (timer.milliseconds() > 1000 && shootStep == 1) {
            robot.revolver.liftReset();

            timer.reset();
            ++shootStep;
        }

        if (timer.milliseconds() > 1000 && shootStep == 2) {
            robot.revolver.liftReset();

            timer.reset();
            ++shootStep;
        }

        if (timer.milliseconds() > 1000 && shootStep == 3) {
            ++ball;
            shootStep = 0;
        }
    }
}
