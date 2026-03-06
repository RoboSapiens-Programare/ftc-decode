//package org.firstinspires.ftc.teamcode.TeleOP;
//
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.teamcode.Robot.Robot;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import java.text.DecimalFormat;
//
//@TeleOp(name = "TeleOp balance")
//public class AutomaticBalancing extends OpMode {
//
//    private Robot robot;
//
//    private ElapsedTime runtime = new ElapsedTime();
//
//    static final double kOffBalanceAngleThresholdDegrees = 5.0f;
//    static final double kOnBalanceAngleThresholdDegrees  = 5.0f;
//
//    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
//    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
//
//    private boolean calibration_complete = false;
//    private boolean autoBalanceXMode = false;
//    private boolean autoBalanceYMode = false;
//
//    DecimalFormat df;
//
//    @Override
//    public void init() {
//        robot = new Robot(hardwareMap);
//        Robot.follower = Constants.createFollower(hardwareMap);
//
//        Robot.follower.startTeleOpDrive(true);
//
//        Robot.follower.setStartingPose(new Pose(0, 0));
//    }
//
//    public double limit(double a) {
//        return Math.min(Math.max(a, MIN_MOTOR_OUTPUT_VALUE), MAX_MOTOR_OUTPUT_VALUE);
//    }
//
//    @Override
//    public void loop() {
//        YawPitchRollAngles angles = robot.imu.getRobotYawPitchRollAngles();
//
//            double xAxisRate = gamepad1.left_stick_x;
//            // (note: The joystick goes negative when pushed forwards, so negate it)
//            double yAxisRate = -gamepad1.left_stick_y;
//            double pitchAngleDegrees = angles.getPitch(AngleUnit.DEGREES);
//            double rollAngleDegrees = angles.getRoll(AngleUnit.DEGREES);
//
//            if (!autoBalanceXMode &&
//                    (Math.abs(pitchAngleDegrees) >=
//                            Math.abs(kOffBalanceAngleThresholdDegrees))) {
//                autoBalanceXMode = true;
//            } else if (autoBalanceXMode &&
//                    (Math.abs(pitchAngleDegrees) <=
//                            Math.abs(kOnBalanceAngleThresholdDegrees))) {
//                autoBalanceXMode = false;
//            }
//            if (!autoBalanceYMode &&
//                    (Math.abs(pitchAngleDegrees) >=
//                            Math.abs(kOffBalanceAngleThresholdDegrees))) {
//                autoBalanceYMode = true;
//            } else if (autoBalanceYMode &&
//                    (Math.abs(pitchAngleDegrees) <=
//                            Math.abs(kOnBalanceAngleThresholdDegrees))) {
//                autoBalanceYMode = false;
//            }
//
//            // Control drive system automatically,
//            // driving in reverse direction of pitch/roll angle,
//            // with a magnitude based upon the angle
//
//            if ( autoBalanceXMode ) {
//                double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
//                xAxisRate = Math.sin(pitchAngleRadians) * -1;
//            }
//
//            if ( autoBalanceYMode ) {
//                double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
//                yAxisRate = Math.sin(rollAngleRadians) * -1;
//            }
//
//            Robot.follower.setTeleOpDrive(
//                xAxisRate,
//                yAxisRate,
//                -gamepad1.right_stick_x - 0.1 * gamepad2.right_stick_x,
//                true);
//
//            Robot.follower.update();
//
//            // At this point, the X/Axis motion rates are proportional to the
//            // angle, and in the inverse direction.
//
//            // NOTE:  This algorithm assumes an omni-directional drive system (e.g., Mecanum)
//            // that can navigate linearly in both X and Y axis direction.  Tank-style drive
//            // systems (without the ability to travel in a linear direction in the "strafe"
//            // [side-to-side] direction will require additional logic.
//            telemetry.addData("Pitch Angle (degrees):", pitchAngleDegrees);
//            telemetry.addData("Roll Angle (degrees): ", rollAngleDegrees);
//            telemetry.addData("X Axis Balance Rate:  ", xAxisRate);
//            telemetry.addData("X Axis Balance Rate:  ", yAxisRate);
//        }
//    }
//
