package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.ftc.localization.Encoder;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants().mass(15);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .twoWheelLocalizer(twoWheelLocalizerConstants)
                // .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(mecanumConstants)
                .build();
    }

    public static MecanumConstants mecanumConstants =
            new MecanumConstants()
                    .maxPower(1)
                    .rightFrontMotorName("rightFront")
                    .rightRearMotorName("rightRear")
                    .leftRearMotorName("leftRear")
                    .leftFrontMotorName("leftFront")
                    .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .xVelocity(57.723720)
                    .yVelocity(57.723720)
                    .forwardZeroPowerAcceleration(-23.30059)
                    .lateralZeroPowerAcceleration(-20691.667);

    public static PinpointConstants localizerConstants =
            new PinpointConstants()
                    .forwardPodY(6)
                    .strafePodX(-16)
                    .distanceUnit(DistanceUnit.CM)
                    .hardwareMapName("pinpoint")
                    .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                    .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                    .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static TwoWheelConstants twoWheelLocalizerConstants =
            new TwoWheelConstants()
                    .forwardEncoder_HardwareMapName("leftRear") // expansion hub 2
                    .forwardEncoderDirection(Encoder.REVERSE)
                    .forwardTicksToInches(0.002056)
                    .forwardPodY(6 / 2.5)

                    .strafeEncoder_HardwareMapName("leftFront") // expansion hub 3
                    .strafeEncoderDirection(Encoder.REVERSE)
                    .strafeTicksToInches(0.000543)
                    .strafePodX(-20 / 2.5)

                    .IMU_HardwareMapName("imu")
                    .IMU_Orientation(
                            new RevHubOrientationOnRobot(
                                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                    RevHubOrientationOnRobot.UsbFacingDirection.UP));

    public static Follower createMecanumFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .build();
    }
}
