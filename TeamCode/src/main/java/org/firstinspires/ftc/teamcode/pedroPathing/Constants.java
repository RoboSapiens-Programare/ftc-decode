package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static FollowerConstants followerConstants =
            new FollowerConstants()
                    .mass(13.7)
                    .forwardZeroPowerAcceleration(-43.5)
                    .lateralZeroPowerAcceleration(-65.3)
                    .headingPIDFCoefficients(new PIDFCoefficients(1.6, 0.04, 0.07, 0.02))
                    .translationalPIDFCoefficients(new PIDFCoefficients(0.16, 0.0002, 0.01, 0.0031))
                    .secondaryTranslationalPIDFCoefficients(
                            new PIDFCoefficients(0.6, 0, 0.05, 0.04))
                    .drivePIDFCoefficients(
                            new FilteredPIDFCoefficients(0.006, 0.0008, 0.0002, 0.6, 0.05))
                    .secondaryDrivePIDFCoefficients(
                            new FilteredPIDFCoefficients(0.02, 0.005, 0.009, 0.1, 0.04))
                    .useSecondaryTranslationalPIDF(true)
                    .useSecondaryDrivePIDF(true);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                //                .driveEncoderLocalizer(driveEncoderLocalizerConstants)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

    public static MecanumConstants mecanumConstants =
            new MecanumConstants()
                    .maxPower(1)
                    .rightFrontMotorName("rightFront")
                    .rightRearMotorName("rightRear")
                    .leftRearMotorName("leftRear")
                    .leftFrontMotorName("leftFront")
                    .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .xVelocity(78.765)
                    .yVelocity(61.59);

    public static PinpointConstants localizerConstants =
            new PinpointConstants()
                    .forwardPodY(4)
                    .strafePodX(-12.4)
                    .distanceUnit(DistanceUnit.CM)
                    .hardwareMapName("pinpoint")
                    .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                    .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                    .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static DriveEncoderConstants driveEncoderLocalizerConstants =
            new DriveEncoderConstants()
                    .rightFrontMotorName("rightFront")
                    .rightRearMotorName("rightRear")
                    .leftRearMotorName("leftRear")
                    .leftFrontMotorName("leftFront")
                    .leftFrontEncoderDirection(Encoder.REVERSE)
                    .leftRearEncoderDirection(Encoder.FORWARD)
                    .rightFrontEncoderDirection(Encoder.FORWARD)
                    .rightRearEncoderDirection(Encoder.FORWARD)
                    .robotLength(27 / 2.5)
                    .robotWidth(35 / 2.5)
                    .forwardTicksToInches(26434.66703613705)
                    .strafeTicksToInches(-1.1441178787974106E7)
                    .turnTicksToInches(0.9991656241808676);
}
