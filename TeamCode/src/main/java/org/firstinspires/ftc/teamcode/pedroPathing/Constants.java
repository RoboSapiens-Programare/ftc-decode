package org.firstinspires.ftc.teamcode.pedroPathing;

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
                    .mass(15)
                    .forwardZeroPowerAcceleration(-43.5)
                    .lateralZeroPowerAcceleration(-65.3)
                    .headingPIDFCoefficients(new PIDFCoefficients(3, 1, 0, 0));

    //                    .useSecondaryTranslationalPIDF(true)
    //                    .useSecondaryHeadingPIDF(true)
    //                    .useSecondaryDrivePIDF(true);

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
                    .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .xVelocity(69.8)
                    .yVelocity(53.3);

    public static PinpointConstants localizerConstants =
            new PinpointConstants()
                    .forwardPodY(12.4)
                    .strafePodX(5.6)
                    .distanceUnit(DistanceUnit.CM)
                    .hardwareMapName("pinpoint")
                    .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                    .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                    .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

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
