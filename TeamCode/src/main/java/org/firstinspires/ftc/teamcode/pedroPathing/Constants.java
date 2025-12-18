package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@Configurable
public class Constants {
    public static FollowerConstants followerConstants =
            new FollowerConstants()
                    .mass(12)
                    .forwardZeroPowerAcceleration(-33.342664)
                    .lateralZeroPowerAcceleration(-63.882851);

    public static PathConstraints pathConstraints = new PathConstraints(0.9, 3, 0, 0);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                //                .twoWheelLocalizer(twoWheelLocalizerConstants)
                .pinpointLocalizer(localizerConstants)
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
                    .xVelocity(55.12409)
                    .yVelocity(46.47083);

    public static PinpointConstants localizerConstants =
            new PinpointConstants()
                    .forwardPodY(6.3)
                    .strafePodX(-18)
                    .distanceUnit(DistanceUnit.CM)
                    .hardwareMapName("pinpoint")
                    .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                    .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
                    .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
}
