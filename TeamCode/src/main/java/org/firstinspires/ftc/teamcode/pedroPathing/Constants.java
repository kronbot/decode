package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(4);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            /// reverse if needed
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("parallerOdometry")
            .strafeEncoder_HardwareMapName("perpendicularOdometry")
            .strafePodX(10) /// change
            .forwardPodY(10) /// change
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            /// to be changed if needed
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                    )
            )
            .forwardTicksToInches(1/307.699557) /// change
            .strafeTicksToInches(1/307.699557) /// change

            ///  should not be necesarry
//            .forwardEncoderDirection(Encoder.REVERSE)
//            .forwardEncoderDirection(Encoder.REVERSE)
            ;


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .twoWheelLocalizer(localizerConstants)
                .build();
    }
}