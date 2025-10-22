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

import java.sql.Time;

public class Constants {
    public static final double TValue = 0.99;
    public static final double Velocity = 0.1;
    public static final double TimeoutConstraint = 100;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(6.54)
            .forwardZeroPowerAcceleration(-25.9346931313679598)
            .lateralZeroPowerAcceleration(-67.342491844080064);

    public static PathConstraints pathConstraints = new PathConstraints(
            TValue,
            Velocity,
            0.1,
            0.009,
            TimeoutConstraint,
            1.25,
            10,
            1);

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
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            /// ?
            .xVelocity(78.261926752421046666666666666667)
            .yVelocity(61.494551922189565);


    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("parallelOdometry")
            .strafeEncoder_HardwareMapName("perpendicularOdometry")
            .strafePodX(7.36) /// change
            .forwardPodY(7.36) /// change
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            /// to be changed if needed
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            )
            .forwardTicksToInches(1.0/307.699557) /// change
            .strafeTicksToInches(1.0/307.699557) /// change

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