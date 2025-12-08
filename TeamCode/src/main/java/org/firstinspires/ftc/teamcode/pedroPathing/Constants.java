package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
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
    public static final double TValue = 0;
    public static final double Velocity = 0;
    public static final double TimeoutConstraint = 0;

    public static FollowerConstants followerConstants = new FollowerConstants()
//            code for pid tuning
            .mass(0)
            .forwardZeroPowerAcceleration(0)
            .lateralZeroPowerAcceleration(0);

    public static PathConstraints pathConstraints = new PathConstraints(
            TValue,
            Velocity,
            0,
            0,
            TimeoutConstraint,
            0,
            0,
            0);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)

            .xVelocity(0)
            .yVelocity(0);


    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("parallelOdometry")
            .strafeEncoder_HardwareMapName("perpendicularOdometry")
            .strafePodX(0)
            .forwardPodY(0)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    //modify this so it's correct
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                            RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                    )
            )
            .forwardTicksToInches(0)
            .strafeTicksToInches(0)

            //needed?
            .forwardEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.REVERSE)
            ;

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .twoWheelLocalizer(localizerConstants)
                .build();
    }
}