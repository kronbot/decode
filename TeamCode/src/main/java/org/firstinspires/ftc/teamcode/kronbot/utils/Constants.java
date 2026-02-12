package org.firstinspires.ftc.teamcode.kronbot.utils;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;

import org.firstinspires.ftc.teamcode.kronbot.autonomous.AutonomousConstants;

/**
 * Constants for KronBot viewable from dashboard
 *
 * @version 1.0
 */
@Config
public class Constants {
    public final static String TEST_GROUP = "test";
    public final static String MAIN_GROUP = "main";

    public static double CONTROLLER_DEADZONE = 0.15;

    public static int BUTTON_LONG_PRESS_TIME = 750;

    public static double ROBOT_SPEED = 1.0;
    public static double POWER_EXPONENT = 2.0;

    public static double LIFT_POWER = 1.0;
    public static double LIFT_REST_POWER = 0.0005;
    public static double LIFT_REVERSE_POWER = 0.9;
    public static double LIFT_TOLERANCE = 50;
    public static int LIFT_INIT_POSITION = 0;
    public static int LIFT_ACTION_POSITION = 1100;
    public static int LIFT_MAX_POSITION = 3200;

    public static double CAMERA_TRASH_HOLD = 0.2;
    public static int BLUE_HUE_LOW = 0;
    public static int BLUE_HUE_HIGH = 180;

    public static LogoFacingDirection LOGO_FACING_DIRECTION = LogoFacingDirection.RIGHT;
    public static UsbFacingDirection USB_FACING_DIRECTION = UsbFacingDirection.UP;

    public static boolean LOADER_SERVO_REVERSED = true;
    public static double TURRET_SERVO_MAX = 0.82;
    public static double TURRET_SERVO_MIN = 0.19;
    public static double TURRET_SERVO_UNITS_PER_RAD = 0.160746493;
    public static double ANGLE_SERVO_MAX = 0.72;
    public static double ANGLE_SERVO_CLOSE = 0.7;

    public static double ANGLE_SERVO_FAR = 0.72;
    public static double ANGLE_SERVO_MIN = 0;
    public static double FLAP_CLOSED = 0;
    public static double FLAP_OPEN = 0.6;

    public static double minVelocity = 1140;
    public static double maxVelocity = 1500;

    public static double RANGE_1_ANGLE = 0;
    public static double RANGE_1_VELOCITY = 950;

    public static double RANGE_2_ANGLE = 0.5;
    public static double RANGE_2_VELOCITY = 1080;

    public static double RANGE_3_ANGLE = 0.72;
    public static double RANGE_3_VELOCITY = 1200;

    public static double RANGE_4_ANGLE = 0.72;
    public static double RANGE_4_VELOCITY = 1400;

    public static AutonomousConstants.Coordinates TestPoseStart = new AutonomousConstants.Coordinates(0, 0, 0);

    //Tuning Constants
    public static double TEST_LAUNCH_MOTOR_DELTA = 0.1;
    public static double TEST_LAUNCH_ANGLE_DELTA = 0.075;
    public static double TEST_TURRET_PIVOT_DELTA = 0.05;

    // Outtake motor PID
    public static double OUT_MOTOR_KP = 7;
    public static double OUT_MOTOR_KI = 0;
    public static double OUT_MOTOR_KD = 0.8;
    public static double OUT_MOTOR_KF = 10;


    //AIM PID - to be tuned
    public static double AIM_KP = 0.02;
    public static double AIM_KI = 0.0;
    public static double AIM_KD = 0.003;
    public static double ANGLE_TOLERANCE = 2.0;
    public static double MAX_ROTATION_POWER = 0.5;
}