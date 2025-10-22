package org.firstinspires.ftc.teamcode.kronbot.utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;

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

    public static LogoFacingDirection LOGO_FACING_DIRECTION = LogoFacingDirection.LEFT;
    public static UsbFacingDirection USB_FACING_DIRECTION = UsbFacingDirection.UP;

    public static boolean LOADER_SERVO_REVERSED = true;
}