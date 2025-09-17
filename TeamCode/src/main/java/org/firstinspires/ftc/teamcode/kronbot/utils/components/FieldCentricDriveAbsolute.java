package org.firstinspires.ftc.teamcode.kronbot.utils.components;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CONTROLLER_DEADZONE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.POWER_EXPONENT;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ROBOT_SPEED;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kronbot.KronBot;

import org.firstinspires.ftc.teamcode.kronbot.utils.pid.ControllerPID;

/**
 * Field centric drive absolute is a drive system that allows the robot to move in a direction relative to the field and its rotation
 *
 * @version 1.0
 */
@Config
public class FieldCentricDriveAbsolute {
    KronBot robot;
    Gamepad gamepad;

    double rotatedX = 0;
    double rotatedY = 0;

    double angleOffset = 0.0;
    double joystickAngleDeg;
    boolean wasInDeadzone = false;

    boolean reverse = false;
    boolean robotCentricMovement = true;


    ControllerPID pidController = new ControllerPID(1.8, 0.5, 0.1);

    public FieldCentricDriveAbsolute(KronBot robot, Gamepad gamepad) {
        this.robot = robot;
        this.gamepad = gamepad;
    }

    public void run() {
        robot.gyroscope.updateOrientation();
        double x = gamepad.left_stick_x;
        double y = gamepad.left_stick_y;
        double rotX = gamepad.right_stick_x;
        double rotY = gamepad.right_stick_y;

        double heading = robot.gyroscope.getHeading();
        double robotOrientation = heading + angleOffset;

        if(reverse)
        {
            x *= -1;
            y *= -1;
        }

        if(robotCentricMovement) {
            rotatedX = x;
            rotatedY = y;
        }
        else {
            // Rotation matrix
            /// Doesn't work sometimes, don't know why.
            rotatedX = x * Math.cos(Math.toRadians(-heading)) - y * Math.sin(Math.toRadians(-heading));
            rotatedY = x * Math.sin(Math.toRadians(-heading)) + y * Math.cos(Math.toRadians(-heading));
        }


        // Filter gamepad joystick inputs
        rotatedX = addons(rotatedX);
        rotatedY = addons(rotatedY);
        // Only used for these two for the deadzone
        double rotXc = addons(rotX);
        double rotYc = addons(rotY);

        if(isInDeadzone(gamepad))
        {
            if(!wasInDeadzone)
            {
                wasInDeadzone = true;
                joystickAngleDeg = -heading;
                pidController.reset();
            }
        }
        else
        {
            if(wasInDeadzone)
                wasInDeadzone = false;

            joystickAngleDeg = Math.toDegrees(Math.atan2(rotXc, -rotYc));
        }
        double rotError;

        if(reverse) // ToDo: Make sure this works
            rotError = normalizeAngle(joystickAngleDeg + robotOrientation + 180);
        else
            rotError = normalizeAngle(joystickAngleDeg + robotOrientation);

        // The robot should hopefully rotate slower when pushing the joystick less
        rotError *= joystickPower(rotX, rotY);

        double r = pidController.calculate(0, -rotError) / 180;
        r = Math.max(-1.0, Math.min(1.0, r));

        if(Math.abs(rotError) < 1.5)
        {
            r = 0;
            pidController.reset();
        }

        //r = 0;

        //double rotPower = Math.abs(r);
        //double movPower = Math.abs(rotatedX) + Math.abs(rotatedY);



        double normalizer = Math.max(Math.abs(rotatedX) + Math.abs(rotatedY) + Math.abs(r), 1.0);

        double leftFrontPower = (rotatedY + rotatedX + r) / normalizer;
        double leftRearPower = (rotatedY - rotatedX + r) / normalizer;
        double rightRearPower = (rotatedY + rotatedX - r) / normalizer;
        double rightFrontPower = (rotatedY - rotatedX - r) / normalizer;

        robot.motors.leftFront.setPower(leftFrontPower);
        robot.motors.leftRear.setPower(-leftRearPower);
        robot.motors.rightRear.setPower(rightRearPower);
        robot.motors.rightFront.setPower(rightFrontPower);
    }

    /** Helper function used to smooth out input values from a controller joystick.
     *  Provides finer control for lower values and cancels the deadzone.
     */
    double addons(double value) {
        if (Math.abs(value) < CONTROLLER_DEADZONE) return 0;
        double normalizedValue = (Math.abs(value) - CONTROLLER_DEADZONE) / (1 - CONTROLLER_DEADZONE);
        double poweredValue = Math.pow(normalizedValue, POWER_EXPONENT);
        return Math.signum(value) * poweredValue * ROBOT_SPEED; // ROBOT_SPEED should always be 1
    }

    /**
     * Returns the modulus of the joystick, squared
     * (this means that the user has higher control at smaller modulus)
     */
    static double joystickPower(double x, double y)
    {
        return x * x + y * y;
    }

    static double normalizeAngle(double angle)
    {
        angle = angle % 360;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }


    /**
     *  Simple helper functon that returns true when left joystick is in deadzone. <br>
     *  (not normalized, since I reckon a square is a good enough approximation)
     */
    static boolean isInDeadzone(Gamepad gamepad)
    {
        return Math.abs(gamepad.left_stick_x) < CONTROLLER_DEADZONE &&
                Math.abs(gamepad.left_stick_y) < CONTROLLER_DEADZONE;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("---FIELD CENTRIC DRIVE---");

        telemetry.addData("Speed Multiplier", ROBOT_SPEED);
        telemetry.addData("Robot Angle", robot.gyroscope.getHeading() + angleOffset);

        telemetry.addData("rotatedX", rotatedX);
        telemetry.addData("rotatedY", rotatedY);

        telemetry.addData("LeftRear Position", robot.motors.leftRear.getCurrentPosition());
        telemetry.addData("RightRear Position", robot.motors.rightRear.getCurrentPosition());
        telemetry.addData("LeftFront Position", robot.motors.leftFront.getCurrentPosition());
        telemetry.addData("RightFront Position", robot.motors.rightFront.getCurrentPosition());

        telemetry.addData("LeftRear Power", robot.motors.leftRear.getPower());
        telemetry.addData("RightRear Power", robot.motors.rightRear.getPower());
        telemetry.addData("LeftFront Power", robot.motors.leftFront.getPower());
        telemetry.addData("RightFront Power", robot.motors.rightFront.getPower());
    }

    /**
     * By default, when the joystick is at (0, 1)(forward), the robot will go
     * in the forward direction that it was initialized in. With this function,
     * you can add an angle offset to what the robot considers to be forward. <br>
     * If you give 90, on joystick forward, the robot will go left (from the
     * initialised orientation). <br>
     * If you give -90, on joystick forward, the robot will go right (from the
     * initialised orientation). <br><br>
     * Can be called in Init or during TeleOp
     */
    public void calibrateOrientation() {
        // This should automatically clamp offset to [-180, 180], worked on w3schools
        angleOffset -= robot.gyroscope.getHeading() + angleOffset;
        joystickAngleDeg = 0;
    }

    /**
     * Reverse what direction is forward, basically
     */
    public void setReverse(boolean reverse)
    {
        this.reverse = reverse;
    }

    /**
     * If true, when moving the left joystick forward, the robot will move
     * in it's forward direction.<br>
     * If false, when moving the left joystick forward, the robot will move
     * forward in the field (the same direction as when pointing forwards)
     */
    public void setDrivingMode(boolean mode)
    {
        robotCentricMovement = mode;
    }

}