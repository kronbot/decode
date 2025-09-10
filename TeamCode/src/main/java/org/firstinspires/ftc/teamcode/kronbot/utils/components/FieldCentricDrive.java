package org.firstinspires.ftc.teamcode.kronbot.utils.components;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CONTROLLER_DEADZONE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.POWER_EXPONENT;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ROBOT_SPEED;

import com.acmerobotics.dashboard.config.Config;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kronbot.KronBot;

import org.firstinspires.ftc.teamcode.kronbot.utils.pid.ControllerPID;

/**
 * Field centric drive is a drive system that allows the robot to move in a direction relative to the field
 *
 * @version 1.0
 */
@Config
public class FieldCentricDrive {
    KronBot robot;
    Gamepad gamepad;

    double rotatedX = 0;
    double rotatedY = 0;

    double angleOffset = 0.0;

    ControllerPID pidController = new ControllerPID(1, 0, 1);

    public FieldCentricDrive(KronBot robot, Gamepad gamepad) {
        this.robot = robot;
        this.gamepad = gamepad;
    }

    public void run() {
        robot.gyroscope.updateOrientation();
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double rotX = gamepad.right_stick_x;
        double rotY = gamepad.right_stick_y;
        // ToDo: test getHeading() and angleOffset positive and negative combos
        double robotOrientation = -robot.gyroscope.getHeading() + angleOffset;

        // Rotation matrix
        rotatedX = x * Math.cos(robotOrientation) - y * Math.sin(robotOrientation);
        rotatedY = x * Math.sin(robotOrientation) + y * Math.cos(robotOrientation);

        // Filter gamepad joystick inputs
        rotatedX = addons(rotatedX);
        rotatedY = addons(rotatedY);
        // Only used for these two for the deadzone
        rotX = addons(rotX);
        rotY = addons(rotY);

        double joystickAngleDeg = Math.toDegrees(Math.atan2(rotY, rotX));

        double rotError = normalizeAngle(joystickAngleDeg - robotOrientation);

        double r = pidController.calculate(0, rotError) / 180;
        r = Math.clamp(r, -1.0, 1.0);

        double normalizer = Math.max(Math.abs(rotatedX) + Math.abs(rotatedY) + Math.abs(r), 1.0);

        double leftFrontPower = (rotatedY + rotatedX + r) / normalizer;
        double leftRearPower = (rotatedY - rotatedX + r) / normalizer;
        double rightRearPower = (rotatedY + rotatedX - r) / normalizer;
        double rightFrontPower = (rotatedY - rotatedX - r) / normalizer;

        robot.motors.leftFront.setPower(leftFrontPower);
        robot.motors.leftRear.setPower(leftRearPower);
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
        telemetry.addData("Robot Angle", robot.gyroscope.getHeading());

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
    public void setOrientationOffset(double offset) {
        // This should automatically clamp offset to [-180, 180], worked on w3schools
        angleOffset = Math.IEEEremainder(offset, 360);
    }
}
