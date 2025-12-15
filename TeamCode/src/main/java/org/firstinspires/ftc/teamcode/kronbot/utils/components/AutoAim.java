package org.firstinspires.ftc.teamcode.kronbot.utils.components;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.pid.ControllerPID;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.AIM_KP;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.AIM_KI;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.AIM_KD;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ANGLE_TOLERANCE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.MAX_ROTATION_POWER;

public class AutoAim {
    private KronBot robot;
    private ControllerPID aimPID;

    private boolean isAiming = false;
    private double rotationPower = 0.0;

    public AutoAim(KronBot robot) {
        this.robot = robot;
        this.aimPID = new ControllerPID(AIM_KP, AIM_KI, AIM_KD);
    }

    public double calculateAimRotation(AprilTagDetection tag) {
        if (tag == null) {
            isAiming = false;
            aimPID.reset();
            return 0.0;
        }

        isAiming = true;

        // - = left ; + = right
        double targetAngle = tag.ftcPose.bearing;

        rotationPower = aimPID.calculate(0, targetAngle);

        //clamps pid engine power
        rotationPower = Math.max(-MAX_ROTATION_POWER,
                Math.min(MAX_ROTATION_POWER, rotationPower));

        return rotationPower;
    }

    public boolean isOnTarget(AprilTagDetection tag) {
        if (tag == null) return false;
        //verifies if the bearing is angle tolerance, duuh
        return Math.abs(tag.ftcPose.bearing) < ANGLE_TOLERANCE;
    }

    public void applyAimToDrive(double x, double y, double rotation) {
        double normalizer = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rotation), 1.0); // this normalizes (kind of like a clamp) the values as to not be greater than 1

        double leftFrontPower = (y + x + rotation) / normalizer; // all wheels must rotate forward (y) but whe subtract strafing (x) to create rotation and side to side movement
        double leftRearPower = (y - x + rotation) / normalizer;
        double rightRearPower = (y + x - rotation) / normalizer;
        double rightFrontPower = (y - x - rotation) / normalizer;

        //i think u know what this does.
        robot.motors.leftFront.setPower(leftFrontPower);
        robot.motors.leftRear.setPower(leftRearPower);
        robot.motors.rightRear.setPower(rightRearPower);
        robot.motors.rightFront.setPower(rightFrontPower);
    }

    public void reset() {
        aimPID.reset();
        isAiming = false;
        rotationPower = 0.0;
    }

//    public boolean isAiming() {
//        return isAiming;
//    }

    public void telemetry(Telemetry telemetry, AprilTagDetection tag) {
        telemetry.addLine("=== AUTO-AIM STATUS ===");
        telemetry.addData("Auto-Aim Active", isAiming);

        if (tag != null) {
            telemetry.addData("Target Bearing", "", tag.ftcPose.bearing);
            telemetry.addData("On Target", isOnTarget(tag));
            telemetry.addData("Rotation Power", "", rotationPower);
            telemetry.addData("Distance to Target", " cm", tag.ftcPose.range);
        } else {
            telemetry.addData("Target", "NOT DETECTED");
        }
    }

}