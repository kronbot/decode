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
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TURRET_SERVO_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TURRET_SERVO_MAX;

public class AutoAim {
    private KronBot robot;
    private ControllerPID aimPID;

    private boolean isAiming = false;
    private double servoPosition = 0.5;
    private double servoOffset = 0.0; // Offset in servo position units (0.0-1.0) to account for servo home position
    private double gearRatio = 1.0; // Ratio between driver gear and turret ring (turretRotation = servoRotation * gearRatio)

    public AutoAim(KronBot robot) {
        this.robot = robot;
        this.aimPID = new ControllerPID(AIM_KP, AIM_KI, AIM_KD);
        this.servoOffset = (TURRET_SERVO_MIN + TURRET_SERVO_MAX) / 2.0; // Default to center
    }

    public AutoAim(KronBot robot, double servoOffset) {
        this.robot = robot;
        this.aimPID = new ControllerPID(AIM_KP, AIM_KI, AIM_KD);
        this.servoOffset = servoOffset;
    }

    public AutoAim(KronBot robot, double servoOffset, double gearRatio) {
        this.robot = robot;
        this.aimPID = new ControllerPID(AIM_KP, AIM_KI, AIM_KD);
        this.servoOffset = servoOffset;
        this.gearRatio = gearRatio;
    }

    public void setServoOffset(double offset) {
        this.servoOffset = offset;
    }

    public double getServoOffset() {
        return this.servoOffset;
    }

    public void setGearRatio(double ratio) {
        this.gearRatio = ratio;
    }

    public double getGearRatio() {
        return this.gearRatio;
    }

    public double calculateServoPosition(AprilTagDetection tag) {
        if (tag == null) {
            isAiming = false;
            aimPID.reset();
            return servoOffset; // Return to offset position when no target
        }

        isAiming = true;

        // Get target bearing in degrees (- = left ; + = right)
        double targetBearing = tag.ftcPose.bearing;

        // Use PID to calculate correction based on bearing error
        // Target is 0 degrees (centered on target), current is the bearing angle
        double pidCorrection = aimPID.calculate(0, targetBearing);

        // Convert PID output to servo position change
        // Scale the PID output to servo range, accounting for gear ratio
        double servoRange = TURRET_SERVO_MAX - TURRET_SERVO_MIN;
        double servoMovement = pidCorrection * servoRange / gearRatio;

        // Calculate final servo position: offset + PID-controlled movement
        servoPosition = servoOffset + servoMovement;

        // Clamp servo position to valid range
        servoPosition = Math.clamp(servoPosition, TURRET_SERVO_MIN, TURRET_SERVO_MAX);

        return servoPosition;
    }

    public double getServoPosition() {
        return servoPosition;
    }

    public boolean isOnTarget(AprilTagDetection tag) {
        if (tag == null) return false;
        //verifies if the bearing is angle tolerance, duuh
        return Math.abs(tag.ftcPose.bearing) < ANGLE_TOLERANCE;
    }

    public void reset() {
        aimPID.reset();
        isAiming = false;
        servoPosition = servoOffset;
    }

//    public boolean isAiming() {
//        return isAiming;
//    }

    public void telemetry(Telemetry telemetry, AprilTagDetection tag) {
        telemetry.addLine("=== AUTO-AIM STATUS ===");
        telemetry.addData("Auto-Aim Active", isAiming);
        telemetry.addData("Servo Offset", "%.3f", servoOffset);
        telemetry.addData("Gear Ratio", "%.2f:1", gearRatio);

        if (tag != null) {
            telemetry.addData("Target Bearing", "%.1f°", tag.ftcPose.bearing);
            telemetry.addData("On Target", isOnTarget(tag));
            telemetry.addData("Servo Position", "%.3f", servoPosition);
            telemetry.addData("Distance to Target", "%.1f cm", tag.ftcPose.y);
        } else {
            telemetry.addData("Target", "NOT DETECTED");
        }
    }

}
