package org.firstinspires.ftc.teamcode.kronbot.utils.components;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.kronbot.Robot;

public class TurretAligner {
    private final Robot robot;
    private double targetX = 0;
    private double targetY = 0;

    // Simple P-gain if you want to smooth the movement
    // (1.0 means it snaps instantly to the target)
    private double kP = 0.8;

    public TurretAligner(Robot robot) {
        this.robot = robot;
    }

    public void setTarget(double x, double y) {
        this.targetX = x;
        this.targetY = y;
    }

    public void update() {
        Pose currentPose = robot.follower.getPose();

        // 1. Calculate the difference
        double dx = targetX - currentPose.getX();
        double dy = targetY - currentPose.getY();

        // 2. Calculate the global angle to the target (radians)
        // atan2 takes (y, x)
        double globalTargetAngle = Math.atan2(dy, dx);

        robot.turret.angle = globalTargetAngle;
    }

    /**
     * Normalizes an angle to be between -Pi and Pi
     */
    private double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}