package org.firstinspires.ftc.teamcode.kronbot.utils.components;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.kronbot.Robot;

public class TurretAligner {
    private final Robot robot;
    private Pose target;

    // Simple P-gain if you want to smooth the movement
    // (1.0 means it snaps instantly to the target)
    private double kP = 0.8;

    public TurretAligner(Robot robot) {
        this.robot = robot;
    }

    public void setTarget(double x, double y) {
        target = new Pose(x, y);
    }

    public void update() {
        Pose currentPose = robot.follower.getPose();

        // Angle from robot to the target
        double r = Math.atan2(target.getY() - currentPose.getY(), target.getX() - currentPose.getX());

        double turret = r - currentPose.getHeading();
        if(Math.abs(turret) > Math.PI) {
            turret += 2 * Math.PI;
        }



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