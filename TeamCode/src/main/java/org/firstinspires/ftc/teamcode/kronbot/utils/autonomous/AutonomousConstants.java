package org.firstinspires.ftc.teamcode.kronbot.utils.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;


@Config
public final class AutonomousConstants {

    public static class Coordinates {
        public double y;
        public double x;
        public double heading;

        public Coordinates(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    public static Coordinates StartingPose = new Coordinates(30, 0, Math.toRadians(180));
    public static Coordinates LaunchZone = new Coordinates(26, 0, Math.toRadians(135));

    public static Pose coordinates(Coordinates coord) {
        return new Pose(coord.x, coord.y, coord.heading);
    }
}