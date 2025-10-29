package org.firstinspires.ftc.teamcode.kronbot.utils.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;


@Config
public final class AutonomousConstants {
    public static class Coordinates {
        public double x;
        public double y;
        public double heading;

        public Coordinates(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    public static final Pose FIELD_CENTER = new Pose(0, 0, 0);

    public static Coordinates StartingPose = new Coordinates(0, 0, 0);
    public static Coordinates LaunchZone = new Coordinates(20, 0, 0);

    public static Coordinates MidPose1 = new Coordinates(40, 15, 0);
    public static Coordinates MidPose2 = new Coordinates(57, 27, 0.85);


    public static Pose coordinates(Coordinates coord) {
        return new Pose(coord.x, coord.y, Math.toRadians(coord.heading));
    }

}