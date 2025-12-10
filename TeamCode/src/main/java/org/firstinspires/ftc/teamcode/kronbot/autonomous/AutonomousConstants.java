package org.firstinspires.ftc.teamcode.kronbot.autonomous;

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

    public static Coordinates StartingPoseClose = new Coordinates(0, 0, 0);
    public static Coordinates LaunchZoneClose = new Coordinates(16, 32, -0.8);
    public static Coordinates LaunchZoneClose2 = new Coordinates(-5.22, 55.15, -0.84);
    public static Coordinates ParkClose = new Coordinates(-5.22, 55.15, -0.84);
    public static double launchSpeedClose = 2000;
    public static Coordinates StartingPoseBack = new Coordinates(0, 0, 0);
    public static Coordinates LaunchZoneBack = new Coordinates(30, 33, -0.7);
    public static Coordinates ParkBack = new Coordinates(30, 33, -0.7);
    public static double launchSpeedBack = 2130;



    public static Pose coordinates(Coordinates coord) {
        return new Pose(coord.x, coord.y, coord.heading);
    }

}