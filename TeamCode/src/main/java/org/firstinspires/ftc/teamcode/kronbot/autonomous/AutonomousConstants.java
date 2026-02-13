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

    public static double launchSpeedClose = 1150;
    public static double launchSpeedBack = 1400;
    public static double angleServoBack = 0.6;
    public static double angleServoClose = 0.65;

    /// RED auto movement
    public static Coordinates StartingPoseCloseRed = new Coordinates(0, 0, 0);
    public static Coordinates LaunchZoneClose = new Coordinates(-25, 32, -0.8);
    public static Coordinates LaunchZoneClose2 = new Coordinates(-19, 55, -0.84);
    public static Coordinates ParkClose = new Coordinates(-15, 0, 0);
    public static Coordinates StartingPoseBackRed = new Coordinates(0, 0, 0);
    public static Coordinates LaunchZoneBack = new Coordinates(13, -2.3, -0.4);
    public static Coordinates ParkBack = new Coordinates(30, 0, 0);

    /// BLUE auto movement
    public static Coordinates StartingPoseCloseBlue = new Coordinates(0, 0, 0);
    public static Coordinates LaunchZoneCloseBlue = new Coordinates(16, -32, 0.7);
    public static Coordinates LaunchZoneClose2Blue = new Coordinates(-25, -58, 0.8);
    public static Coordinates ParkCloseBlue = new Coordinates(-5, 0, 0);
    public static Coordinates StartingPoseBackBlue = new Coordinates(0, 0, 0);
    public static Coordinates LaunchZoneBackBlue = new Coordinates(11, 2.3, 0.5);
    public static Coordinates ParkBackBlue = new Coordinates(23, 0, 0);

    public static Pose coordinates(Coordinates coord) {
        return new Pose(coord.x, coord.y, coord.heading);
    }

}