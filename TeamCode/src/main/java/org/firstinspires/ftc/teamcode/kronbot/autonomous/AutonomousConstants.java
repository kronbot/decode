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
    public static double launchSpeedBack = 1550;
    public static double angleServoBack = 0.6;
    public static double angleServoClose = 0.65;
    //how much reflecting light is hitting the sensor? - to be tuned
    //works regardless of color, just for the detection of the object in front of the sensor
    public static final int BALL_EXIT_THRESHOLD = 150;

    /// RED auto movement
    public static Coordinates StartingPoseClose = new Coordinates(0, 0, 0);
    public static Coordinates LaunchZoneClose = new Coordinates(-25, 32, -0.8);
    public static Coordinates LaunchZoneClose2 = new Coordinates(-19, 55, -0.84);
    public static Coordinates ParkClose = new Coordinates(-15, 0, 0);
    public static Coordinates StartingPoseBack = new Coordinates(0, 0, 0);
    public static Coordinates LaunchZoneBack = new Coordinates(20, -2.3, -0.4);
    public static Coordinates ParkBack = new Coordinates(40, 0, 0);

    /// BLUE auto movement
    public static Coordinates LaunchZoneCloseBlue = new Coordinates(16, -32, 0.7);
    public static Coordinates LaunchZoneClose2Blue = new Coordinates(-25, -58, 0.8);
    public static Coordinates ParkCloseBlue = new Coordinates(-5, 0, 0);
    public static Coordinates LaunchZoneBackBlue = new Coordinates(18, 2.3, 0.4);
    public static Coordinates ParkBackBlue = new Coordinates(40, 0, 0);


    public static Pose coordinates(Coordinates coord) {
        return new Pose(coord.x, coord.y, coord.heading);
    }

}