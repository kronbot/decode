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
    public static double angleServoClose = 0.45;

    /// RED CLOSE auto movement
    public static Coordinates StartingPoseCloseRed = new Coordinates(130, 110, 0);
    public static Coordinates LaunchZoneClose1 = new Coordinates(130, 140, 0);
    public static Coordinates LaunchZoneClose11 = new Coordinates(120, 144, -0.8);
    public static Coordinates IntakeZoneClose1 = new Coordinates(105, 140, -1.5);
    public static Coordinates IntakeZoneClose11 = new Coordinates(105, 110, -1.5);
    public static Coordinates LaunchZoneClose2 = new Coordinates(120, 144, -0.8);
    public static Coordinates IntakeZoneClose2 = new Coordinates(80, 140, -1.5);
    public static Coordinates IntakeZoneClose22 = new Coordinates(80, 100, -1.5);

    public static Coordinates LaunchZoneClose3 = new Coordinates(120, 144, -0.8);
    public static Coordinates ParkClose = new Coordinates(128, 104, 0);

    /// RED FAR auto movement
    public static Coordinates StartingPoseBackRed = new Coordinates(79, 7.4, 0);
    public static Coordinates LaunchZoneBack = new Coordinates(83, 19, -0.4);
    public static Coordinates ParkBack = new Coordinates(81, 35, 0);

    /// BLUE auto movement
    public static Coordinates StartingPoseCloseBlue = new Coordinates(0, 0, 0);
    public static Coordinates LaunchZoneCloseBlue = new Coordinates(16, -32, 0.7);
    public static Coordinates LaunchZoneClose2Blue = new Coordinates(-25, -58, 0.8);
    public static Coordinates ParkCloseBlue = new Coordinates(-5, 0, 0);
    public static Coordinates StartingPoseBackBlue = new Coordinates(0, 0, 0);
    public static Coordinates LaunchZoneBackBlue = new Coordinates(11, 2.3, 0.4);
    public static Coordinates ParkBackBlue = new Coordinates(30, 0, 0);

    public static Pose coordinates(Coordinates coord) {
        return new Pose(coord.x, coord.y, coord.heading);
    }

}