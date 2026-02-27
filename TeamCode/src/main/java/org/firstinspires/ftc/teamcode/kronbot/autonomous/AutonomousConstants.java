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
    public static double angleServoClose = 0.32;

    /// RED CLOSE auto movement
    public static Coordinates StartingPoseCloseRed = new Coordinates(130, 110, 0);
    public static Coordinates LaunchZoneClose1 = new Coordinates(125, 140, 0);
    public static Coordinates LaunchZoneClose11 = new Coordinates(120, 144, -0.8);
    public static Coordinates IntakeZoneClose1 = new Coordinates(105, 140, -1.5);
    public static Coordinates IntakeZoneClose11 = new Coordinates(105, 110, -1.5);
    public static Coordinates LaunchZoneClose2 = new Coordinates(125, 144, -0.8);
    public static Coordinates IntakeZoneClose2 = new Coordinates(76, 140, -1.5);
    public static Coordinates IntakeZoneClose22 = new Coordinates(76, 110, -1.5);

    public static Coordinates LaunchZoneClose3 = new Coordinates(125, 144, -0.8);
    public static Coordinates ParkClose = new Coordinates(110, 110, 0);

    /// RED FAR auto movement
    public static Coordinates StartingPoseBackRed = new Coordinates(80, 8.7, 1.57);
    public static Coordinates LaunchZoneBack = new Coordinates(85, 17, 1.17);
    public static Coordinates ParkBack = new Coordinates(84, 34.5, 1.57);

    /// BLUE CLOSE auto movement
    public static Coordinates StartingPoseCloseBlue = new Coordinates(14, 110, 0);
    public static Coordinates LaunchZoneClose1Blue = new Coordinates(19, 140, 0);
    public static Coordinates LaunchZoneClose11Blue = new Coordinates(25, 144, 0.8);
    public static Coordinates IntakeZoneClose1Blue = new Coordinates(40, 140, 1.5);
    public static Coordinates IntakeZoneClose11Blue = new Coordinates(40, 110, 1.5);
    public static Coordinates LaunchZoneClose2Blue = new Coordinates(30, 144, 0.8);
    public static Coordinates IntakeZoneClose2Blue = new Coordinates(10, 140, 1.5);
    public static Coordinates IntakeZoneClose22Blue = new Coordinates(10, 110, 1.5);

    public static Coordinates LaunchZoneClose3Blue = new Coordinates(30, 144, 0.8);
    public static Coordinates ParkCloseBlue = new Coordinates(-6, 110, 0);

    /// BLUE FAR auto movement
    public static Coordinates StartingPoseBackBlue = new Coordinates(63, 8, 1.57);
    public static Coordinates LaunchZoneBackBlue = new Coordinates(60, 17, 1.97);
    public static Coordinates ParkBackBlue = new Coordinates(61, 32, 1.57);

    public static Pose coordinates(Coordinates coord) {
        return new Pose(coord.x, coord.y, coord.heading);
    }

}