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

    public static double launchSpeedClose = 1230;
    public static double launchSpeedBack = 1600;
    public static double angleServoBack = 0.3;
    public static double angleServoClose = 0.2;

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
    public static Coordinates StartingPoseBackRed = new Coordinates(0, -75, 0);
    public static Coordinates LaunchZoneBack = new Coordinates(7, -84, -0.13);
    public static Coordinates IntakeZoneBack1 = new Coordinates(8, -100, -1.5);
    public static Coordinates IntakeZoneBack11 = new Coordinates(5, -144, -1.5);
    public static Coordinates ParkBack = new Coordinates(15, -80, 0);



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
    public static Coordinates StartingPoseBackBlue = new Coordinates(5, -63, 0);
    public static Coordinates LaunchZoneBackBlue = new Coordinates(9, -59, 0.13);
    public static Coordinates IntakeZoneBack1Blue = new Coordinates(5, -40, 1.5);
    public static Coordinates IntakeZoneBack11Blue = new Coordinates(5, -20, 1.5);
    public static Coordinates ParkBackBlue = new Coordinates(30, -50, 0);

    public static Pose coordinates(Coordinates coord) {
        return new Pose(coord.x, coord.y, coord.heading);
    }

}