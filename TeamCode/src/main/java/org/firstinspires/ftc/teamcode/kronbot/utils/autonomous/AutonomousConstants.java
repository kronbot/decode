package org.firstinspires.ftc.teamcode.kronbot.utils.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;


@Config
public final class AutonomousConstants {

    public static final Pose FIELD_CENTER = new Pose(0, 0, 0);

    public static Pose StartingPose = new Pose(0, 5, Math.toRadians(0));
    public static Pose LaunchZone = new Pose(0, -5, Math.toRadians(0));

    public static Pose coordinates(Pose relativePose) {
        return new Pose(
                FIELD_CENTER.getX() + relativePose.getX(),
                FIELD_CENTER.getY() + relativePose.getY(),
                FIELD_CENTER.getHeading() + relativePose.getHeading()
        );
    }
}