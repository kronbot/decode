package org.firstinspires.ftc.teamcode.kronbot.utils;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.*;

public class PoseStorage {

    private static final String FILE_NAME = "lastPose.txt";

    //save the pose
    public static void savePose(Pose pose) {
        File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);

        try (PrintWriter writer = new PrintWriter(new FileWriter(file))) {
            writer.println(pose.getX());
            writer.println(pose.getY());
            writer.println(pose.getHeading());
        } catch (IOException e) {
            RobotLog.ee("PoseStorage", "Failed to save pose", e);
        }
    }

    //load the pose
    public static Pose loadPose() {
        File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);

        if (!file.exists()) {
            return new Pose(0, 0, 0);
        }

        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            double x = Double.parseDouble(reader.readLine());
            double y = Double.parseDouble(reader.readLine());
            double heading = Double.parseDouble(reader.readLine());

            return new Pose(x, y, heading);
        } catch (Exception e) {
            RobotLog.ee("PoseStorage", "Failed to load pose", e);
            return new Pose(0, 0, 0);
        }
    }
}
