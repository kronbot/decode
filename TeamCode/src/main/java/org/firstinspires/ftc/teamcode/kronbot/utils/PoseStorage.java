package org.firstinspires.ftc.teamcode.kronbot.utils;

import android.os.Environment;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.RobotLog;

import java.io.*;

public class PoseStorage {

    private static final String FILE_NAME = "lastPose.txt";

    // SAVE pose to external storage
    public static void savePose(Pose pose) {

        String path = Environment.getExternalStorageDirectory().getPath()
                + "/" + FILE_NAME;

        try (PrintWriter writer = new PrintWriter(new FileWriter(path))) {

            writer.println(pose.getX());
            writer.println(pose.getY());
            writer.println(pose.getHeading());

            RobotLog.ii("PoseStorage", "Pose saved to " + path);

        } catch (IOException e) {
            RobotLog.ee("PoseStorage", "Failed to save pose", e);
        }
    }

    // LOAD pose from external storage
    public static Pose loadPose() {

        String path = Environment.getExternalStorageDirectory().getPath()
                + "/" + FILE_NAME;

        File file = new File(path);

        if (!file.exists()) {
            RobotLog.ww("PoseStorage", "Pose file not found");
            return new Pose(0, 0, 0);
        }

        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {

            double x = Double.parseDouble(reader.readLine());
            double y = Double.parseDouble(reader.readLine());
            double heading = Double.parseDouble(reader.readLine());

            RobotLog.ii("PoseStorage", "Pose loaded from " + path);

            return new Pose(x, y, heading);

        } catch (Exception e) {
            RobotLog.ee("PoseStorage", "Failed to load pose", e);
            return new Pose(0, 0, 0);
        }
    }
}