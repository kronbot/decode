package org.firstinspires.ftc.teamcode.kronbot.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.PoseStorage;

@Autonomous(name = "RED Load PoseStorage", group = Constants.TEST_GROUP)
public class LoadRedPoseStorageOp extends OpMode {
    private static final Pose RED_START_POSE = new Pose(9, 9, Math.PI / 2);

    @Override
    public void init() {
        PoseStorage.savePose(RED_START_POSE);
        showPose();
    }

    @Override
    public void init_loop() {
        showPose();
    }

    @Override
    public void loop() {
        showPose();
    }

    private void showPose() {
        telemetry.addLine("Saved RED pose to PoseStorage");
        telemetry.addData("X", "%.2f", RED_START_POSE.getX());
        telemetry.addData("Y", "%.2f", RED_START_POSE.getY());
        telemetry.addData("Heading", "%.4f rad", RED_START_POSE.getHeading());
        telemetry.update();
    }
}
