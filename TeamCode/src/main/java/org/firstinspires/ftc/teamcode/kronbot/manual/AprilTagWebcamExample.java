package org.firstinspires.ftc.teamcode.kronbot.manual;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.detection.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

// Add the necessary FTC Dashboard import
import com.acmerobotics.dashboard.FtcDashboard;

@TeleOp(name = "AprilTagWebcamExample", group = Constants.MAIN_GROUP)
public class AprilTagWebcamExample extends OpMode {

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private FtcDashboard dashboard;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();



        aprilTagWebcam.init(hardwareMap, telemetry);
        telemetry.addData("Status", "AprilTag Webcam Initialized");


        if (aprilTagWebcam.getVisionPortal() != null) {
            dashboard.startCameraStream(aprilTagWebcam.getVisionPortal(), 30); // 30 FPS
        }

        telemetry.update();
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();


        telemetry.clear();

        AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(24);

        if (id24 != null) {
            telemetry.addLine("=== ACTIVE DETECTION ===");
            telemetry.addLine(String.format("DISTANCE TO TAG %d: %.1f cm", id24.id, id24.ftcPose.range));
            telemetry.addLine("");
        } else {
            telemetry.addLine("=== NO DETECTION ===");
            telemetry.addLine("Tag ID 24 not found");
            telemetry.addLine("");
        }


        aprilTagWebcam.displayDetectionTelemetry(id24);

        telemetry.addLine("");
        telemetry.addLine("=== GENERAL INFO ===");
        telemetry.addLine("Looking for AprilTag ID: 24");
        telemetry.addLine("Press stop to end detection");

        telemetry.update();
    }

    @Override
    public void stop() {
        aprilTagWebcam.stop();
        telemetry.addData("Status", "AprilTag Webcam Stopped");
        telemetry.update();
    }
}