package org.firstinspires.ftc.teamcode.kronbot.utils.detection;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class AprilTagWebcam {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private FtcDashboard dashboard;

    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    private Telemetry telemetry;

    public void init(HardwareMap hmMap, Telemetry telemetry){
        this.telemetry = telemetry;
        this.dashboard = FtcDashboard.getInstance();

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setLensIntrinsics(
                        764.4007594510844,   // fx
                        769.3247675231305,   // fy
                        329.1203921338121,   // cx
                        244.6501723615198    // cy
                )
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hmMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();


        if (dashboard != null) {
            dashboard.startCameraStream(visionPortal, 30);
        }
    }

    public void update(){
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags(){
        return detectedTags;
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedId){
        TelemetryPacket packet = new TelemetryPacket();

        if(detectedId==null){
            telemetry.addLine("No AprilTag detected with specified ID");
            packet.put("Status", "No tag detected");
            if (dashboard != null) {
                dashboard.sendTelemetryPacket(packet);
            }
            return;
        }

        if (detectedId.metadata != null) {
            telemetry.addLine("=== APRIL TAG DETECTION ===");
            telemetry.addLine(String.format(Locale.US, "DISTANCE: %.1f cm", detectedId.ftcPose.range));
            telemetry.addLine(String.format(Locale.US, "ID: %d - %s", detectedId.id, detectedId.metadata.name));
            telemetry.addLine(String.format(Locale.US, "Position XYZ (cm): %.1f, %.1f, %.1f",
                    detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
            telemetry.addLine(String.format(Locale.US, "Orientation PRY (deg): %.1f, %.1f, %.1f",
                    detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
            telemetry.addLine(String.format(Locale.US, "Spherical RBE: %.1f cm, %.1f°, %.1f°",
                    detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));

            packet.put("AprilTag ID", detectedId.id);
            packet.put("Tag Name", detectedId.metadata.name);
            packet.put("Distance (cm)", detectedId.ftcPose.range);
            packet.put("X (cm)", detectedId.ftcPose.x);
            packet.put("Y (cm)", detectedId.ftcPose.y);
            packet.put("Z (cm)", detectedId.ftcPose.z);
            packet.put("Pitch (deg)", detectedId.ftcPose.pitch);
            packet.put("Roll (deg)", detectedId.ftcPose.roll);
            packet.put("Yaw (deg)", detectedId.ftcPose.yaw);
            packet.put("Bearing (deg)", detectedId.ftcPose.bearing);
            packet.put("Elevation (deg)", detectedId.ftcPose.elevation);
        } else {
            telemetry.addLine("=== UNKNOWN APRIL TAG ===");
            telemetry.addLine(String.format(Locale.US, "ID: %d", detectedId.id));
            telemetry.addLine(String.format(Locale.US, "Center: %.0f, %.0f (pixels)",
                    detectedId.center.x, detectedId.center.y));

            packet.put("AprilTag ID", detectedId.id);
            packet.put("Center X (px)", detectedId.center.x);
            packet.put("Center Y (px)", detectedId.center.y);
        }

        if (dashboard != null) {
            dashboard.sendTelemetryPacket(packet);
        }
    }

    public AprilTagDetection getTagBySpecificId(int id){
        for(AprilTagDetection detection : detectedTags){
            if(detection.id == id){
                return detection;
            }
        }
        return null;
    }

    public AprilTagDetection getTowerTags() {
        for(AprilTagDetection detection : detectedTags) {
            if(detection.id == 20 || detection.id == 24)
            {
                return detection;
            }
        }
        return null;
    }

    public void stop(){
        if (dashboard != null) {
            dashboard.stopCameraStream();
        }
        if(visionPortal != null){
            visionPortal.close();
        }
    }

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }
}