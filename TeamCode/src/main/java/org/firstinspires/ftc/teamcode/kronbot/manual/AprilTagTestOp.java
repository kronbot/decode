package org.firstinspires.ftc.teamcode.kronbot.manual;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.kronbot.utils.detection.legacy.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@TeleOp(name="AprilTag Test (TeleOp)", group="Concept")
public class AprilTagTestOp extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    @Override
    public void runOpMode()
    {
        // Combine DS telemetry + Dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), // must match Robot Config name
                cameraMonitorViewId);

        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(telemetry);
        camera.setPipeline(aprilTagDetectionPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

                // Stream camera to Dashboard (30 fps)
                FtcDashboard.getInstance().startCameraStream(camera, 30);
            }

            @Override
            public void onError(int errorCode) {}
        });

        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        while (opModeIsActive())
        {
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getLatestDetections();

            if(detections.size() != 0)
            {
                for(AprilTagDetection detection : detections)
                {
                    telemetry.addLine(String.format("Detected tag ID=%d", detection.id));
                    telemetry.addLine(String.format("X: %.2f m", detection.pose.x));
                    telemetry.addLine(String.format("Y: %.2f m", detection.pose.y));
                    telemetry.addLine(String.format("Z: %.2f m", detection.pose.z));

                    telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.z)));
                    telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.x)));
                    telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.y)));
                }
            }
            else
            {
                telemetry.addLine("No tags detected");
            }

            telemetry.update();
            sleep(20);
        }
    }
}
