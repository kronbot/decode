package org.firstinspires.ftc.teamcode.kronbot.manual;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
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
                    telemetry.addLine(String.format("X: %.2f ft", detection.pose.x*FEET_PER_METER));
                    telemetry.addLine(String.format("Y: %.2f ft", detection.pose.y*FEET_PER_METER));
                    telemetry.addLine(String.format("Z: %.2f ft", detection.pose.z*FEET_PER_METER));

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
