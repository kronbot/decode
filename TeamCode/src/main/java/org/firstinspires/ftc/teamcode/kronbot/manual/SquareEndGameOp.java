package org.firstinspires.ftc.teamcode.kronbot.manual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.kronbot.utils.detection.SquareEndGame;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Square Detection TeleOp", group = "Vision")
public class SquareEndGameOp extends LinearOpMode {

    private OpenCvCamera webcam;
    private SquareEndGame squarePipeline;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the pipeline
        squarePipeline = new SquareEndGame();

        // Setup webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(squarePipeline);

        // Open camera asynchronously
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera error", errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // Get detected quadrilateral info
            int quadCount = squarePipeline.getDetectedQuadCount();
            double angle = squarePipeline.getDetectedAngle();

            telemetry.addData("Quadrilaterals Detected", quadCount);
            telemetry.addData("Detected Angle", angle);
            telemetry.update();

            sleep(50);
        }

        // Stop streaming when OpMode ends
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }
}