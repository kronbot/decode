package org.firstinspires.ftc.teamcode.kronbot.manual;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.Robot;
import org.firstinspires.ftc.teamcode.kronbot.utils.detection.SquareEndGame;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@TeleOp(name = "End Game Blue" , group = "Vision")
public class EndGameBlueOp extends LinearOpMode {

    private OpenCvCamera camera;
    private SquareEndGame squarePipeline;
    private  KronBot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new KronBot();
        // Initialize the pipeline
        squarePipeline = new SquareEndGame();
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.setPipeline(squarePipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(604, 480, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 30);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera error", errorCode);
                telemetry.update();
            }
        });
        waitForStart();
        while (opModeIsActive()) {
            while (squarePipeline.getDetectedQuadCount() == 0) {
                telemetry.addData("Status", "No quadrilaterals detected");
                telemetry.update();

                robot.motors.leftFront.setPower(0.5);
                robot.motors.rightFront.setPower(-0.5);
                robot.motors.leftRear.setPower(0.5);
                robot.motors.rightRear.setPower(-0.5);
            }
            robot.motors.leftFront.setPower(0);
            robot.motors.rightFront.setPower(0);
            robot.motors.leftRear.setPower(0);
            robot.motors.rightRear.setPower(0);
        }

    }
}
