package org.firstinspires.ftc.teamcode.kronbot.manual;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.detection.BlueSquareDetecion;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "End Game Blue" , group = "Vision")
public class EndGameBlueOp extends LinearOpMode {

    private OpenCvCamera camera;
    private BlueSquareDetecion squarePipeline;
    private  KronBot robot;
    private double kD = 0.01;
    private double kP = 0.2;
    private double strafeSpeed = 0.3;
    private double targetAngle = 90.0;
    private double targetArea = 0.0;
    private double angleTolerance = 5.0;
    private double strafeDirection = 1.0; // 1 for right, -1 for left
    private double strafe = 0.0;
    private double distanceError = 0.0;
    private double forwardSpeed = 0.0;
    private double rotationSpeed = 0.0;
    private double angleError = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new KronBot();
        robot.initTeleop(hardwareMap);
        // Initialize the pipeline
        squarePipeline = new BlueSquareDetecion();
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.setPipeline(squarePipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 30);
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera error", errorCode);
                telemetry.update();
            }
        });
        boolean found = false;
        waitForStart();
        while(opModeIsActive()) {

            if (squarePipeline.getDetectedQuadCount() == 0 && !found) {
                // search for square
                targetArea = squarePipeline.getDetectedArea();
                telemetry.addData("Status", "No quadrilaterals detected");
                telemetry.update();
                // example search motion
                robot.motors.leftFront.setPower(-0.3);
                robot.motors.rightFront.setPower(-0.3);
                robot.motors.leftRear.setPower(0.3);
                robot.motors.rightRear.setPower(-0.3);
            }
            else {
                telemetry.addData("Area", squarePipeline.getDetectedArea());
                telemetry.update();
                found = true;
                // decide strafe direction
                if (squarePipeline.getDetectedAngle() < targetAngle - angleTolerance) {
                    strafeDirection = 1;  // strafe right
                } else if (squarePipeline.getDetectedAngle() > targetAngle + angleTolerance) {
                    strafeDirection = -1; // strafe left
                } else {
                    strafeDirection = 0;  // stop orbit
                }

                // compute motion
                strafe = strafeDirection * strafeSpeed;
                distanceError = targetArea - squarePipeline.getDetectedArea();
                forwardSpeed = distanceError * kD;
                angleError = squarePipeline.getAngleError();
                rotationSpeed = angleError * kP;

                // mecanum wheel powers
                double lf = forwardSpeed + strafe + rotationSpeed;
                double rf = forwardSpeed - strafe - rotationSpeed;
                double lr = forwardSpeed - strafe + rotationSpeed;
                double rr = forwardSpeed + strafe - rotationSpeed;

                // normalize
                double max = Math.max(Math.abs(lf), Math.max(Math.abs(rf), Math.max(Math.abs(lr), Math.abs(rr))));
                if (max > 1.0) {
                    lf /= max; rf /= max; lr /= max; rr /= max;
                }

                // set motors
                robot.motors.leftFront.setPower(lf);
                robot.motors.rightFront.setPower(rf);
                robot.motors.leftRear.setPower(lr);
                robot.motors.rightRear.setPower(rr);
            }
        }
    }
}

