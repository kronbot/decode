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

import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;

@TeleOp(name = "End Game Blue", group = "Vision")
public class EndGameBlueOp extends LinearOpMode {

    private OpenCvCamera camera;
    private BlueSquareDetecion squarePipeline;
    private KronBot robot;

    // Tunable constants
    private final double SPIN_SPEED    = Constants.SpinEndGame;
    private final double ORBIT_STRAFE  = Constants.StrafeEndGame;
    private final double kP_DISTANCE   = Constants.pDistanceEndGame; // area values are large, keep this tiny
    private final double kP_ROTATION   = Constants.pRoataionEndGame;      // angleError is -1 to 1, needs stronger gain
    private final double TARGET_AREA   = Constants.squareAreaEndGame;  // tune between 20000-90000

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new KronBot();
        robot.initTeleop(hardwareMap);

        squarePipeline = new BlueSquareDetecion();
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.setPipeline(squarePipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
                FtcDashboard.getInstance().startCameraStream(camera, 30);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera error", errorCode);
                telemetry.update();
            }
        });

        waitForStart();
        boolean found = false;

        while (opModeIsActive()) {

            boolean squareVisible = squarePipeline.getDetectedQuadCount() > 0;

            if (!squareVisible && !found) {
                // === SEARCH MODE: Spin in place ===
                telemetry.addData("Status", "Searching — spinning...");
                telemetry.update();

                setMecanumPower(0, 0, SPIN_SPEED);

            }
            /*else if(squarePipeline.getDetectedAngle() == 0 || squarePipeline.getDetectedAngle() == 90 || squarePipeline.getDetectedAngle() == 180){
                setMecanumPower(0,0,0);
                break;
            }*/
            else if (squarePipeline.getDetectedQuadCount() > 0) {
                found = true;
                // Use angleError (-1 to 1) to keep square centered in frame
                double angleError    = squarePipeline.getAngleError();
                double rotationSpeed = angleError * kP_ROTATION;

                // Use area to maintain distance
                double detectedArea  = squarePipeline.getDetectedArea();
                double distanceError = TARGET_AREA - detectedArea;
                double forwardSpeed  = distanceError * kP_DISTANCE;
                forwardSpeed = Math.max(-0.3, Math.min(0.3, forwardSpeed)); // clamp

                telemetry.addData("Status", "Orbiting");
                telemetry.addData("Angle Error", angleError);
                telemetry.addData("Detected Area", detectedArea);
                telemetry.addData("Distance Error", distanceError);
                telemetry.addData("Forward Speed", forwardSpeed);
                telemetry.addData("Rotation Speed", rotationSpeed);
                telemetry.update();

                setMecanumPower(forwardSpeed, ORBIT_STRAFE, rotationSpeed);
            }
            else{
                setMecanumPower( -ORBIT_STRAFE , 0 , 0);
            }
        }

        setMecanumPower(0, 0, 0);
    }

    private void setMecanumPower(double forward, double strafe, double rotation) {
        double leftFrontPower  = forward + strafe + rotation;
        double rightFrontPower = forward - strafe - rotation;
        double leftRearPower   = forward - strafe + rotation;
        double rightRearPower  = forward + strafe - rotation;

        double maxPower = Math.max(1.0, Math.max(
                Math.abs(leftFrontPower),
                Math.max(Math.abs(rightFrontPower),
                        Math.max(Math.abs(leftRearPower), Math.abs(rightRearPower)))
        ));

        robot.motors.leftFront.setPower(leftFrontPower / maxPower);
        robot.motors.rightFront.setPower(rightFrontPower / maxPower);
        robot.motors.leftRear.setPower(leftRearPower / maxPower);
        robot.motors.rightRear.setPower(rightRearPower / maxPower);
    }
}