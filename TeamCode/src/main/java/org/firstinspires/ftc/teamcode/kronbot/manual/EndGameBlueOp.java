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

@TeleOp(name = "End Game Blue", group = "Vision")
public class EndGameBlueOp extends LinearOpMode {

    private OpenCvCamera camera;
    private BlueSquareDetecion squarePipeline;
    private KronBot robot;

    // Tunable constants
    private final double SPIN_SPEED    = 0.3;  // Speed to spin while searching
    private final double ORBIT_STRAFE  = 0.3;  // Lateral orbit speed
    private final double kP_DISTANCE   = 0.01; // How aggressively to correct distance
    private final double kP_ROTATION   = 0.02; // How aggressively to face the square
    private final double TARGET_AREA   = 5000.0; // Tune: desired apparent size of square
    private final double TARGET_ANGLE  = 0.0;    // Tune: desired angle offset from center (0 = centered)
    private final double ANGLE_TOLERANCE = 5.0;

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
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
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

            boolean squareVisible = squarePipeline.getDetectedQuadCount() > 0;

            if (!squareVisible) {
                // === SEARCH MODE: Spin in place ===
                telemetry.addData("Status", "Searching — spinning...");
                telemetry.update();

                setMecanumPower(0, 0, SPIN_SPEED);

            } else {
                double detectedAngle = squarePipeline.getDetectedAngle();
                double detectedArea  = squarePipeline.getDetectedArea();

                // Check if we've reached either target angle
                boolean at90  = Math.abs(detectedAngle - 90.0)  <= ANGLE_TOLERANCE;
                boolean at180 = Math.abs(detectedAngle - 180.0) <= ANGLE_TOLERANCE;

                if (at90 || at180) {
                    // === DONE: Stop the robot ===
                    telemetry.addData("Status", "Target angle reached: " + detectedAngle);
                    telemetry.update();

                    setMecanumPower(0, 0, 0);

                } else {
                    // === ORBIT MODE: Circle around the square ===
                    double angleError    = 0 - detectedAngle; // Keep square centered in frame
                    double rotationSpeed = angleError * kP_ROTATION;

                    double distanceError = TARGET_AREA - detectedArea;
                    double forwardSpeed  = distanceError * kP_DISTANCE;

                    double strafe = ORBIT_STRAFE;

                    telemetry.addData("Status", "Orbiting");
                    telemetry.addData("Detected Angle", detectedAngle);
                    telemetry.addData("Detected Area", detectedArea);
                    telemetry.update();

                    setMecanumPower(forwardSpeed, strafe, rotationSpeed);
                }
            }
        }

        setMecanumPower(0, 0, 0);
    }

    /**
     * Sets mecanum wheel powers using standard mecanum drive math.
     *
     * @param forward  Positive = forward, negative = backward
     * @param strafe   Positive = right, negative = left
     * @param rotation Positive = clockwise, negative = counter-clockwise
     */
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