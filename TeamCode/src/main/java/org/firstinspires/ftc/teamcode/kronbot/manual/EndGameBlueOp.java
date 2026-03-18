package org.firstinspires.ftc.teamcode.kronbot.manual;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    private final double SPIN_SPEED        = Constants.SpinEndGame;
    private final double ORBIT_STRAFE      = Constants.StrafeEndGame;
    private final double kP_DISTANCE       = Constants.pDistanceEndGame;
    private final double kP_ROTATION       = Constants.pRoataionEndGame;
    private final double TARGET_AREA       = Constants.squareAreaEndGame;
    private final double BACK_SPIN_SPEED   = Constants.BACK_SPIN_SPEED;
    private final double BACK_SPEED        = Constants.BACK_SPEED;
    private final double ALGIN_STRAFE_SPEED = Constants.ALGIN_STRAFE_SPEED;
    private final double pALIGN = Constants.pALIGN;
    private final double ALIGN_ROATION_SPEED = Constants.ALIGN_ROATION_SPEED;
    private final double pALIGN_ROTATION = Constants.pALIGN_ROTATION;
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
        ElapsedTime CorrectionTimer = new ElapsedTime();

        while (opModeIsActive()) {
            boolean squareVisible = squarePipeline.getDetectedQuadCount() > 0;
            boolean correcting = false;
            double angle = squarePipeline.getDetectedAngle();

            // --- Shared telemetry shown in every state ---
            telemetry.addData("Square Visible",   squareVisible);
            telemetry.addData("Quad Count",        squarePipeline.getDetectedQuadCount());
            telemetry.addData("Detected Angle",    "%.2f°", angle);
            telemetry.addData("Detected Area",     "%.0f", squarePipeline.getDetectedArea());
            telemetry.addData("Angle Error",       "%.3f", squarePipeline.getAngleError());
            telemetry.addData("Found Flag",        found);
            telemetry.addLine("----------------------------");

            if (!squareVisible && !found) {
                // === SEARCH MODE ===
                telemetry.addData("Status", "SEARCHING — spinning...");
                telemetry.addData("Spin Speed", SPIN_SPEED);
                telemetry.update();

                setMecanumPower(0, 0, SPIN_SPEED);

            } else if ((angle > 89 && angle < 91) || (angle > 179 || angle < 2) && squareVisible) {
                // === ALIGNED / DONE ===
                telemetry.addData("Status", "ALIGNED — stopping loop");
                telemetry.addData("Final Angle", "%.2f°", angle);
                telemetry.update();

                setMecanumPower(0, 0, 0);
                break;

            } else if (squarePipeline.getDetectedQuadCount() > 0) {
                // === ORBIT MODE ===
                found = true;
                double angleError    = squarePipeline.getAngleError();
                double rotationSpeed = angleError * kP_ROTATION;

                double detectedArea  = squarePipeline.getDetectedArea();
                double distanceError = TARGET_AREA - detectedArea;
                double forwardSpeed  = distanceError * kP_DISTANCE;
                forwardSpeed = Math.max(-0.3, Math.min(0.3, forwardSpeed));

                telemetry.addData("Status",         "ORBITING");
                telemetry.addData("Target Area",    "%.0f", TARGET_AREA);
                telemetry.addData("Distance Error", "%.0f", distanceError);
                telemetry.addData("Forward Speed",  "%.3f", forwardSpeed);
                telemetry.addData("Rotation Speed", "%.3f", rotationSpeed);
                telemetry.addData("Orbit Strafe",   ORBIT_STRAFE);
                telemetry.update();

                setMecanumPower(forwardSpeed, ORBIT_STRAFE, rotationSpeed);

            } else if (!squareVisible && found) {
                // === CORRECTION / RECOVERY MODE ===
                if (!correcting) {
                    CorrectionTimer = new ElapsedTime();
                    correcting = true;
                    setMecanumPower(0, 0, 0);
                } else if (CorrectionTimer.seconds() > 1 && correcting) {
                    found = false;
                    correcting = false;
                }

                telemetry.addData("Status",            "RECOVERING — backing up");
                telemetry.addData("Correcting",        correcting);
                telemetry.addData("Correction Timer",  "%.2fs", CorrectionTimer.seconds());
                telemetry.addData("Back Speed",        -BACK_SPEED);
                telemetry.addData("Back Spin Speed",   -BACK_SPIN_SPEED);
                telemetry.update();

                setMecanumPower(-BACK_SPEED, 0, -BACK_SPIN_SPEED);
            }
        }

        // === FINAL STRAFE ALIGNMENT ===
        telemetry.addData("Status", "FINAL ALIGNMENT — strafing");
        telemetry.update();

        while (opModeIsActive()) {
            double angle        = Math.sin(Math.toRadians(2 * squarePipeline.getDetectedAngle()));
            double angleError   = squarePipeline.getAngleError();
            double rotationSpeed = pALIGN_ROTATION * ALIGN_ROATION_SPEED * angleError;
            double strafeSpeed   = pALIGN * ALGIN_STRAFE_SPEED * angleError;
            /*if(angleError < 0.2 && (angle > 179.3 || angle < 0.5))
                break;*/
            setMecanumPower(0, strafeSpeed, rotationSpeed);

            telemetry.addData("Status",         "FINAL ALIGNMENT");
            telemetry.addData("Square Angle",   "%.2f°", squarePipeline.getDetectedAngle());
            telemetry.addData("sin(2θ)",        "%.3f", angle);
            telemetry.addData("Angle Error",    "%.3f", angleError);
            telemetry.addData("Strafe Speed",   "%.3f", strafeSpeed);
            telemetry.addData("Rotation Speed", "%.3f", rotationSpeed);
            telemetry.update();
        }

        setMecanumPower(0, 0, 0);
        telemetry.addData("Status", "COMPLETE — all motors stopped");
        telemetry.update();
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

        double lf = leftFrontPower  / maxPower;
        double rf = rightFrontPower / maxPower;
        double lr = leftRearPower   / maxPower;
        double rr = rightRearPower  / maxPower;

        robot.motors.leftFront.setPower(lf);
        robot.motors.rightFront.setPower(rf);
        robot.motors.leftRear.setPower(lr);
        robot.motors.rightRear.setPower(rr);

        // Motor power telemetry
        telemetry.addLine("--- Motor Powers ---");
        telemetry.addData("Left  Front", "%.3f", lf);
        telemetry.addData("Right Front", "%.3f", rf);
        telemetry.addData("Left  Rear",  "%.3f", lr);
        telemetry.addData("Right Rear",  "%.3f", rr);
    }
}