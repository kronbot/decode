package org.firstinspires.ftc.teamcode.kronbot.manual;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.maxVelocity;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.minVelocity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kronbot.utils.components.SimpleLinearRegression;
import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.detection.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "Webcam Driving", group = Constants.MAIN_GROUP)
public class WebcamDrivingOp extends LinearOpMode {

    private final KronBot robot = new KronBot();
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initTeleop(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        telemetry.addLine("Initialized");
        telemetry.update();

        //  SHOOTER LINEAR REGRESSION MODEL
        SimpleLinearRegression regression = new SimpleLinearRegression();

        //replaced with actual measured calibrations
        //distance meters, velocity
        regression.addData(211, 1900);
        regression.addData(239, 2000);
        regression.addData(241, 2050);
        regression.addData(250, 2100);
        regression.addData(351, 2200);
        regression.addData(379, 2300);

        boolean isLaunching = false;
        boolean wasRightBumperPressed = false;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            // Detect AprilTag distance
            aprilTagWebcam.update();
            double distance = -1;
            List<AprilTagDetection> detections = aprilTagWebcam.getDetectedTags();

            if (detections != null && detections.size() > 0) {
                AprilTagDetection tag = detections.get(0);

                double x = tag.ftcPose.x;
                double y = tag.ftcPose.y;

                distance = Math.sqrt(x * x + y * y);

                telemetry.addData("Tag Distance", "%.2f m", distance);
            } else {
                telemetry.addLine("No Tag Detected");
            }


            if (gamepad1.right_bumper && !wasRightBumperPressed) {
                isLaunching = !isLaunching;
            }

            if (isLaunching) {

                double targetVelocity;

                if (distance > 0) {
                    // Use the regression model to predict shooter speed
                    targetVelocity = regression.predict(distance);
                } else {
                    // No tag detected → fallback
                    targetVelocity = minVelocity;
                }

                // Clamp velocity
                targetVelocity = Math.max(minVelocity, Math.min(maxVelocity, targetVelocity));

                robot.leftOuttake.setVelocity(targetVelocity);
                robot.rightOuttake.setVelocity(targetVelocity);

                telemetry.addData("Shooter Mode", "AUTO");
                telemetry.addData("Predicted Velocity", targetVelocity);

            } else {
                robot.leftOuttake.setPower(0);
                robot.rightOuttake.setPower(0);
                telemetry.addData("Shooter Mode", "OFF");
            }

            wasRightBumperPressed = gamepad1.right_bumper;

            telemetry.addData("Left Vel", robot.leftOuttake.getVelocity());
            telemetry.addData("Right Vel", robot.rightOuttake.getVelocity());

            if (aprilTagWebcam.getVisionPortal() != null) {
                dashboard.startCameraStream(aprilTagWebcam.getVisionPortal(), 30);
            }

            telemetry.update();
        }
    }
}
