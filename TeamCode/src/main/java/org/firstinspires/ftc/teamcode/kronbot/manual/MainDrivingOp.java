package org.firstinspires.ftc.teamcode.kronbot.manual;



import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LOADER_SERVO_REVERSED;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.maxVelocity;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.minVelocity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.detection.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Button;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * The main TeleOP program for the driving period of the game.
 *
 * @version 1.0
 */
@TeleOp(name = "Main Driving", group = Constants.MAIN_GROUP)
public class MainDrivingOp extends LinearOpMode {
    private final KronBot robot = new KronBot();

    RobotCentricDrive robotCentricDrive;
    FieldCentricDrive fieldCentricDrive;

    Gamepad drivingGamepad;
    Gamepad utilityGamepad;
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initTeleop(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        telemetry.update();

        drivingGamepad = gamepad1;
        utilityGamepad = gamepad2;

        robotCentricDrive = new RobotCentricDrive(robot, drivingGamepad);
        fieldCentricDrive = new FieldCentricDrive(robot, drivingGamepad);

        aprilTagWebcam.init(hardwareMap, telemetry);
        if (aprilTagWebcam.getVisionPortal() != null) {
            dashboard.startCameraStream(aprilTagWebcam.getVisionPortal(), 30);
        }


        Button driveModeButton = new Button();
        Button reverseButton = new Button();

        boolean isLaunching = false;
        boolean wasRightBumperPressed = false;
        double currentVelocity = 1300;

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Initialization Ready");
            telemetry.update();
        }

        if (isStopRequested()) {
            aprilTagWebcam.stop();
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {
            //Outtake servo
            if(gamepad1.left_trigger > 0.5) {
                if(!LOADER_SERVO_REVERSED)
                    robot.loaderServo.setPosition(0);
                else
                    robot.loaderServo.setPosition(1);
            } else {
                double val = gamepad1.right_trigger;
                if(!LOADER_SERVO_REVERSED)
                    val = val / 2 + 0.5;
                else
                    val = 1 - (val / 2 + 0.5);

                robot.loaderServo.setPosition(val);
            }

            //Outake Wheels
            if (gamepad1.right_bumper && !wasRightBumperPressed) {
                isLaunching = !isLaunching;

                if (isLaunching) {
                    currentVelocity = minVelocity;
                }
            }

            if (isLaunching) {

                if (gamepad1.dpad_up) {
                    currentVelocity = maxVelocity;
                } else if (gamepad1.dpad_down) {
                    currentVelocity = minVelocity;
                }

                robot.leftOuttake.setVelocity(currentVelocity);
                robot.rightOuttake.setVelocity(currentVelocity);

            } else {
                robot.leftOuttake.setPower(0);
                robot.rightOuttake.setPower(0);
            }

            wasRightBumperPressed = gamepad1.right_bumper;

            // Wheels
            driveModeButton.updateButton(gamepad1.square);
            driveModeButton.longPress();

            reverseButton.updateButton(gamepad1.circle);
            reverseButton.shortPress();

            robotCentricDrive.setReverse(reverseButton.getShortToggle());
            if (!driveModeButton.getLongToggle()) {
                robotCentricDrive.run();
                robotCentricDrive.telemetry(telemetry);
            } else {
                fieldCentricDrive.run();
                fieldCentricDrive.telemetry(telemetry);
            }

            aprilTagWebcam.update();
            AprilTagDetection tag24 = aprilTagWebcam.getTagBySpecificId(24);

            if (tag24 != null) {
                telemetry.addLine("=== TAG 24 DETECTED ===");
                telemetry.addData("Distance (cm)", tag24.ftcPose.range);
                telemetry.addData("X (cm)", tag24.ftcPose.x);
                telemetry.addData("Y (cm)", tag24.ftcPose.y);
                telemetry.addData("Z (cm)", tag24.ftcPose.z);
                aprilTagWebcam.displayDetectionTelemetry(tag24);
            } else {
                telemetry.addLine("=== TAG 24 NOT FOUND ===");
            }

            telemetry.addLine("");
            telemetry.addData("Left Shooter Vel", robot.leftOuttake.getVelocity());
            telemetry.addData("Right Shooter Vel", robot.rightOuttake.getVelocity());

            double leftVel = robot.leftOuttake.getVelocity();
            telemetry.addData("leftVel is: ", leftVel);
            double rightVel = robot.rightOuttake.getVelocity();
            telemetry.addData("rightVel is: ", rightVel);

            telemetry.update();
        }
    }
}