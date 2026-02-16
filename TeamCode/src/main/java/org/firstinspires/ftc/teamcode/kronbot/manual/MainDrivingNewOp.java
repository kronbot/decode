package org.firstinspires.ftc.teamcode.kronbot.manual;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LOADER_SERVO_REVERSED;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.maxVelocity;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.minVelocity;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.AutoAim;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.detection.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Button;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


/**
 * The main TeleOP program for the driving period of the game.
 * @version 1.0
 */
@TeleOp(name = "New Main Driving", group = Constants.MAIN_GROUP)
public class MainDrivingNewOp extends LinearOpMode {
    private final KronBot robot = new KronBot();
    private RobotCentricDrive robotCentricDrive;
    private FieldCentricDrive fieldCentricDrive;

    private Gamepad drivingGamepad;
    private AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private AprilTagDetection tag24, tag20, tag;
    private FtcDashboard dashboard;

    //Control Variables
    private boolean isLaunching = false,  autoAimEnabled = false;
    private double currentVelocity = 1300, rightVel, leftVel;
    private Timer buttonRebounceTimer;

    AutoAim autoAim;

    @Override
    public void runOpMode() throws InterruptedException {

        // INIT objects
        robot.initHardware(hardwareMap);
        dashboard = FtcDashboard.getInstance();

        drivingGamepad = gamepad1;

        robotCentricDrive = new RobotCentricDrive(robot, drivingGamepad);
        fieldCentricDrive = new FieldCentricDrive(robot, drivingGamepad);

        Button driveModeButton = new Button();
        Button reverseButton = new Button();
        Button aimButton = new Button();
        Button aimButtonStop = new Button();
        buttonRebounceTimer = new Timer();

        autoAim = new AutoAim(robot);

        //INIT telemetry and webcam
        telemetry.update();
        aprilTagWebcam.init(hardwareMap, telemetry);
        if (aprilTagWebcam.getVisionPortal() != null) {
            dashboard.startCameraStream(aprilTagWebcam.getVisionPortal(), 30);
        }

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Initialization Ready");
            telemetry.update();
            return;
        }

            //main loop
            while (opModeIsActive() && !isStopRequested()) {

                aimButton.updateButton(gamepad1.dpad_right);
                aimButton.shortPress();

                aimButtonStop.updateButton(gamepad1.dpad_left);
                aimButtonStop.shortPress();

                if (aimButton.getShortToggle() && !aimButtonStop.getShortToggle()) {
                    autoAimEnabled = true;
                    autoAim.reset();
                    aimButton.resetToggles();
                }
                else if (aimButtonStop.getShortToggle()) {
                    autoAimEnabled = false;
                    autoAim.reset();
                    aimButtonStop.resetToggles();
                }

                outake();

                // Driving
                driveModeButton.updateButton(gamepad1.square);
                driveModeButton.longPress();

                reverseButton.updateButton(gamepad1.circle);
                reverseButton.shortPress();

                robotCentricDrive.setReverse(reverseButton.getShortToggle());
                if(!autoAimEnabled || tag==null) {
                    if (!driveModeButton.getLongToggle()) {
                        robotCentricDrive.run();
                        robotCentricDrive.telemetry(telemetry);
                    } else {
                        fieldCentricDrive.run();
                        fieldCentricDrive.telemetry(telemetry);
                    }
                } else
                    handleAutoAim();

                // Webcam
                aprilTagWebcam.update();
                tag24 = aprilTagWebcam.getTowerTags();
                //tag23 = aprilTagWebcam.getTagBySpecificId(23);

                // Outtake velocity telemetry
                leftVel = robot.leftOuttake.getVelocity();
                rightVel = robot.rightOuttake.getVelocity();

                telemetryUpdates();
                telemetry.update();
            }
        }

        private void outake(){
            //Outtake Servo

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

            //Outtake Wheels

            if (gamepad1.right_bumper && buttonRebounceTimer.getElapsedTimeSeconds()>=0.3) {
                isLaunching = !isLaunching;

                if (isLaunching) {
                    currentVelocity = minVelocity;
                }
                buttonRebounceTimer.resetTimer();
            }


            //Outtake velocity control
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

        }

        private void handleAutoAim(){
            if (autoAimEnabled && tag24 != null) {
                double rotationPower = 0;

                // manual drive cuz rotation is controlled by autoaim
                double x = drivingGamepad.left_stick_x;
                double y = -drivingGamepad.left_stick_y;

                if (Math.abs(x) < 0.1) x = 0;
                if (Math.abs(y) < 0.1) y = 0;

//                autoAim.applyAimToDrive(x, y, rotationPower);

                autoAim.telemetry(telemetry, tag24);

            } else {
                autoAim.reset();

                if (autoAimEnabled && tag24 == null) {
                    telemetry.addLine("AUTO-AIM: Waiting for tag...");
                }
            }
        }

        private void telemetryUpdates(){
            //Webcam telemetry
//        if (tag24 != null && !autoAimEnabled) {
//            telemetry.addLine("=== TAG 24 DETECTED ===");
//            telemetry.addData("Distance (cm)", tag24.ftcPose.range);
//            telemetry.addData("X (cm)", tag24.ftcPose.x);
//            telemetry.addData("Y (cm)", tag24.ftcPose.y);
//            telemetry.addData("Z (cm)", tag24.ftcPose.z);
//            aprilTagWebcam.displayDetectionTelemetry(tag24);
//        }

            if(tag24==null){
                telemetry.addLine("=== TAG 24 NOT FOUND ===");
            }

            //Outtake Telemetry
            telemetry.addLine("");
            telemetry.addData("Left Shooter Vel", robot.leftOuttake.getVelocity());
            telemetry.addData("Right Shooter Vel", robot.rightOuttake.getVelocity());
            telemetry.addData("leftVel is: ", leftVel);
            telemetry.addData("rightVel is: ", rightVel);
        }
    }