package org.firstinspires.ftc.teamcode.kronbot.manual;



import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ANGLE_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ANGLE_SERVO_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LOADER_SERVO_REVERSED;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TURRET_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TURRET_SERVO_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.maxVelocity;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.minVelocity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.Robot;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.detection.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.kronbot.utils.pid.ControllerPID;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Button;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * The main TeleOP program for the driving period of the game.
 *
 * @version 1.0
 */
@TeleOp(name = "Main Driving", group = Constants.MAIN_GROUP)
public class MainDrivingOp extends LinearOpMode {
    private final Robot robot = new Robot();

    RobotCentricDrive robotCentricDrive;
    FieldCentricDrive fieldCentricDrive;

    Gamepad drivingGamepad;
    Gamepad utilityGamepad;
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    ControllerPID tagAlignmentPID = new ControllerPID(0.7, 0.3, 0.15);
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
        tagAlignmentPID.reset();


        Button driveModeButton = new Button();
        Button reverseButton = new Button();
        Button alignButton = new Button();
        boolean alignmentPressed = false;
        Button pivotButton = new Button();
        Button intakeMotor = new Button();
        Button intakeMotorStop = new Button();
        Button testButton = new Button();

        boolean isIntakeOn = false;
        boolean loaderOn = false;
        boolean wasShooterButtonPressed = false;
        boolean isLaunching = false;
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
            //loader servo
            if(gamepad1.left_trigger > 0.5) {
                loaderOn = true;
                if(!LOADER_SERVO_REVERSED)
                    robot.loaderServo.setPosition(0);
                else
                    robot.loaderServo.setPosition(1);

                if(isIntakeOn)
                    robot.intakeMotor.setPower(-1);
            } else {
                loaderOn=false;
                double val = gamepad1.right_trigger;
                if(!LOADER_SERVO_REVERSED)
                    val = val / 2 + 0.5;
                else {
                    val = 1 - (val / 2 + 0.5);
//                    if(isIntakeOn)
//                        robot.intakeMotor.setPower(-1);
                }
                robot.loaderServo.setPosition(val);
            }

            //pivot turret servo
//            pivotButton.updateButton(gamepad1.triangle);
//            pivotButton.shortPress();
//            if(pivotButton.shortPress()) {
//                robot.turretServo.setPosition(robot.turretServo.getPosition() == TURRET_SERVO_MAX ? TURRET_SERVO_MIN : TURRET_SERVO_MAX);
//            }

            //test
//            testButton.updateButton(gamepad1.dpad_right);
//            testButton.shortPress();
//            if(testButton.getShortToggle()) {
//                robot.angleServo.setPosition(ANGLE_SERVO_MIN);
//            } else {
//                robot.angleServo.setPosition(ANGLE_SERVO_MAX);
//            }

            //intake motor
            intakeMotor.updateButton(gamepad1.right_bumper);
            intakeMotor.shortPress();
            intakeMotorStop.updateButton(gamepad1.left_bumper);
            intakeMotorStop.shortPress();
            if(!isIntakeOn && intakeMotor.getShortToggle()) {
                robot.intakeMotor.setPower(1);
                isIntakeOn=true;
            } else if (isIntakeOn && intakeMotorStop.getShortToggle()){
                robot.intakeMotor.setPower(0);
                isIntakeOn=false;
                intakeMotor.resetToggles();
                intakeMotorStop.resetToggles();
            } else if (isIntakeOn && !loaderOn)
                robot.intakeMotor.setPower(1);

            //Outake motor
            if (gamepad1.cross && !wasShooterButtonPressed) {
                isLaunching = !isLaunching;

                if (isLaunching) {
                    currentVelocity = minVelocity;
                    robot.angleServo.setPosition(ANGLE_SERVO_MIN);
                    robot.turretServo.setPosition(TURRET_SERVO_MIN);
                }
            }

            if (isLaunching) {

                if (gamepad1.dpad_up) {
                    currentVelocity = maxVelocity;
                    robot.angleServo.setPosition(ANGLE_SERVO_MAX);
                    robot.turretServo.setPosition(TURRET_SERVO_MAX);
                } else if (gamepad1.dpad_down) {
                    currentVelocity = minVelocity;
                    robot.angleServo.setPosition(ANGLE_SERVO_MIN);
                    robot.turretServo.setPosition(TURRET_SERVO_MIN);
                }

                robot.shooterMotor.setVelocity(currentVelocity);

            } else {
                robot.shooterMotor.setPower(0);
            }

            wasShooterButtonPressed = gamepad1.cross;


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
            AprilTagDetection tag = aprilTagWebcam.getTowerTags();

            alignButton.updateButton(gamepad1.dpad_left);

            if(alignmentPressed != alignButton.press() && alignButton.press())
            {
                tagAlignmentPID.reset();
                alignmentPressed = alignButton.press();
            }

            if(tag != null)
            {
                double bearing = tag.ftcPose.bearing;

                if(Math.abs(bearing) > 1)
                    if(alignmentPressed) {
                        double value = tagAlignmentPID.calculate(0, bearing);

                        robot.motors.leftFront.setPower(value);
                        robot.motors.rightFront.setPower(-value);
                        robot.motors.leftRear.setPower(value);
                        robot.motors.rightRear.setPower(-value);
                    }
                    else {
                        tagAlignmentPID.reset();
                    }
            }

            alignmentPressed = alignButton.press();

            telemetry.addData("shooter motor vel:", robot.shooterMotor.getVelocity());
            telemetry.addData("angle servo pos:", robot.turretServo.getPosition());
            telemetry.update();
        }
    }
}