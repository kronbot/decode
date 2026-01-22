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
import android.os.Environment;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.detection.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Button;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name = "Turret Data Recorder", group = Constants.MAIN_GROUP)
public class TurretDataOp extends LinearOpMode {
    private final KronBot robot = new KronBot();

    Gamepad utilityGamepad;
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private FtcDashboard dashboard;

    // Data recording
    private FileWriter dataRecorder;
    private int recordCount = 0;

    // Joystick control variables
    private double currentVelocity = minVelocity;
    private double currentAngleServoPosition = ANGLE_SERVO_MIN;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            robot.initTeleop(hardwareMap);
            dashboard = FtcDashboard.getInstance();
            telemetry.update();

            utilityGamepad = gamepad1;

            aprilTagWebcam.init(hardwareMap, telemetry);
            if (aprilTagWebcam.getVisionPortal() != null) {
                dashboard.startCameraStream(aprilTagWebcam.getVisionPortal(), 30);
            }

            // Setup CSV file
            String filePath = Environment.getExternalStorageDirectory().getPath() + "/shooter_data.csv";
            dataRecorder = new FileWriter(filePath);
            dataRecorder.write("RecordNumber,AprilTagDistance,ShooterVelocity,AngleServoPosition\n");

            Button pivotButton = new Button();
            Button intakeMotor = new Button();
            Button intakeMotorStop = new Button();
            Button recordButton = new Button();

            boolean isIntakeOn = false;

            while (!isStopRequested() && !opModeIsActive()) {
                telemetry.addLine("Selective Data Recorder Ready");
                telemetry.addLine("Left Joystick Y: Shooter Velocity");
                telemetry.addLine("Right Joystick Y: Angle Servo Position");
                telemetry.addLine("Press DPAD_RIGHT to record data point");
                telemetry.addData("File Path", filePath);
                telemetry.update();
            }

            if (isStopRequested()) {
                aprilTagWebcam.stop();
                return;
            }

            while (opModeIsActive() && !isStopRequested()) {
                // Loader servo
                if (gamepad1.left_trigger > 0.5) {
                    if (!LOADER_SERVO_REVERSED)
                        robot.loaderServo.setPosition(0);
                    else
                        robot.loaderServo.setPosition(1);
                } else {
                    double val = gamepad1.right_trigger;
                    if (!LOADER_SERVO_REVERSED)
                        val = val / 2 + 0.5;
                    else
                        val = 1 - (val / 2 + 0.5);

                    robot.loaderServo.setPosition(val);
                }

                // Pivot turret servo
                pivotButton.updateButton(gamepad1.triangle);
                pivotButton.shortPress();
                if (pivotButton.getShortToggle()) {
                    robot.turretServo.setPosition(TURRET_SERVO_MIN);
                } else {
                    robot.turretServo.setPosition(TURRET_SERVO_MAX);
                }

                // Intake motor
                intakeMotor.updateButton(gamepad1.right_bumper);
                intakeMotor.shortPress();
                intakeMotorStop.updateButton(gamepad1.left_bumper);
                intakeMotorStop.shortPress();
                if (!isIntakeOn && intakeMotor.getShortToggle()) {
                    robot.intakeMotor.setPower(1);
                    isIntakeOn = true;
                } else if (isIntakeOn && intakeMotorStop.getShortToggle()) {
                    robot.intakeMotor.setPower(0);
                    isIntakeOn = false;
                    intakeMotor.resetToggles();
                    intakeMotorStop.resetToggles();
                }

                // JOYSTICK 1 (Left stick Y-axis): Control shooter motor velocity
                // Map joystick range [-1, 1] to velocity range [minVelocity, maxVelocity]
                double leftStickY = -gamepad1.left_stick_y; // Inverted for intuitive control (up = increase)
                if (Math.abs(leftStickY) > 0.05) { // Deadzone
                    // Map from [-1, 1] to [minVelocity, maxVelocity]
                    currentVelocity = minVelocity + ((leftStickY + 1) / 2.0) * (maxVelocity - minVelocity);
                }
                robot.shooterMotor.setVelocity(currentVelocity);

                // JOYSTICK 2 (Right stick Y-axis): Control angle servo position
                // Map joystick range [-1, 1] to servo range [ANGLE_SERVO_MIN, 1.0] (full servo range)
                double rightStickY = -gamepad1.right_stick_y; // Inverted for intuitive control
                if (Math.abs(rightStickY) > 0.05) { // Deadzone
                    // Map from [-1, 1] to [ANGLE_SERVO_MIN, 1.0]
                    currentAngleServoPosition = ANGLE_SERVO_MIN + ((rightStickY + 1) / 2.0) * (1.0 - ANGLE_SERVO_MIN);
                }
                robot.angleServo.setPosition(currentAngleServoPosition);

                aprilTagWebcam.update();
                AprilTagDetection tag = aprilTagWebcam.getTowerTags();

                // DATA RECORDING - DPAD_RIGHT button press
                recordButton.updateButton(gamepad1.dpad_right);
                if (recordButton.toggle()) {
                    recordDataPoint(tag);
                }

                telemetry.addData("Shooter Velocity", "%.2f", currentVelocity);
                telemetry.addData("Angle Servo Position", "%.4f", currentAngleServoPosition);
                telemetry.addData("Actual Shooter Velocity", "%.2f", robot.shooterMotor.getVelocity());
                telemetry.addData("Records Captured", recordCount);
                telemetry.addLine("---");
                telemetry.addLine("Left Stick Y: Adjust Velocity");
                telemetry.addLine("Right Stick Y: Adjust Angle");
                telemetry.addLine("DPAD_RIGHT: Record Data");
                telemetry.update();
            }
        } catch (IOException e) {
            telemetry.addData("ERROR", e.toString());
            telemetry.update();
            throw new RuntimeException(e);
        } finally {
            if (dataRecorder != null) {
                try {
                    dataRecorder.flush();
                    dataRecorder.close();
                } catch (IOException ignored) {
                }
            }
            aprilTagWebcam.stop();
        }
    }

    private synchronized void recordDataPoint(AprilTagDetection tag) throws IOException {
        double aprilTagDistance = -1; // Default if no tag detected
        if (tag != null) {
            aprilTagDistance = tag.ftcPose.y; // Distance to AprilTag
        }

        double shooterVelocity = robot.shooterMotor.getVelocity();
        double angleServoPosition = robot.angleServo.getPosition();

        recordCount++;

        dataRecorder.write(String.format(
                "%d,%.4f,%.2f,%.4f\n",
                recordCount,
                aprilTagDistance,
                shooterVelocity,
                angleServoPosition
        ));
        dataRecorder.flush();

        telemetry.addLine("*** DATA RECORDED ***");
        telemetry.addData("Distance", aprilTagDistance);
        telemetry.addData("Velocity", shooterVelocity);
        telemetry.addData("Angle Servo", angleServoPosition);
        telemetry.update();
    }
}