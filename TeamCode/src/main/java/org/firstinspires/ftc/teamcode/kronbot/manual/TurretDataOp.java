package org.firstinspires.ftc.teamcode.kronbot.manual;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ANGLE_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ANGLE_SERVO_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LOADER_SERVO_REVERSED;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TEST_LAUNCH_ANGLE_DELTA;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TEST_LAUNCH_MOTOR_DELTA;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TEST_TURRET_PIVOT_DELTA;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TURRET_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TURRET_SERVO_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.maxVelocity;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.minVelocity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

    Gamepad drivingGamepad;
    Gamepad utilityGamepad;
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private FtcDashboard dashboard;

    // Data recording
    private FileWriter dataRecorder;
    private int recordCount = 0;


    private double launchAngle = 0;
    private double launchVelocity = 0;
    private double turretPivotAngle = 0;

    private boolean lastIntakeButton = false;
    private boolean lastIntakeOn = false;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            robot.initTeleop(hardwareMap);
            dashboard = FtcDashboard.getInstance();
            telemetry.update();

            drivingGamepad = gamepad1;
            utilityGamepad = gamepad2;

            aprilTagWebcam.init(hardwareMap, telemetry);
            if (aprilTagWebcam.getVisionPortal() != null) {
                dashboard.startCameraStream(aprilTagWebcam.getVisionPortal(), 30);
            }

            // Setup CSV file
            String filePath = Environment.getExternalStorageDirectory().getPath() + "/shooter_data.csv";
            dataRecorder = new FileWriter(filePath);
            dataRecorder.write("RecordNumber,AprilTagDistance,LaunchVel,LaunchAngle\n");

            Button pivotButton = new Button();
            Button intakeMotor = new Button();
            Button intakeMotorStop = new Button();
            Button recordButton = new Button();

            boolean isIntakeOn = false;

            while (!isStopRequested() && !opModeIsActive()) {
                telemetry.addData("File Path", filePath);
                telemetry.update();
            }

            if (isStopRequested()) {
                aprilTagWebcam.stop();
                return;
            }

            while (opModeIsActive() && !isStopRequested()) {
                double deltaT = getLoopTime();

                // Loader servo
                if (utilityGamepad.right_bumper) {
                    if (!LOADER_SERVO_REVERSED)
                        robot.loaderServo.setPosition(0);
                    else
                        robot.loaderServo.setPosition(1);
                } else {
                    double val = utilityGamepad.right_trigger;
                    if (!LOADER_SERVO_REVERSED)
                        val = val / 2 + 0.5;
                    else
                        val = 1 - (val / 2 + 0.5);

                    robot.loaderServo.setPosition(val);
                }

                // Pivot turret servo
                double pivot = utilityGamepad.left_stick_x;
                if(Math.abs(pivot) > 0.02)
                    turretPivotAngle += pivot * TEST_TURRET_PIVOT_DELTA * deltaT;

                if(turretPivotAngle < TURRET_SERVO_MIN)
                    turretPivotAngle = TURRET_SERVO_MIN;
                if(turretPivotAngle > TURRET_SERVO_MAX)
                    turretPivotAngle = TURRET_SERVO_MAX;

                robot.turretServo.setPosition(turretPivotAngle);


                // Launch Angle Servo
                double angle = utilityGamepad.left_stick_y;
                if(Math.abs(angle) > 0.02)
                    launchAngle += angle * TEST_LAUNCH_ANGLE_DELTA * deltaT;

                if(launchAngle < ANGLE_SERVO_MIN)
                    launchAngle = ANGLE_SERVO_MIN;
                if(launchAngle > ANGLE_SERVO_MAX)
                    launchAngle = ANGLE_SERVO_MAX;

                robot.angleServo.setPosition(launchAngle);


                // Launch Velocity
                double vel = utilityGamepad.right_stick_y;
                if(Math.abs(vel) > 0.02)
                    launchVelocity += vel * TEST_LAUNCH_MOTOR_DELTA * deltaT;

                if(launchVelocity < 0)
                    launchVelocity = 0;
                if(launchVelocity > 2000)
                    launchVelocity = 2000;

                robot.shooterMotor.setVelocity(launchVelocity);


                // Intake motor
                if(lastIntakeButton != gamepad1.right_bumper && !lastIntakeButton) {
                    if (lastIntakeOn) {
                        robot.intakeMotor.setPower(0);
                        lastIntakeOn = true;
                    } else {
                        robot.intakeMotor.setPower(1);
                        lastIntakeOn = false;
                    }
                }
                lastIntakeButton = gamepad1.right_bumper;



                aprilTagWebcam.update();
                AprilTagDetection tag = aprilTagWebcam.getTowerTags();

                double distance = -1;
                if(tag != null)
                    distance = tag.ftcPose.range;

                // DATA RECORDING - DPAD_RIGHT button press
                recordButton.updateButton(gamepad1.dpad_right);
                if (recordButton.toggle()) {
                }

                telemetry.addData("Shooter Velocity", launchVelocity);
                telemetry.addData("Actual Shooter Velocity", robot.shooterMotor.getVelocity());
                telemetry.addData("Angle Servo Position",  launchAngle);
                telemetry.addData("Turret Pivot Angle", turretPivotAngle);
                telemetry.addData("Tag Range", distance);
                telemetry.addLine("---");
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



    boolean started = false;
    ElapsedTime timer = new ElapsedTime();

    private double getLoopTime() {
        if (!started) {
            started = true;
            timer.reset();
        }
        double time = timer.seconds();
        timer.reset();
        return time;
    }
}