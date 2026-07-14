package org.firstinspires.ftc.teamcode.kronbot.manual;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_DRIVER_POWER;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_DRIVER_REVERSE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_REVERSE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RedTowerCoords;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kronbot.Robot;
import org.firstinspires.ftc.teamcode.kronbot.utils.Controls;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.TurretAligner;
import org.firstinspires.ftc.teamcode.kronbot.utils.misc.LpsCounter;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

/**
 * Data Recorder V4 — Correct Frame Recording
 *
 * Records pose + ROBOT-FRAME smoothed velocities + mechanism states at 50Hz.
 * Velocities are computed in the ROBOT'S FRAME during recording, so replay
 * does not need to rotate them. This eliminates the field→robot frame
 * mismatch bug entirely.
 *
 * CSV Format:
 *   Time,X,Y,Heading,Voltage,VxRobot,VyRobot,Omega,IntakePwr,LoaderPwr,LShooterPwr,RShooterPwr,TurretPos,AnglePos,FlapPos
 *
 * @version 5.0
 */
@TeleOp(name = "Data Recorder V4", group = "Replay")
public class DataRecordingOp4 extends OpMode {
    private final Robot robot = Robot.getInstance();
    private Controls drivingGP;
    private Controls utilityGP;
    private TurretAligner turretAligner;
    private FtcDashboard dashboard;
    private boolean autoAimEnabled = false;

    ElapsedTime turretTimer = new ElapsedTime();
    LpsCounter lpsCounter;
    boolean rumbled = false;

    // Data Recording
    private FileWriter dataRecorder;
    private static final long RECORD_INTERVAL_MS = 20;
    private long startTime;
    private long lastRecordTime = 0;

    // Velocity smoothing (in ROBOT FRAME)
    private double smoothedVxRobot = 0, smoothedVyRobot = 0, smoothedOmega = 0;
    private static final double VEL_SMOOTH_ALPHA = 0.35;

    // Previous pose for velocity computation
    private double prevX = 0, prevY = 0, prevHeading = 0;
    private long prevPoseTime = 0;
    private boolean firstPose = true;

    // Wheel velocity recording (optional debug)
    private DcMotorEx leftFront, rightFront, leftRear, rightRear;

    @Override
    public void init() {
        lpsCounter = new LpsCounter();
        lpsCounter.getLoopTime();

        robot.initFollower(hardwareMap, true);
        robot.init(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        robot.webcam.init(hardwareMap, telemetry);
        if (robot.webcam.getVisionPortal() != null) {
            dashboard.startCameraStream(robot.webcam.getVisionPortal(), 30);
        }

        turretAligner = new TurretAligner(robot);
        turretAligner.setTarget(RedTowerCoords.x, RedTowerCoords.y);

        drivingGP = new Controls(gamepad1);
        utilityGP = new Controls(gamepad2);

        try {
            robot.follower.getPoseTracker().resetIMU();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // Get motors and LOG THEIR DIRECTIONS for replay verification
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        telemetry.addLine("=== COPY THESE DIRECTIONS TO REPLAY CODE ===");
        telemetry.addData("LF", leftFront.getDirection().toString());
        telemetry.addData("RF", rightFront.getDirection().toString());
        telemetry.addData("LR", leftRear.getDirection().toString());
        telemetry.addData("RR", rightRear.getDirection().toString());
        telemetry.addLine("============================================");
        telemetry.update();

        String filePath = Environment.getExternalStorageDirectory().getPath() + "/robot_data_v4.csv";
        try {
            dataRecorder = new FileWriter(filePath);
            dataRecorder.write("Time,X,Y,Heading,Voltage,VxRobot,VyRobot,Omega,IntakePwr,LoaderPwr,LShooterPwr,RShooterPwr,TurretPos,AnglePos,FlapPos\n");
        } catch (IOException e) {
            telemetry.addData("Error initializing recorder", e.getMessage());
        }
    }

    @Override
    public void init_loop() {
        lpsCounter.getLoopTime();
        telemetry.addLine("Initialization Ready (Recording V4)");
        telemetry.addLine("Verify motor directions above match replay code!");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.follower.startTeleopDrive();
        startTime = System.currentTimeMillis();
        firstPose = true;
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();
        lpsCounter.getLoopTime();

        drivingGP.update();
        utilityGP.update();
        robot.follower.update();

        // ===== MECHANISM CONTROL (same as before) =====
        robot.intake.speed = utilityGP.rightStick.y;
        robot.intake.reversed = INTAKE_REVERSE;
        turretAligner.update();

        if (!drivingGP.rightBumper.pressed()) {
            robot.loader.speed = utilityGP.leftStick.y;
            robot.flap.open = false;
        } else {
            robot.loader.speed = drivingGP.rightTrigger - drivingGP.leftTrigger;
            robot.flap.open = true;
            if (robot.loader.speed > 0.1)
                robot.intake.speed = INTAKE_DRIVER_POWER;
            else if (robot.loader.speed < -0.2)
                robot.intake.speed = INTAKE_DRIVER_REVERSE;
            else
                robot.intake.speed = 0;
        }

        // Turret/Angle aiming
        if (autoAimEnabled) {
            // To do
        } else {
            if (drivingGP.dpadLeft.pressed()) {
                if (turretTimer.seconds() == 0) turretTimer.reset();
                double increment = turretTimer.seconds() > 1.5 ? 0.1 :
                        turretTimer.seconds() > 1.0 ? 0.07 : 0.03;
                robot.turret.driverOffset += increment;
            } else if (drivingGP.dpadRight.pressed()) {
                if (turretTimer.seconds() == 0) turretTimer.reset();
                double decrement = turretTimer.seconds() > 1.5 ? 0.1 :
                        turretTimer.seconds() > 1.0 ? 0.07 : 0.03;
                robot.turret.driverOffset -= decrement;
            } else {
                turretTimer.reset();
            }

            if (drivingGP.dpadUp.pressed())
                robot.outtake.activeConfig.angle += 0.01;
            else if (drivingGP.dpadDown.pressed())
                robot.outtake.activeConfig.angle -= 0.01;
        }

        // Shoot presets
        if (drivingGP.triangle.justPressed()) {
            robot.turret.autoAimEnabled = false;
            robot.shoot.activateRange(1);
        }
        if (drivingGP.square.justPressed()) {
            robot.turret.autoAimEnabled = false;
            robot.shoot.activateRange(2);
        }
        if (drivingGP.cross.justPressed()) {
            robot.turret.autoAimEnabled = false;
            robot.shoot.activateRange(3);
        }
        if (drivingGP.circle.justPressed()) {
            robot.turret.autoAimEnabled = false;
            robot.shoot.activateRange(4);
        }

        // Rumble when shooter ready
        if (robot.outtake.on &&
                robot.leftOuttake.getVelocity() >= robot.outtake.activeConfig.velocity - 30 &&
                robot.leftOuttake.getVelocity() <= robot.outtake.activeConfig.velocity + 90) {
            gamepad1.rumble(1, 0, 150);
            rumbled = true;
        }

        if (!autoAimEnabled && drivingGP.leftBumper.justPressed()) {
            robot.turret.autoAimEnabled = true;
            if (robot.outtake.on) {
                robot.shoot.deactivate();
                gamepad1.rumble(1, 1, 100);
                rumbled = false;
            }
        }

        robot.follower.setTeleOpDrive(-drivingGP.leftStick.y, -drivingGP.leftStick.x, -drivingGP.rightStick.x, true);
        robot.updateAllSystems();

        // ===== RECORD DATA =====
        if (now - lastRecordTime >= RECORD_INTERVAL_MS) {
            try {
                recordData(now);
            } catch (IOException e) {
                telemetry.addData("Recording Error", e.getMessage());
            }
            lastRecordTime = now;
        }

        _telemetry();
    }

    @Override
    public void stop() {
        robot.webcam.stop();
        if (dataRecorder != null) {
            try {
                dataRecorder.flush();
                dataRecorder.close();
            } catch (IOException ignored) {}
        }
    }

    private void recordData(long now) throws IOException {
        double t = (now - startTime) / 1000.0;
        double x = robot.follower.getPose().getX();
        double y = robot.follower.getPose().getY();
        double heading = robot.follower.getHeading();
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        // Compute raw velocity from pose delta (in FIELD FRAME)
        double rawVxField = 0, rawVyField = 0, rawOmega = 0;
        if (!firstPose) {
            double dt = (now - prevPoseTime) / 1000.0;
            if (dt > 0.001) {
                rawVxField = (x - prevX) / dt;
                rawVyField = (y - prevY) / dt;
                rawOmega = normalizeAngle(heading - prevHeading) / dt;
            }
        } else {
            firstPose = false;
        }

        // Rotate field-frame velocity into ROBOT FRAME
        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);
        double rawVxRobot = cosH * rawVxField + sinH * rawVyField;
        double rawVyRobot = -sinH * rawVxField + cosH * rawVyField;

        // Exponential smoothing (in ROBOT FRAME)
        smoothedVxRobot = smoothedVxRobot + VEL_SMOOTH_ALPHA * (rawVxRobot - smoothedVxRobot);
        smoothedVyRobot = smoothedVyRobot + VEL_SMOOTH_ALPHA * (rawVyRobot - smoothedVyRobot);
        smoothedOmega = smoothedOmega + VEL_SMOOTH_ALPHA * (rawOmega - smoothedOmega);

        // Update previous pose
        prevX = x;
        prevY = y;
        prevHeading = heading;
        prevPoseTime = now;

        dataRecorder.write(String.format(Locale.US,
                "%.3f,%.4f,%.4f,%.4f,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                t, x, y, heading, voltage,
                smoothedVxRobot, smoothedVyRobot, smoothedOmega,
                robot.intakeMotor.getPower(),
                robot.loaderMotor.getPower(),
                robot.leftOuttake.getPower(),
                robot.rightOuttake.getPower(),
                robot.turretServo.getPosition(),
                robot.angleServo.getPosition(),
                robot.flapsServo.getPosition()
        ));
    }

    private static double normalizeAngle(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    public void _telemetry() {
        telemetry.addData("LPS", "%.1f", 1 / lpsCounter.delta);
        telemetry.addData("Recording", "V4 ACTIVE (robot-frame velocities)");
        telemetry.addData("x", robot.follower.getPose().getX());
        telemetry.addData("y", robot.follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(robot.follower.getPose().getHeading()));
        telemetry.addData("robot V", "%.1f, %.1f, %.1f°/s", smoothedVxRobot, smoothedVyRobot, Math.toDegrees(smoothedOmega));
        telemetry.addData("shooter vel", robot.leftOuttake.getVelocity());
        telemetry.addData("turret pos", robot.turretServo.getPosition());
        robot.intake.telemetry(telemetry);
        robot.loader.telemetry(telemetry);
        robot.outtake.telemetry(telemetry);
        drivingGP.telemetry(telemetry);
        telemetry.update();
    }
}
