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
 * Data Recorder V8 — Exact Match to MainDrivingOp
 *
 * Controls and mechanisms behave IDENTICALLY to MainDrivingOp.
 * Records driver inputs + pose + all mechanism states.
 *
 * CSV Format:
 *   Time,X,Y,Heading,Voltage,
 *   GamepadFwd,GamepadStr,GamepadTurn,
 *   IntakePwr,LoaderPwr,LShooterPwr,RShooterPwr,
 *   TurretPos,AnglePos,FlapPos,
 *   AutoAimEnabled,BlueTarget
 */
@TeleOp(name = "Data Recorder V8", group = "Replay")
public class DataRecordingOp8 extends OpMode {
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

        String filePath = Environment.getExternalStorageDirectory().getPath() + "/robot_data_v8.csv";
        try {
            dataRecorder = new FileWriter(filePath);
            dataRecorder.write("Time,X,Y,Heading,Voltage,GamepadFwd,GamepadStr,GamepadTurn,IntakePwr,LoaderPwr,LShooterPwr,RShooterPwr,TurretPos,AnglePos,FlapPos,AutoAim,BlueTarget\n");
        } catch (IOException e) {
            telemetry.addData("Error initializing recorder", e.getMessage());
        }
    }

    @Override
    public void init_loop() {
        lpsCounter.getLoopTime();
        telemetry.addLine("Initialization Ready (Recording V8)");
        telemetry.addLine("Verify motor directions above match replay code!");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.follower.startTeleopDrive();
        startTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();
        lpsCounter.getLoopTime();

        drivingGP.update();
        utilityGP.update();
        robot.follower.update();

        // ===== CAPTURE DRIVER INPUTS (the feedforward signal) =====
        // EXACTLY as passed to setTeleOpDrive in MainDrivingOp
        double gamepadFwd  = -drivingGP.leftStick.y;
        double gamepadStr  = -drivingGP.leftStick.x;
        double gamepadTurn = -drivingGP.rightStick.x;

        // ===== MECHANISM CONTROL — EXACT COPY OF MainDrivingOp =====

        // Intake
        robot.intake.speed = utilityGP.rightStick.y;
        robot.intake.reversed = INTAKE_REVERSE;

        // Loader
        if (!drivingGP.rightBumper.pressed()) {
            robot.loader.speed = utilityGP.leftStick.y;
            robot.flap.open = false;
        } else {
            robot.loader.speed = (drivingGP.rightTrigger - drivingGP.leftTrigger) * 0.8;
            robot.flap.open = true;
            if (robot.loader.speed > 0.1)
                robot.intake.speed = INTAKE_DRIVER_POWER;
            else if (robot.loader.speed < -0.2)
                robot.intake.speed = INTAKE_DRIVER_REVERSE;
            else
                robot.intake.speed = 0;
        }

        // Turret/Angle aiming — EXACT copy from MainDrivingOp
        if (drivingGP.dpadLeft.pressed()) {
            if (turretTimer.seconds() == 0) {
                turretTimer.reset();
            }
            double increment = 0.03;
            if (turretTimer.seconds() > 1) {
                increment = 0.07;
            }
            if (turretTimer.seconds() > 1.5) {
                increment = 0.1;
            }
            robot.turret.driverOffset += increment;
        } else if (drivingGP.dpadRight.pressed()) {
            double decrement = 0.03;
            if (turretTimer.seconds() == 0) {
                turretTimer.reset();
            }
            if (turretTimer.seconds() > 1) {
                decrement = 0.07;
            }
            if (turretTimer.seconds() > 1.5) {
                decrement = 0.1;
            }
            robot.turret.driverOffset -= decrement;
        } else {
            turretTimer.reset();
        }

        // Auto-aim toggles — EXACT copy from MainDrivingOp
        if (drivingGP.dpadDown.justPressed())
            robot.turret.autoAimEnabled = !robot.turret.autoAimEnabled;

        if (drivingGP.dpadUp.justPressed())
            autoAimEnabled = !autoAimEnabled;

        if (autoAimEnabled)
            robot.shoot.activateRange(0);

        // Shoot presets — EXACT copy from MainDrivingOp
        if (drivingGP.triangle.justPressed()) {
            robot.shoot.activateRange(1);
        }
        if (drivingGP.square.justPressed()) {
            robot.shoot.activateRange(2);
        }
        if (drivingGP.cross.justPressed()) {
            robot.shoot.activateRange(3);
        }
        if (drivingGP.circle.justPressed()) {
            robot.shoot.activateRange(4);
        }

        // Shooter ready rumble — EXACT copy from MainDrivingOp
        if (robot.outtake.on &&
                robot.leftOuttake.getVelocity() >= robot.outtake.activeConfig.velocity - 30 &&
                robot.leftOuttake.getVelocity() <= robot.outtake.activeConfig.velocity + 90) {
            gamepad1.rumble(1, 0, 150);
            rumbled = true;
        }

        // Left bumper — EXACT copy from MainDrivingOp
        if (!autoAimEnabled && drivingGP.leftBumper.justPressed()) {
            robot.turret.autoAimEnabled = true;
            if (robot.outtake.on) {
                robot.shoot.deactivate();
                gamepad1.rumble(1, 1, 100);
                rumbled = false;
            }
        }

        // Blue target toggle — EXACT copy from MainDrivingOp
        if (drivingGP.rightStick.button.justPressed())
            robot.Blue_Target = !robot.Blue_Target;

        // Update robot systems — EXACT copy from MainDrivingOp
        robot.follower.setTeleOpDrive(gamepadFwd, gamepadStr, gamepadTurn, true);
        robot.updateAllSystems();

        // ===== RECORD DATA =====
        if (now - lastRecordTime >= RECORD_INTERVAL_MS) {
            try {
                recordData(now, gamepadFwd, gamepadStr, gamepadTurn);
            } catch (IOException e) {
                telemetry.addData("Recording Error", e.getMessage());
            }
            lastRecordTime = now;
        }

        _telemetry(gamepadFwd, gamepadStr, gamepadTurn);
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

    private void recordData(long now, double gpFwd, double gpStr, double gpTurn) throws IOException {
        double t = (now - startTime) / 1000.0;
        double x = robot.follower.getPose().getX();
        double y = robot.follower.getPose().getY();
        double heading = robot.follower.getHeading();
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        dataRecorder.write(String.format(Locale.US,
                "%.3f,%.4f,%.4f,%.4f,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d\n",
                t, x, y, heading, voltage,
                gpFwd, gpStr, gpTurn,
                robot.intakeMotor.getPower(),
                robot.loaderMotor.getPower(),
                robot.leftOuttake.getPower(),
                robot.rightOuttake.getPower(),
                robot.turretServo.getPosition(),
                robot.angleServo.getPosition(),
                robot.flapsServo.getPosition(),
                autoAimEnabled ? 1 : 0,
                robot.Blue_Target ? 1 : 0
        ));
    }

    public void _telemetry(double gpFwd, double gpStr, double gpTurn) {
        telemetry.addData("LPS", "%.1f", 1 / lpsCounter.delta);
        telemetry.addData("Recording", "V8 ACTIVE (input feedforward)");
        telemetry.addData("Inputs", "fwd=%.2f str=%.2f turn=%.2f", gpFwd, gpStr, gpTurn);
        telemetry.addData("x", robot.follower.getPose().getX());
        telemetry.addData("y", robot.follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(robot.follower.getPose().getHeading()));
        telemetry.addData("shooter vel", robot.leftOuttake.getVelocity());
        telemetry.addData("turret pos", robot.turretServo.getPosition());
        telemetry.addData("autoAim", autoAimEnabled);
        telemetry.addData("BlueTarget", robot.Blue_Target);
        robot.intake.telemetry(telemetry);
        robot.loader.telemetry(telemetry);
        robot.outtake.telemetry(telemetry);
        drivingGP.telemetry(telemetry);
        telemetry.update();
    }
}
