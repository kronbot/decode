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
 * Data Recorder V7 — Driver Input Feedforward + Continuous Blending Replay
 *
 * Records the ACTUAL DRIVER INPUTS (gamepad stick values) that caused the motion,
 * plus pose for correction reference.
 *
 * CSV Format:
 *   Time,X,Y,Heading,Voltage,
 *   GamepadFwd,GamepadStr,GamepadTurn,  // [-1, 1] robot-frame stick values
 *   IntakePwr,LoaderPwr,LShooterPwr,RShooterPwr,
 *   TurretPos,AnglePos,FlapPos
 *
 * @version 7.0
 */
@TeleOp(name = "Data Recorder V7", group = "Replay")
public class DataRecordingOp7 extends OpMode {
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

        String filePath = Environment.getExternalStorageDirectory().getPath() + "/robot_data_v7.csv";
        try {
            dataRecorder = new FileWriter(filePath);
            dataRecorder.write("Time,X,Y,Heading,Voltage,GamepadFwd,GamepadStr,GamepadTurn,IntakePwr,LoaderPwr,LShooterPwr,RShooterPwr,TurretPos,AnglePos,FlapPos\n");
        } catch (IOException e) {
            telemetry.addData("Error initializing recorder", e.getMessage());
        }
    }

    @Override
    public void init_loop() {
        lpsCounter.getLoopTime();
        telemetry.addLine("Initialization Ready (Recording V7)");
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
        // These are the EXACT values passed to setTeleOpDrive
        // Note: setTeleOpDrive takes (forward, strafe, turn, robotCentric=true)
        // We negate Y because gamepad Y is up=negative in FTC
        double gamepadFwd  = -drivingGP.leftStick.y;   // forward/back
        double gamepadStr  = -drivingGP.leftStick.x;   // strafe
        double gamepadTurn = -drivingGP.rightStick.x;  // turn

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

        // Pass inputs to PedroPathing (same as always)
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
                "%.3f,%.4f,%.4f,%.4f,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                t, x, y, heading, voltage,
                gpFwd, gpStr, gpTurn,
                robot.intakeMotor.getPower(),
                robot.loaderMotor.getPower(),
                robot.leftOuttake.getPower(),
                robot.rightOuttake.getPower(),
                robot.turretServo.getPosition(),
                robot.angleServo.getPosition(),
                robot.flapsServo.getPosition()
        ));
    }

    public void _telemetry(double gpFwd, double gpStr, double gpTurn) {
        telemetry.addData("LPS", "%.1f", 1 / lpsCounter.delta);
        telemetry.addData("Recording", "V7 ACTIVE (input feedforward)");
        telemetry.addData("Inputs", "fwd=%.2f str=%.2f turn=%.2f", gpFwd, gpStr, gpTurn);
        telemetry.addData("x", robot.follower.getPose().getX());
        telemetry.addData("y", robot.follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(robot.follower.getPose().getHeading()));
        telemetry.addData("shooter vel", robot.leftOuttake.getVelocity());
        telemetry.addData("turret pos", robot.turretServo.getPosition());
        robot.intake.telemetry(telemetry);
        robot.loader.telemetry(telemetry);
        robot.outtake.telemetry(telemetry);
        drivingGP.telemetry(telemetry);
        telemetry.update();
    }
}
