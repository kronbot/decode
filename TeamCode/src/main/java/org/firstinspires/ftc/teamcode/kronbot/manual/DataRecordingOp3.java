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
 * Enhanced TeleOP data recorder (V3).
 * Records drive motor powers (for feedforward replay) and mechanism velocities
 * (for faithful shooter reproduction).
 *
 * CSV columns (17 total):
 * Time, LR, RR, LF, RF, X, Y, Heading, Voltage,
 * IntakeVel, LoaderVel, LeftShtrVel, RightShtrVel,
 * TurretPos, AnglePos, FlapPos
 *
 * Key differences from DataRecordingOp2:
 * - Mechanism columns record velocities instead of raw powers (better for shooter replay)
 * - Uses ElapsedTime (nanosecond resolution) for timestamps, matching the replay timer exactly
 * - Otherwise identical format for drive motors (LR, RR, LF, RF powers)
 *
 * @version 3.0
 */
@TeleOp(name = "Data Recorder V3", group = "Replay")
public class DataRecordingOp3 extends OpMode {
    private final Robot robot = Robot.getInstance();
    private Controls drivingGP;
    private Controls utilityGP;

    private TurretAligner turretAligner;

    private FtcDashboard dashboard;

    private boolean autoAimEnabled = false;

    ElapsedTime turretTimer = new ElapsedTime();

    LpsCounter lpsCounter;

    boolean rumbled = false;

    // Data Recording Fields
    private FileWriter dataRecorder;
    private static final double RECORD_INTERVAL_SEC = 0.020; // 20ms in seconds
    private ElapsedTime recordTimer = new ElapsedTime();
    private double lastRecordTime = 0;

    // Direct access to drive motors for recording (follower hides them)
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

        // Initialize drive motors for recording
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        // Initialize Data Recorder
        String filePath = Environment.getExternalStorageDirectory().getPath() + "/robot_data.csv";
        try {
            dataRecorder = new FileWriter(filePath);
            dataRecorder.write("Time,LR,RR,LF,RF,X,Y,Heading,Voltage,IntakeVel,LoaderVel,LeftShtrVel,RightShtrVel,TurretPos,AnglePos,FlapPos\n");
        } catch (IOException e) {
            telemetry.addData("Error initializing recorder", e.getMessage());
        }
    }

    @Override
    public void init_loop() {
        lpsCounter.getLoopTime();

        telemetry.addLine("Initialization Ready (Recording V3 Enabled)");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.follower.startTeleopDrive();
        recordTimer.reset();
    }

    @Override
    public void loop() {
        double now = recordTimer.seconds();

        lpsCounter.getLoopTime();

        drivingGP.update();
        utilityGP.update();

        robot.follower.update();

        // Intake
        robot.intake.speed = utilityGP.rightStick.y;
        robot.intake.reversed = INTAKE_REVERSE;

        // Alignment
        turretAligner.update();

        // Loader
        if (!drivingGP.rightBumper.pressed()) {
            robot.loader.speed = utilityGP.leftStick.y;
            robot.flap.open = false;
        } else {
            robot.loader.speed = (drivingGP.rightTrigger - drivingGP.leftTrigger) * 0.9;
            robot.flap.open = true;
            if (robot.loader.speed > 0.1)
                robot.intake.speed = INTAKE_DRIVER_POWER;
            else if (robot.loader.speed < -0.2)
                robot.intake.speed = INTAKE_DRIVER_REVERSE;
            else
                robot.intake.speed = 0;
        }

        // Turret/Angle aiming

        // Turret aiming
        if (drivingGP.dpadLeft.pressed()) {

            //if button is pressed for longer, increase increment
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

        if(drivingGP.dpadDown.justPressed())
            robot.turret.autoAimEnabled = !robot.turret.autoAimEnabled;

        if(drivingGP.dpadUp.justPressed())
            autoAimEnabled=!autoAimEnabled;

        if(autoAimEnabled)
            robot.shoot.activateRange(0);
        // Shoot ranges
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

        // Update robot systems
        robot.follower.setTeleOpDrive(-drivingGP.leftStick.y, -drivingGP.leftStick.x, -drivingGP.rightStick.x, true);
        robot.updateAllSystems();

        // Record Data (Fix #5: uses same ElapsedTime as replay for consistent timestamps)
        if (now - lastRecordTime >= RECORD_INTERVAL_SEC) {
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
            } catch (IOException ignored) {
            }
        }
    }

    public void _telemetry() {
        telemetry.addData("LPS", "%.1f", 1 / lpsCounter.delta);
        telemetry.addData("Recording V3", "ACTIVE");
        telemetry.addData("x", robot.follower.getPose().getX());
        telemetry.addData("y", robot.follower.getPose().getY());
        telemetry.addData("heading", robot.follower.getPose().getHeading());
        telemetry.addData("Heading", robot.follower.getHeading());
        telemetry.addData("Drive Powers", "LF:%.2f RF:%.2f LR:%.2f RR:%.2f",
                leftFront.getPower(), rightFront.getPower(), leftRear.getPower(), rightRear.getPower());
        telemetry.addData("shooter motor vel:", robot.leftOuttake.getVelocity());
        telemetry.addData("angle servo pos:", robot.turretServo.getPosition());
        telemetry.addData("turret angle:", robot.turret.angle);
        robot.intake.telemetry(telemetry);
        robot.loader.telemetry(telemetry);
        robot.outtake.telemetry(telemetry);
        robot.heading.telemetry(telemetry);
        robot.turret.telemetry(telemetry);
        drivingGP.telemetry(telemetry);
        telemetry.update();
    }

    private void recordData(double t) throws IOException {

        double x = robot.follower.getPose().getX();
        double y = robot.follower.getPose().getY();
        double heading = robot.follower.getHeading();

        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        // Record mechanism velocities instead of raw powers for better reproduction
        double intakeVel = robot.intakeMotor.getVelocity();
        double loaderVel = robot.loaderMotor.getVelocity();
        double leftShtrVel = robot.leftOuttake.getVelocity();
        double rightShtrVel = robot.rightOuttake.getVelocity();

        // CSV: Time, LR, RR, LF, RF, X, Y, Heading, Voltage, IntakeVel, LoaderVel, LeftShtrVel, RightShtrVel, TurretPos, AnglePos, FlapPos
        dataRecorder.write(String.format(Locale.US,
                "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                t,
                leftRear.getPower(),
                rightRear.getPower(),
                leftFront.getPower(),
                rightFront.getPower(),
                x,
                y,
                heading,
                voltage,
                intakeVel,
                loaderVel,
                leftShtrVel,
                rightShtrVel,
                robot.turretServo.getPosition(),
                robot.angleServo.getPosition(),
                robot.flapsServo.getPosition()
        ));
    }
}