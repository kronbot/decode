package org.firstinspires.ftc.teamcode.kronbot.manual;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_DRIVER_POWER;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_DRIVER_REVERSE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_REVERSE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_1_VELOCITY;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_2_VELOCITY;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_3_VELOCITY;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_4_VELOCITY;
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
 * TeleOp data recorder (V3.2).
 *
 * This OpMode is a faithful copy of MainDrivingOp with data-recording added.
 * Every mechanism control line is identical to MainDrivingOp so the driver
 * experiences exactly the same robot behavior while recording.
 *
 * CSV columns (22 total):
 *   Time, LR, RR, LF, RF, X, Y, Heading, Voltage,
 *   IntakeVel, LoaderVel, LeftShtrVel, RightShtrVel,
 *   TurretPos, AnglePos, FlapPos,
 *   IntakeCmd, LoaderCmd, FlapOpen, ShootRange, TurretOffset, BlueTarget
 *
 * The first 16 columns are the V3 mechanism-state log (for direct servo
 * and velocity control). The last 6 columns are the high-level mechanism
 * commands the TeleOp applied — these let the replay call the same
 * high-level API (intake.speed, shoot.activateRange, etc.) and let
 * robot.updateAllSystems() drive the motors, exactly like the TeleOp.
 *
 * V3.2 changes:
 *   - Fixed loader scalar: was * 0.9, now * 0.8 (matches MainDrivingOp)
 *   - Added rightStick.button toggle for Blue_Target (matches MainDrivingOp)
 *   - Removed turretAligner.update() from loop (TeleOp doesn't call it)
 *   - Webcam init commented out (matches MainDrivingOp)
 *   - Added 6 high-level mechanism command columns for replay parity
 *
 * @version 3.2
 */
@TeleOp(name = "RECORDERRRR", group = "Replay")
public class DataRecordingOpV3FIXED extends OpMode {
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
        // Webcam init intentionally NOT done here — matches MainDrivingOp,
        // which has it commented out. Driver doesn't see camera stream
        // during normal play, so we don't show it during recording either.

        // Initialize the new coordinate aligner
        turretAligner = new TurretAligner(robot);
        turretAligner.setTarget(RedTowerCoords.x, RedTowerCoords.y);

        drivingGP = new Controls(gamepad1);
        utilityGP = new Controls(gamepad2);

        // Initialize drive motors for recording
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        // Reset IMU for a clean starting pose — done before init_loop so
        // the driver doesn't notice any difference from MainDrivingOp.
        try {
            robot.follower.getPoseTracker().resetIMU();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // Initialize Data Recorder — V3.2 header (22 columns)
        String filePath = Environment.getExternalStorageDirectory().getPath() + "/robot_data.csv";
        try {
            dataRecorder = new FileWriter(filePath);
            dataRecorder.write("Time,LR,RR,LF,RF,X,Y,Heading,Voltage,IntakeVel,LoaderVel,LeftShtrVel,RightShtrVel,TurretPos,AnglePos,FlapPos,IntakeCmd,LoaderCmd,FlapOpen,ShootRange,TurretOffset,BlueTarget\n");
        } catch (IOException e) {
            telemetry.addData("Error initializing recorder", e.getMessage());
        }
    }

    @Override
    public void init_loop() {
        lpsCounter.getLoopTime();

        telemetry.addLine("Initialization Ready (Recording V3.2 Enabled)");
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

        // ----- Everything below this point is identical to MainDrivingOp -----

        //Intake
        robot.intake.speed = utilityGP.rightStick.y;
        robot.intake.reversed = INTAKE_REVERSE;

        //Loader
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

        //Turret aiming
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

        if(drivingGP.dpadDown.justPressed())
            robot.turret.autoAimEnabled = !robot.turret.autoAimEnabled;

        if(drivingGP.dpadUp.justPressed())
            autoAimEnabled=!autoAimEnabled;

        if(autoAimEnabled)
            robot.shoot.activateRange(0);
        //Shoot Close/Far
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

        if(drivingGP.rightStick.button.justPressed())
            robot.Blue_Target = !robot.Blue_Target;

        //Update robot systems status
        robot.follower.setTeleOpDrive(-drivingGP.leftStick.y, -drivingGP.leftStick.x, -drivingGP.rightStick.x, true);
        robot.updateAllSystems();

        // ----- End of MainDrivingOp-identical block -----

        // Record Data
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
        telemetry.addData("Recording V3.2", "ACTIVE");
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

        // Mechanism states (V3 columns)
        double intakeVel    = robot.intakeMotor.getVelocity();
        double loaderVel    = robot.loaderMotor.getVelocity();
        double leftShtrVel  = robot.leftOuttake.getVelocity();
        double rightShtrVel = robot.rightOuttake.getVelocity();
        double turretPos    = robot.turretServo.getPosition();
        double anglePos     = robot.angleServo.getPosition();
        double flapPos      = robot.flapsServo.getPosition();

        // High-level mechanism commands (V3.2 columns) — what the TeleOp
        // applied to robot.intake / robot.loader / robot.shoot / etc.
        // The replay uses these to call the same high-level API instead
        // of setting motor powers directly.
        double intakeCmd     = robot.intake.speed;
        double loaderCmd     = robot.loader.speed;
        int    flapOpen      = robot.flap.open ? 1 : 0;
        int    shootRange    = deriveShootRange();
        double turretOffset  = robot.turret.driverOffset;
        int    blueTarget    = robot.Blue_Target ? 1 : 0;

        dataRecorder.write(String.format(Locale.US,
                "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d,%.4f,%d\n",
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
                turretPos,
                anglePos,
                flapPos,
                intakeCmd,
                loaderCmd,
                flapOpen,
                shootRange,
                turretOffset,
                blueTarget
        ));
    }

    /**
     * Derives the current shoot range number (1-4) from the active config.
     *
     * RangeConfig doesn't store the range number itself, and Outtake doesn't
     * track which range is active — that info is only known inside
     * Shoot.activateRange() at the moment of the call. So we recover it by
     * matching activeConfig.velocity against the known RANGE_X_VELOCITY
     * constants. Returns 0 for the auto-aim interpolated range (case 0 in
     * Shoot.activateRange) and -1 when the outtake is off.
     *
     * Brittle: if you change RANGE_X_VELOCITY, the matching may break.
     * Clean fix: add {@code public int activeRange = -1;} to Robot.Outtake
     * and set it in Shoot.activateRange / deactivate, then read
     * {@code robot.outtake.activeRange} here directly.
     */
    private int deriveShootRange() {
        if (!robot.outtake.on) return -1;
        double vel = robot.outtake.activeConfig.velocity;
        double eps = 1.0; // velocity tolerance in ticks/sec
        if (Math.abs(vel - RANGE_1_VELOCITY) < eps) return 1;
        if (Math.abs(vel - RANGE_2_VELOCITY) < eps) return 2;
        if (Math.abs(vel - RANGE_3_VELOCITY) < eps) return 3;
        if (Math.abs(vel - RANGE_4_VELOCITY) < eps) return 4;
        return 0; // auto-aim interpolated range
    }
}