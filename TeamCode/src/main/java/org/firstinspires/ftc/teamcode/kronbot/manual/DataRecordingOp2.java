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
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.Controls;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.TurretAligner;
import org.firstinspires.ftc.teamcode.kronbot.utils.misc.LpsCounter;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

/**
 * The main TeleOP program for the driving period of the game, with data recording.
 *
 * @version 1.0
 */
@TeleOp(name = "Data Recorder", group = "Replay")
public class DataRecordingOp2 extends OpMode {
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
    private static final long RECORD_INTERVAL_MS = 20;
    private long startTime;
    private long lastRecordTime = 0;

    // Direct access to motors for recording since MainDrivingOp uses follower which hides them
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

        // Initialize the new coordinate aligner
        turretAligner = new TurretAligner(robot);
        turretAligner.setTarget(RedTowerCoords.x, RedTowerCoords.y);

        drivingGP = new Controls(gamepad1);
        utilityGP = new Controls(gamepad2);

        try {
            robot.follower.getPoseTracker().resetIMU();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // Initialize motors for recording
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        // Initialize Data Recorder
        String filePath = Environment.getExternalStorageDirectory().getPath() + "/robot_data.csv";
        try {
            dataRecorder = new FileWriter(filePath);
            // Header updated to include mechanism data
            dataRecorder.write("Time,LR,RR,LF,RF,X,Y,Heading,Voltage,IntakePwr,LoaderPwr,LeftShtrPwr,RightShtrPwr,TurretPos,AnglePos,FlapPos\n");
        } catch (IOException e) {
            telemetry.addData("Error initializing recorder", e.getMessage());
        }
    }

    @Override
    public void init_loop() {
        lpsCounter.getLoopTime();

        telemetry.addLine("Initialization Ready (Recording Enabled)");
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

        // Update Loops/s delta
        lpsCounter.getLoopTime();

        //Update controller inputs
        drivingGP.update();
        utilityGP.update();

        robot.follower.update();

        //Intake
        robot.intake.speed = utilityGP.rightStick.y;
        robot.intake.reversed = INTAKE_REVERSE;

        //Aliniere
        turretAligner.update();

        //Loader
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

        //AprilTagDetection tag = robot.webcam.getTowerTags();
        //autoAim.telemetry(telemetry, tag);

        //Turret/Angle aiming
        if (autoAimEnabled) {
            //To do
//            robot.webcam.update();

            //robot.turret.angle = autoAim.calculateServoPosition(tag);

        } else {
            //Turret aiming
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

            //Angle aiming
            if (drivingGP.dpadUp.pressed())
                robot.outtake.activeConfig.angle+= 0.01;
            else if (drivingGP.dpadDown.pressed())
                robot.outtake.activeConfig.angle -= 0.01;
        }

        //Shoot Close/Far
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

        //Update robot systems status
        robot.follower.setTeleOpDrive(-drivingGP.leftStick.y, -drivingGP.leftStick.x, -drivingGP.rightStick.x, true);
        robot.updateAllSystems();

        // Record Data
        if (now - lastRecordTime >= RECORD_INTERVAL_MS) {
            try {
                recordData();
            } catch (IOException e) {
                telemetry.addData("Recording Error", e.getMessage());
            }
            lastRecordTime = now;
        }

        _telemetry();
        //robot.webcam.update();
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
        telemetry.addData("Recording", "ACTIVE");
        telemetry.addData("x", robot.follower.getPose().getX());
        telemetry.addData("y", robot.follower.getPose().getY());
        telemetry.addData("heading", robot.follower.getPose().getHeading());
        telemetry.addData("Heading", robot.follower.getHeading());
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

    private void recordData() throws IOException {
        double t = (System.currentTimeMillis() - startTime) / 1000.0;
        // Get pose from follower (PedroPathing)
        // com.pedropathing.geometry.Pose pose = robot.follower.getPose();
        // However accessing getPose() might require checking if follower is initialized, which it should be.
        // Assuming robot.follower.getPose() returns the pose.

        double x = robot.follower.getPose().getX();
        double y = robot.follower.getPose().getY();
        double heading = robot.follower.getHeading(); // or getPose().getHeading()

        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        // Writing columns: Time, Powers(LR,RR,LF,RF), X, Y, Heading, Voltage, Mech Powers, Servo Pos
        dataRecorder.write(String.format(Locale.US,
                "%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                t,
                leftRear.getPower(),
                rightRear.getPower(),
                leftFront.getPower(),
                rightFront.getPower(),
                x,
                y,
                heading,
                voltage,
                robot.intakeMotor.getPower(),
                robot.loaderMotor.getPower(),
                robot.leftOuttake.getPower(),
                robot.rightOuttake.getPower(),
                robot.turretServo.getPosition(),
                robot.angleServo.getPosition(),
                robot.flapsServo.getPosition()
        ));
    }
}