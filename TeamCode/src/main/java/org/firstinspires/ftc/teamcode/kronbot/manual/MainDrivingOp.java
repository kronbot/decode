package org.firstinspires.ftc.teamcode.kronbot.manual;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.BlueTowerCoords;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_DRIVER_POWER;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_DRIVER_REVERSE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_REVERSE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.OUT_MOTOR_KD;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.OUT_MOTOR_KF;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.OUT_MOTOR_KI;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.OUT_MOTOR_KP;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_1_VELOCITY;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_2_VELOCITY;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_3_VELOCITY;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_4_VELOCITY;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RedTowerCoords;

import com.acmerobotics.dashboard.FtcDashboard;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kronbot.Robot;
import org.firstinspires.ftc.teamcode.kronbot.utils.Controls;
import org.firstinspires.ftc.teamcode.kronbot.utils.PoseStorage;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.AutoAim;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.TurretAligner;
import org.firstinspires.ftc.teamcode.kronbot.utils.misc.LpsCounter;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * The main TeleOP program for the driving period of the game.
 *
 * @version 1.0
 */
@TeleOp(name = "Main Driving", group = Constants.MAIN_GROUP)
public class MainDrivingOp extends OpMode {
    private final Robot robot = Robot.getInstance();
    private Controls drivingGP;
    private Controls utilityGP;


    private TurretAligner turretAligner;

    private FtcDashboard dashboard;

    private boolean autoOuttake = false;

    ElapsedTime turretTimer = new ElapsedTime();

    LpsCounter lpsCounter;

    boolean rumbled = false;

    @Override
    public void init() {
        lpsCounter = new LpsCounter();
        lpsCounter.getLoopTime();
        robot.initFollower(hardwareMap, true);
        robot.init(hardwareMap);


        dashboard = FtcDashboard.getInstance();
//        robot.webcam.init(hardwareMap, telemetry);

//        if (robot.webcam.getVisionPortal() != null) {
//            dashboard.startCameraStream(robot.webcam.getVisionPortal(), 30);
//        }

        // Initialize the new coordinate aligner
        turretAligner = new TurretAligner(robot);
        turretAligner.setTarget(RedTowerCoords.x, RedTowerCoords.y);

        drivingGP = new Controls(gamepad1);
        utilityGP = new Controls(gamepad2);

        robot.limelight.init(hardwareMap, telemetry);
    }

    @Override
    public void init_loop() {
        lpsCounter.getLoopTime();



        telemetry.addLine("Initialization Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.onStart();
        robot.follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Update Loops/s delta
        lpsCounter.getLoopTime();

        //Update controller inputs
        drivingGP.update();
        utilityGP.update();

        robot.follower.update();

        if(drivingGP.leftStick.button.justPressed()) {
            double heading = robot.heading.get();
            if(heading < (Math.PI / 4) && heading > (-Math.PI / 4)) {
                heading = 0;
            } else if(heading < (3 * Math.PI / 4) && heading > (Math.PI / 4)) {
                heading = Math.PI / 2;
            } else if(heading > (-3 * Math.PI / 4) && heading < (-Math.PI / 4)) {
                heading = -Math.PI / 2;
            } else {
                heading = Math.PI;
            }

            if(robot.follower.getPose().getX() > 72) {
                robot.follower.setPose(new Pose(135, 9, heading));
            } else {
                robot.follower.setPose(new Pose(9, 9, heading));
            }

            robot.turret.driverOffset = 0;
        }

        robot.intake.reversed = true;

        if(drivingGP.rightStick.button.justPressed()) {
            robot.blueTarget = !robot.blueTarget;
            robot.limelight.switchPipeline(robot.blueTarget);
        }

        //Loader
        if (!drivingGP.rightBumper.pressed()) {
            robot.loader.speed = (drivingGP.leftTrigger - drivingGP.rightTrigger);
            robot.intake.speed = -(drivingGP.leftTrigger - drivingGP.rightTrigger);
            robot.flap.open = false;
        } else {
            robot.loader.speed = (drivingGP.leftTrigger - drivingGP.rightTrigger) * 0.8;
            robot.flap.open = true;

            if (robot.loader.speed > 0.1)
                robot.intake.speed = INTAKE_DRIVER_REVERSE;   // swapped
            else if (robot.loader.speed < -0.2)
                robot.intake.speed = INTAKE_DRIVER_POWER;     // swapped
            else
                robot.intake.speed = 0;
        }



        //Turret/Angle aiming

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

        if(drivingGP.dpadDown.justPressed())
            robot.turret.autoAimEnabled = !robot.turret.autoAimEnabled;

        if(drivingGP.dpadUp.justPressed()) {
            autoOuttake = !autoOuttake;
            robot.outtake.on = autoOuttake;
        }

        if(robot.turret.autoAimEnabled) {
            if(robot.blueTarget) {
                robot.outtake.setRangeOfDistance(robot.follower.getPose().distanceFrom(new Pose(BlueTowerCoords.x, BlueTowerCoords.y)));
            } else {
                robot.outtake.setRangeOfDistance(robot.follower.getPose().distanceFrom(new Pose(RedTowerCoords.x, RedTowerCoords.y)));
            }
        } else {
            if (drivingGP.triangle.justPressed())
                robot.shoot.activateRange(1);
            if (drivingGP.square.justPressed())
                robot.shoot.activateRange(2);
            if (drivingGP.cross.justPressed())
                robot.shoot.activateRange(3);
            if (drivingGP.circle.justPressed())
                robot.shoot.activateRange(4);
        }

        if (robot.outtake.on &&
                robot.leftOuttake.getVelocity() >= robot.outtake.activeConfig.velocity - 30 &&
                robot.leftOuttake.getVelocity() <= robot.outtake.activeConfig.velocity + 90) {
            gamepad1.rumble(1, 0, 150);
            rumbled = true;
        }

        if (!robot.turret.autoAimEnabled && drivingGP.leftBumper.justPressed()) {
            if (robot.outtake.on) {
                robot.shoot.deactivate();
                gamepad1.rumble(1, 1, 100);
                rumbled = false;
            }
        }


        //Update robot systems status
        robot.follower.setTeleOpDrive(-drivingGP.leftStick.y, -drivingGP.leftStick.x, -drivingGP.rightStick.x, true);
        robot.updateAllSystems();
        _telemetry();
        //robot.webcam.update();
    }


    @Override
    public void stop() {
        robot.limelight.stop();
        robot.webcam.stop();
        Pose finalPose = robot.follower.getPose();
        PoseStorage.savePose(finalPose);
    }

    public void _telemetry() {
        telemetry.addData("LPS", "%.1f", 1 / lpsCounter.delta);
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
        robot.limelight.telemetry();
        telemetry.update();
    }
}
