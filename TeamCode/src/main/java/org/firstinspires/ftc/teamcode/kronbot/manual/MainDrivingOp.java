package org.firstinspires.ftc.teamcode.kronbot.manual;

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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kronbot.Robot;
import org.firstinspires.ftc.teamcode.kronbot.utils.Controls;
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
    private  Controls drivingGP;
    private  Controls utilityGP;


    private TurretAligner turretAligner;

    private FtcDashboard dashboard;

    private boolean autoAimEnabled = false;

    ElapsedTime turretTimer = new ElapsedTime();

    LpsCounter lpsCounter;

    boolean rumbled = false;

    @Override
    public void init(){
        lpsCounter = new LpsCounter();
        lpsCounter.getLoopTime();
        robot.initFollower(hardwareMap);
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
    }

    @Override
    public void init_loop(){
        lpsCounter.getLoopTime();
        telemetry.addLine("Initialization Ready");
        telemetry.update();
    }

    @Override
    public void start(){
        robot.follower.startTeleopDrive();
    }

    @Override
    public void loop(){
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
        if(!drivingGP.rightBumper.pressed()) {
            robot.loader.speed = utilityGP.leftStick.y;
            robot.flap.open = false;
        }
        else {
            robot.loader.speed = drivingGP.rightTrigger - drivingGP.leftTrigger;
            robot.flap.open = true;
            if(robot.loader.speed > 0.1)
                robot.intake.speed = INTAKE_DRIVER_POWER;
            else if(robot.loader.speed < -0.2)
                robot.intake.speed = INTAKE_DRIVER_REVERSE;
            else
                robot.intake.speed = 0;
        }

        //AprilTagDetection tag = robot.webcam.getTowerTags();
        //autoAim.telemetry(telemetry, tag);

        //Turret/Angle aiming
        if(autoAimEnabled){
            //To do
//            robot.webcam.update();

            //robot.turret.angle = autoAim.calculateServoPosition(tag);

        } else {
            //Turret aiming
            if(drivingGP.dpadLeft.pressed()) {

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

            }

            else if(drivingGP.dpadRight.pressed()) {

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
            if(drivingGP.dpadUp.pressed())
                robot.outtake.angle += 0.01;
            else if(drivingGP.dpadDown.pressed())
                robot.outtake.angle -= 0.01;
        }

        //Shoot Close/Far
        if (drivingGP.triangle.justPressed()) {
            robot.shoot.activateRange(1, gamepad1);
        }
        if(drivingGP.square.justPressed()) {
            robot.shoot.activateRange(2, gamepad1);
        }
        if (drivingGP.cross.justPressed()) {
            robot.shoot.activateRange(3, gamepad1);
        }
        if(drivingGP.circle.justPressed()) {
            robot.shoot.activateRange(4, gamepad1);
        }

        if( robot.outtake.on &&
                robot.leftOuttake.getVelocity() >= robot.outtake.velocity - 30 &&
                robot.leftOuttake.getVelocity() <= robot.outtake.velocity + 90 )
        {
            gamepad1.rumble(1, 0, 150);
            rumbled = true;
        }

        if(!autoAimEnabled && drivingGP.leftBumper.justPressed()) {
            if(robot.outtake.on) {
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
    public void stop(){
        robot.webcam.stop();
    }

    public void _telemetry(){
        telemetry.addData("LPS", "%.1f", 1 / lpsCounter.delta);
        telemetry.addData("Heading", robot.follower.getHeading());
        telemetry.addData("shooter motor vel:", robot.leftOuttake.getVelocity());
        telemetry.addData("angle servo pos:", robot.turretServo.getPosition());
        telemetry.addData("turret angle:", robot.turret.angle);
        robot.intake.telemetry(telemetry);
        robot.loader.telemetry(telemetry);
        robot.outtake.telemetry(telemetry);
//        robot.turret.telemetry(telemetry);
        drivingGP.telemetry(telemetry);
        telemetry.update();
    }
}