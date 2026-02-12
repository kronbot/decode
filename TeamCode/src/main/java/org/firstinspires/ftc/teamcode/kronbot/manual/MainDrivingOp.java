package org.firstinspires.ftc.teamcode.kronbot.manual;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_1_VELOCITY;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_2_VELOCITY;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_3_VELOCITY;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_4_VELOCITY;

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
    private AutoAim autoAim;

    private RobotCentricDrive robotCentricDrive;
    private FieldCentricDrive fieldCentricDrive;
    private FtcDashboard dashboard;

    private boolean autoAimEnabled = false;

    ElapsedTime turretTimer = new ElapsedTime();

    boolean rumbled = false;
    double targetVel = -1;

    @Override
    public void init(){
        robot.init(hardwareMap);
        robot.loader.reversed = true;

        dashboard = FtcDashboard.getInstance();

        robot.webcam.init(hardwareMap, telemetry);
        if (robot.webcam.getVisionPortal() != null) {
            dashboard.startCameraStream(robot.webcam.getVisionPortal(), 30);
        }

        autoAim = new AutoAim(robot, 0.5, 1);

        drivingGP = new Controls(gamepad1);
        utilityGP = new Controls(gamepad2);
    }

    @Override
    public void init_loop(){
        telemetry.addLine("Initialization Ready");
        telemetry.update();
    }

    @Override
    public void start(){
        robotCentricDrive = new RobotCentricDrive(robot, gamepad1);
        fieldCentricDrive = new FieldCentricDrive(robot, gamepad1);
    }

    @Override
    public void loop(){
        //Update controller inputs
        drivingGP.update();
        utilityGP.update();

        //Intake
        robot.intake.speed = utilityGP.rightStick.y;


        //Loader
        if(!drivingGP.rightBumper.pressed()) {
            robot.loader.speed = utilityGP.leftStick.y;
            robot.flap.open = false;
        }
        else {
            robot.loader.speed = drivingGP.rightTrigger - drivingGP.leftTrigger;
            robot.flap.open = true;
            if(robot.loader.speed > 0.1)
                robot.intake.speed = 0.2;
            else if(robot.loader.speed < -0.2)
                robot.intake.speed = -0.1;
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
                robot.turret.angle += increment;

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

                robot.turret.angle -= decrement;
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
            targetVel = RANGE_1_VELOCITY;
        }
        if(drivingGP.square.justPressed()) {
            robot.shoot.activateRange(2, gamepad1);
            targetVel = RANGE_2_VELOCITY;
        }
        if (drivingGP.cross.justPressed()) {
            robot.shoot.activateRange(3, gamepad1);
            targetVel = RANGE_3_VELOCITY;
        }
        if(drivingGP.circle.justPressed()) {
            robot.shoot.activateRange(4, gamepad1);
            targetVel = RANGE_4_VELOCITY;
        }

        if(targetVel > 0 && robot.leftOuttake.getVelocity() >= targetVel - 50)
        {
            gamepad1.rumble(1, 0, 250);
            rumbled = true;
        }

        if(!autoAimEnabled && drivingGP.leftBumper.justPressed()) {
            if(robot.outtake.on) {
                robot.shoot.deactivate();
                gamepad1.rumble(1, 1, 100);
                targetVel = -1;
                rumbled = false;
            }
        }

        //Update robot systems status
        movement();
        robot.updateAllSystems();
        _telemetry();
        //robot.webcam.update();
    }

    private boolean reverseMovement=false, drivingMode=false;//False for robot centric, true for field centric
    private void movement(){
        if(drivingGP.rightStick.button.shortPressed())
            reverseMovement = !reverseMovement;

//        if(drivingGP.circle.longPressed())
//            drivingMode = !drivingMode;

        robotCentricDrive.setReverse(reverseMovement);
        if (!drivingMode) {
            robotCentricDrive.run();
            robotCentricDrive.telemetry(telemetry);
        } else {
            fieldCentricDrive.run();
            fieldCentricDrive.telemetry(telemetry);
        }

    }

    @Override
    public void stop(){
        robot.webcam.stop();
    }

    public void _telemetry(){
        telemetry.addData("shooter motor vel:", robot.leftOuttake.getVelocity());
        telemetry.addData("angle servo pos:", robot.turretServo.getPosition());

        robot.intake.telemetry(telemetry);
        robot.loader.telemetry(telemetry);
        robot.outtake.telemetry(telemetry);
//        robot.turret.telemetry(telemetry);
        drivingGP.telemetry(telemetry);
        telemetry.update();
    }
}