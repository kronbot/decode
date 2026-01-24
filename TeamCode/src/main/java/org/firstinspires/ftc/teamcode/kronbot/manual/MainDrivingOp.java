package org.firstinspires.ftc.teamcode.kronbot.manual;

import com.acmerobotics.dashboard.FtcDashboard;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    boolean manualOuttake = false;
    private boolean outtakeWasOn = false;



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
        if(drivingGP.rightBumper.justPressed())
            robot.intake.on = !robot.intake.on;

        //Loader
        robot.loader.speed = drivingGP.rightTrigger - drivingGP.leftTrigger;
        if(robot.loader.speed < -0.25)
            robot.intake.reversed = true;
        else
            robot.intake.reversed = false;

        //Auto aim toggle
        if(drivingGP.square.shortPressed())
            autoAimEnabled = !autoAimEnabled;


        AprilTagDetection tag = robot.webcam.getTowerTags();
        autoAim.telemetry(telemetry, tag);

        //Turret/Angle aiming
        if(autoAimEnabled){
            //To do
//            robot.webcam.update();

            //robot.turret.angle = autoAim.calculateServoPosition(tag);

        } else {
            //Turret aiming
            if(drivingGP.dpadLeft.pressed())
                robot.turret.angle += 0.01;
            else if(drivingGP.dpadRight.pressed())
                robot.turret.angle -= 0.01;

            //Angle aiming
            if(drivingGP.dpadUp.pressed())
                robot.outtake.angle += 0.01;
            else if(drivingGP.dpadDown.pressed())
                robot.outtake.angle -= 0.01;
        }

        //Shoot Close/Far
        if (!manualOuttake && drivingGP.cross.justPressed()) {
            robot.shootClose.activate();
        }

        if (!manualOuttake && drivingGP.triangle.justPressed()) {
            robot.shootFar.activate();
        }

        //Manual Outtake
        if(!autoAimEnabled && drivingGP.leftBumper.justPressed()) {
            robot.outtake.on = !robot.outtake.on;
            manualOuttake=!manualOuttake;
        }

        //stop whole outtake
        if (drivingGP.circle.justPressed()) {
            robot.outtake.on = false;
        }

        // Short rumble for driver to be sure that outtake has stopped
        if (outtakeWasOn && !robot.outtake.on) {
            gamepad2.rumble(300);
        }
        //update prev state
        outtakeWasOn = robot.outtake.on;



        //Update robot systems status
        movement();
        robot.updateAllSystems();
        _telemetry();
        robot.webcam.update();
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
        telemetry.addData("shooter motor vel:", robot.shooterMotor.getVelocity());
        telemetry.addData("angle servo pos:", robot.turretServo.getPosition());

        robot.intake.telemetry(telemetry);
        robot.loader.telemetry(telemetry);
        robot.outtake.telemetry(telemetry);
        robot.turret.telemetry(telemetry);
        drivingGP.telemetry(telemetry);
        telemetry.update();
    }
}