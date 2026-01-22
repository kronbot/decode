package org.firstinspires.ftc.teamcode.kronbot.manual;

import com.acmerobotics.dashboard.FtcDashboard;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kronbot.Robot;
import org.firstinspires.ftc.teamcode.kronbot.utils.Controls;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;

/**
 * The main TeleOP program for the driving period of the game.
 *
 * @version 1.0
 */
@TeleOp(name = "Main Driving after Telea", group = Constants.MAIN_GROUP)
public class MainDrivingOp extends OpMode {
    private final Robot robot = Robot.getInstance();
    private final Controls drivingGP = new Controls(gamepad1);
    private final Controls utilityGP = new Controls(gamepad2);

    private RobotCentricDrive robotCentricDrive;
    //private FieldCentricDrive fieldCentricDrive;

    private FtcDashboard dashboard;

    @Override
    public void init(){
        robot.init(hardwareMap);
        robot.loader.reversed = true;
    }

    @Override
    public void init_loop(){
        telemetry.addLine("Initialization Ready");
        telemetry.update();
    }

    @Override
    public void start(){
        robotCentricDrive = new RobotCentricDrive(robot, gamepad1);
        //fieldCentricDrive = new FieldCentricDrive(robot, gamepad1);
    }

    @Override
    public void loop(){
        //Update controller inputs
        drivingGP.update();
        utilityGP.update();

        //Intake
        if(utilityGP.rightBumper.shortToggle())
            robot.intake.on = !robot.intake.on;

        //Loader
        robot.loader.speed = utilityGP.rightTrigger - utilityGP.leftTrigger;

        //Update robot systems status
        movement();
        robot.updateAllSystems();
    }

    private void movement(){
        robotCentricDrive.run();
        robotCentricDrive.telemetry(telemetry);
        /*
        robotCentricDrive.setReverse(drivingGP.circle.pressed());
        if (!driveModeButton.getLongToggle()) {
            robotCentricDrive.run();
            robotCentricDrive.telemetry(telemetry);
        } else {
            fieldCentricDrive.run();
            fieldCentricDrive.telemetry(telemetry);
        }
        */
    }

    @Override
    public void stop(){
        robot.webcam.stop();
    }

}