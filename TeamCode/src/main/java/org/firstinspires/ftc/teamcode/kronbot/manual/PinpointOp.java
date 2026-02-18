package org.firstinspires.ftc.teamcode.kronbot.manual;

import static org.firstinspires.ftc.teamcode.kronbot.autonomous.AutonomousConstants.coordinates;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TestPoseStart;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kronbot.Robot;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.Controls;
import org.firstinspires.ftc.teamcode.kronbot.utils.PoseStorage;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.RobotCentricDrive;

/**
 * This is a TeleOP program for testing stuff with pinpoint for auto shooting.
 *
 * @version 1.0
 */
@TeleOp(name = "Pinpoint Driving", group = Constants.MAIN_GROUP)
public class PinpointOp extends OpMode {
    private final Robot robot = Robot.getInstance();
    private Controls drivingGP;
    private Controls utilityGP;
    private RobotCentricDrive robotCentricDrive;
    private FieldCentricDrive fieldCentricDrive;

    private boolean reverseMovement = false;
    private boolean drivingMode = false;

    //set the pose that the robot got from the end of auto

    @Override
    public void init() {
        robot.init(hardwareMap);
        drivingGP = new Controls(gamepad1);
        utilityGP = new Controls(gamepad2);

        // create pedro follower with pinpoint localizer
        robot.initAutonomy(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    @Override
    public void loop() {
        //update controls
        drivingGP.update();
        utilityGP.update();

        //update robot systems
        movement();
        robot.updateAllSystems();
        updateTelemetry();
    }

    private void movement() {
        if (drivingGP.rightStick.button.shortPressed())
            reverseMovement = !reverseMovement;

        // if(drivingGP.circle.longPressed())
        //     drivingMode = !drivingMode;

        robotCentricDrive.setReverse(reverseMovement);
        if (!drivingMode) {
            robotCentricDrive.run();
            robotCentricDrive.telemetry(telemetry);
        } else {
            fieldCentricDrive.run();
            fieldCentricDrive.telemetry(telemetry);
        }
    }

    private void updateTelemetry() {
        Pose currentPose = robot.follower.getPose();

        telemetry.addLine("=== Pinpoint Localization ===");
        telemetry.addData("X (in)", "%.2f", currentPose.getX());
        telemetry.addData("Y (in)", "%.2f", currentPose.getY());
        telemetry.addData("Heading (rad)", "%.1f", currentPose.getHeading());

        telemetry.update();
    }

    @Override
    public void stop() {
        // may have to add stuff here
    }
}