package org.firstinspires.ftc.teamcode.kronbot.manual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.FieldCentricDriveAbsolute;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Button;

/**
 * A test TeleOP program for the driving period of the game.
 *
 * @version 1.0
 */
@TeleOp(name = "Simple Driving", group = Constants.MAIN_GROUP)
public class SimpleDrivingOp extends LinearOpMode {
    private final KronBot robot = new KronBot();

    RobotCentricDrive robotCentricDrive;
    FieldCentricDrive fieldCentricDrive;
    FieldCentricDriveAbsolute fieldCentricDriveAbsolute;

    Gamepad drivingGamepad;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initSimpleDriving(hardwareMap);

        drivingGamepad = gamepad1;

        robotCentricDrive = new RobotCentricDrive(robot, drivingGamepad);
        fieldCentricDrive = new FieldCentricDrive(robot, drivingGamepad);
        fieldCentricDriveAbsolute = new FieldCentricDriveAbsolute(robot, drivingGamepad);

        Button driveModeButton = new Button();
        Button reverseButton = new Button();
        Button ungureanuButton = new Button();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Initialization Ready");
            telemetry.update();
        }

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Wheels
            driveModeButton.updateButton(drivingGamepad.square);
            driveModeButton.longPress();

            reverseButton.updateButton(drivingGamepad.circle);
            reverseButton.shortPress();
            robotCentricDrive.setReverse(reverseButton.getShortToggle());

            ungureanuButton.updateButton(drivingGamepad.triangle);
            ungureanuButton.shortPress();

            if (ungureanuButton.getShortToggle()) {
                telemetry.addLine("plang");
                fieldCentricDriveAbsolute.run();
                fieldCentricDriveAbsolute.telemetry(telemetry);
            } else if (!driveModeButton.getLongToggle()) {
                robotCentricDrive.run();
                robotCentricDrive.telemetry(telemetry);
            } else {
                fieldCentricDrive.run();
                fieldCentricDrive.telemetry(telemetry);
            }

            telemetry.update();
        }
    }
}