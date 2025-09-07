package org.firstinspires.ftc.teamcode.kronbot.manual;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LIFT_INIT_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Button;

/**
 * The main TeleOP program for the driving period of the game.
 *
 * @version 1.0
 */
@TeleOp(name = "Main Driving", group = Constants.MAIN_GROUP)
public class MainDrivingOp extends LinearOpMode {
    private final KronBot robot = new KronBot();

    RobotCentricDrive robotCentricDrive;
    FieldCentricDrive fieldCentricDrive;

    Gamepad drivingGamepad;
    Gamepad utilityGamepad;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initTeleop(hardwareMap);

        drivingGamepad = gamepad1;
        utilityGamepad = gamepad2;

        robotCentricDrive = new RobotCentricDrive(robot, drivingGamepad);
        fieldCentricDrive = new FieldCentricDrive(robot, drivingGamepad);

        Button driveModeButton = new Button();
        Button reverseButton = new Button();

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
            if (!driveModeButton.getLongToggle()) {
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