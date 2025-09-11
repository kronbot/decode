package org.firstinspires.ftc.teamcode.kronbot.manual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Button;

/**
 * A test TeleOP program for the new Field Centric Driving
 *
 * @version 1.0
 */
@TeleOp(name = "Field Centric Example", group = Constants.MAIN_GROUP)
public class NewFieldCentricExOp extends LinearOpMode {
    private final KronBot robot = new KronBot();

    FieldCentricDrive fieldCentricDrive;

    Gamepad drivingGamepad;

    Button recalibrateBut = new Button();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initSimpleDriving(hardwareMap);

        drivingGamepad = gamepad1;

        fieldCentricDrive = new FieldCentricDrive(robot, drivingGamepad);


        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Initialization Ready");
            telemetry.update();
        }


        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            recalibrateBut.updateButton(gamepad1.square);
            recalibrateBut.shortPress();

            if(gamepad1.square)
            {
                fieldCentricDrive.calibrateOrientation();
            }

            fieldCentricDrive.run();
            fieldCentricDrive.telemetry(telemetry);


            telemetry.update();
        }
    }
}