package org.firstinspires.ftc.teamcode.kronbot.manual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;

/**
 * A test TeleOP program for the new Field Centric Driving
 *
 * @version 1.0
 */
//@TeleOp(name = "Field Centric Example", group = Constants.MAIN_GROUP)
public class NewFieldCentricExOp extends LinearOpMode {
    private final KronBot robot = new KronBot();

    FieldCentricDrive fieldCentricDrive;

    Gamepad drivingGamepad;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initHardware(hardwareMap);

        drivingGamepad = gamepad1;

        fieldCentricDrive = new FieldCentricDrive(robot, drivingGamepad);

        fieldCentricDrive.setOrientationOffset(0);

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Initialization Ready");
            telemetry.update();
        }

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {


            fieldCentricDrive.run();
            fieldCentricDrive.telemetry(telemetry);


            telemetry.update();
        }
    }
}