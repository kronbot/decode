package org.firstinspires.ftc.teamcode.kronbot.utils.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;

/**
 * Simple motor test that runs one motor from -1 → 1 → 0.
 * Each phase lasts 3 seconds.
 */
@Autonomous(name = "Motor Test", group = Constants.TEST_GROUP)
public class MotorTest extends LinearOpMode {

    private static final double TEST_POWER = 1.0;
    private static final long TEST_DURATION_MS = 3000;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor motor = hardwareMap.get(DcMotor.class, "motor1");

        telemetry.addLine("INITIALIZED");
        telemetry.update();

        waitForStart();

        telemetry.addLine("Running FORWARD");
        telemetry.update();
        motor.setPower(TEST_POWER);
        sleep(TEST_DURATION_MS);

        telemetry.addLine("Running BACKWARD");
        telemetry.update();
        motor.setPower(-TEST_POWER);
        sleep(TEST_DURATION_MS);

        telemetry.addLine("Stopping motor");
        telemetry.update();
        motor.setPower(0);
        sleep(1000);

        telemetry.addLine("Completed");
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}
