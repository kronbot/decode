package org.firstinspires.ftc.teamcode.kronbot.utils.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;

/**
 * Simple test class to sweep 6 servos from 0 → 1 → 0 repeatedly.
 * Runs each servo for 3 seconds, then moves to the next.
 */
@Autonomous(name = "Servo Test", group = Constants.TEST_GROUP)
public class ServoTest extends LinearOpMode {

    private static final int SERVO_COUNT = 6;
    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;
    private static final long TEST_DURATION_MS = 3000;

    @Override
    public void runOpMode() throws InterruptedException {

        Servo[] servos = new Servo[SERVO_COUNT];
        servos[0] = hardwareMap.get(Servo.class, "anglePivot");
//        servos[1] = hardwareMap.get(Servo.class, "servo2");
//        servos[2] = hardwareMap.get(Servo.class, "servo3");
//        servos[3] = hardwareMap.get(Servo.class, "servo4");
//        servos[4] = hardwareMap.get(Servo.class, "servo5");
//        servos[5] = hardwareMap.get(Servo.class, "servo6");

        telemetry.addLine("INITIALIZED");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            for (int i = 0; i < SERVO_COUNT && opModeIsActive(); i++) {
                Servo servo = servos[i];
                telemetry.addData("Testing Servo", "servo" + (i + 1));
                telemetry.update();

                // Sweep from 0 → 1
                servo.setPosition(SERVO_MIN);
                sleep(500);
                long startTime = System.currentTimeMillis();
                while (opModeIsActive() && System.currentTimeMillis() - startTime < TEST_DURATION_MS) {
                    double progress = (System.currentTimeMillis() - startTime) / (double) TEST_DURATION_MS;
                    servo.setPosition(SERVO_MIN + progress * (SERVO_MAX - SERVO_MIN));
                }

                // Sweep from 1 → 0
                startTime = System.currentTimeMillis();
                while (opModeIsActive() && System.currentTimeMillis() - startTime < TEST_DURATION_MS) {
                    double progress = (System.currentTimeMillis() - startTime) / (double) TEST_DURATION_MS;
                    servo.setPosition(SERVO_MAX - progress * (SERVO_MAX - SERVO_MIN));
                }

                // Stop at midpoint
                servo.setPosition(0.5);
                sleep(500);
            }

            telemetry.addLine("Cycle completed");
            telemetry.update();
        }
    }
}
