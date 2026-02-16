package org.firstinspires.ftc.teamcode.kronbot.baseOps;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LOADER_SERVO_REVERSED;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.maxVelocity;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.minVelocity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.detection.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Button;

public class MainTeleOp extends OpMode {

    private final KronBot robot = new KronBot();


    RobotCentricDrive robotCentricDrive;

    Gamepad drivingGamepad;
    Gamepad utilityGamepad;

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private FtcDashboard dashboard;


    boolean isLaunching = false;
    boolean wasRightBumperPressed = false;
    double currentVelocity = 1300;

    public void init()
    {
        robot.initHardware(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        telemetry.update();

        drivingGamepad = gamepad1;
        utilityGamepad = gamepad2;

        robotCentricDrive = new RobotCentricDrive(robot, drivingGamepad);

        aprilTagWebcam.init(hardwareMap, telemetry);
        if (aprilTagWebcam.getVisionPortal() != null) {
            dashboard.startCameraStream(aprilTagWebcam.getVisionPortal(), 30);
        }


    }

    public void init_loop()
    {
        telemetry.addLine("Initialization Ready");
        telemetry.update();
    }

    public void start()
    {

    }

    public void loop()
    {
        //Outtake servo
        if(gamepad1.left_trigger > 0.5) {
            if(!LOADER_SERVO_REVERSED)
                robot.loaderServo.setPosition(0);
            else
                robot.loaderServo.setPosition(1);
        } else {
            double val = gamepad1.right_trigger;
            if(!LOADER_SERVO_REVERSED)
                val = val / 2 + 0.5;
            else
                val = 1 - (val / 2 + 0.5);

            robot.loaderServo.setPosition(val);
        }


        //Outake Wheels
        if (gamepad1.right_bumper && !wasRightBumperPressed) {
            isLaunching = !isLaunching;

            if (isLaunching) {
                currentVelocity = minVelocity;
            }
        }


        //Outake Wheels
        if (gamepad1.right_bumper && !wasRightBumperPressed) {
            isLaunching = !isLaunching;

            if (isLaunching) {
                currentVelocity = minVelocity;
            }
        }

        if (isLaunching) {

            if (gamepad1.dpad_up) {
                currentVelocity = maxVelocity;
            } else if (gamepad1.dpad_down) {
                currentVelocity = minVelocity;
            }

            robot.leftOuttake.setVelocity(currentVelocity);
            robot.rightOuttake.setVelocity(currentVelocity);

        } else {
            robot.leftOuttake.setPower(0);
            robot.rightOuttake.setPower(0);
        }
    }

    public void stop()
    {
        aprilTagWebcam.stop();
    }

}
