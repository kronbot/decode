package org.firstinspires.ftc.teamcode.kronbot.manual;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ANGLE_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ANGLE_SERVO_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.OUT_MOTOR_KD;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.OUT_MOTOR_KF;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.OUT_MOTOR_KI;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.OUT_MOTOR_KP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kronbot.Robot;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.Controls;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.AutoAim;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.RobotCentricDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * The main TeleOP program for the driving period of the game.
 *
 * @version 1.0
 */
@TeleOp(name = "Shooting Testing", group = Constants.MAIN_GROUP)
public class OuttakeShootingTesting extends OpMode {
    private final Robot robot = Robot.getInstance();
    private  Controls drivingGP;
    private  Controls utilityGP;

    double shooterVel = 0;
    double anglePos = 0;


    @Override
    public void init(){
        robot.init(hardwareMap);
        robot.loader.reversed = true;

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

    }

    @Override
    public void loop(){
        //Update controller inputs
        drivingGP.update();
        utilityGP.update();

        robot.shooterMotor.setVelocityPIDFCoefficients(
                OUT_MOTOR_KP,   // P - main stabilizer
                OUT_MOTOR_KI,   // I - usually 0
                OUT_MOTOR_KD,   // D - reduces overshoot
                OUT_MOTOR_KF    // F - feedforward (VERY important)
        );


        //Loader
        robot.loader.speed = drivingGP.rightTrigger - drivingGP.leftTrigger;
        if(robot.loader.speed < -0.25)
            robot.intake.reversed = true;
        else
            robot.intake.reversed = false;

        telemetry.addData("Ultrasonic Distance", "%.0f", robot.rangeSensor.cmUltrasonic());

        anglePos += -drivingGP.rightStick.y * 0.01;
        anglePos = Math.max(Math.min(anglePos, ANGLE_SERVO_MAX), ANGLE_SERVO_MIN);
        shooterVel += drivingGP.leftStick.y * 5;
        shooterVel = Math.max(Math.min(shooterVel, 2000), 0);
        robot.turret.angle += drivingGP.leftStick.x * 0.05;

        telemetry.addData("Shooter Angle Pos", "%.4f", anglePos);
        telemetry.addData("Shooter Velocity", "%.0f", shooterVel);
        telemetry.addData("Turret Angle", "%.4f", robot.turret.angle);

        robot.outtake.on = true;
        robot.outtake.velocity = shooterVel;
        robot.outtake.angle = anglePos;



        //Update robot systems status
        robot.updateAllSystems();
        _telemetry();
    }


    @Override
    public void stop(){
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