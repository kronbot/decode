package org.firstinspires.ftc.teamcode.kronbot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.kronbot.utils.detection.AprilTagWebcam;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ANGLE_SERVO_CLOSE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ANGLE_SERVO_FAR;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ANGLE_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ANGLE_SERVO_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LOADER_SERVO_REVERSED;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TURRET_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TURRET_SERVO_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TURRET_SERVO_UNITS_PER_RAD;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.maxVelocity;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.minVelocity;

public class Robot extends KronBot {
    // Singleton instance
    private static Robot instance = null;
    
    // Systems used in all opModes
    public AprilTagWebcam webcam = new AprilTagWebcam();
    public final Outtake outtake;
    public final Intake intake;
    public final Loader loader;
    public final Turret turret;
    public final Shoot shoot;
    
    // Private constructor
    public Robot() {
        this.outtake = new Outtake();
        this.intake = new Intake();
        this.loader = new Loader();
        this.turret = new Turret();
        this.shoot = new Shoot();
    }
    
    // Get the singleton instance
    public static Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        return instance;
    }
    
    // Initialize robot and all systems
    public void init(HardwareMap hardwareMap) {
        super.init(hardwareMap);
        initSystems();
    }
    
    public void initSystems() {
        outtake.init();
        intake.init();
        loader.init();
        turret.init();

        //Add other intis here

    }
    
    // Updates all systems
    public void updateAllSystems() {
        outtake.update();
        intake.update();
        loader.update();
        turret.update();

        gyroscope.updateOrientation();

        //Add other updates here
//        webcam.update();
    }

    public class Outtake {
        public boolean on = false;
        public double angle = 0;
        public double velocity;
        public boolean reversed = false;

        public void init() {
            on = false;
            reversed = false;
            velocity = minVelocity;
        }

        public void update(){
            if(on){
                shooterMotor.setPower(1);
                shooterMotor.setVelocity(velocity);
            } else {
                shooterMotor.setPower(0);
                shooterMotor.setVelocity(0);
            }

            angleServo.setPosition(Math.min(Math.max(angle, ANGLE_SERVO_MIN), ANGLE_SERVO_MAX));
        }

        public void telemetry(Telemetry telemetry) {
            telemetry.addLine("=== OUTTAKE STATUS ===");
            telemetry.addData("On", on);
            telemetry.addData("Reversed", reversed);
            telemetry.addData("Target Velocity", "%.0f", velocity);
            telemetry.addData("Actual Velocity", "%.0f", shooterMotor.getVelocity());
            telemetry.addData("Motor Power", "%.3f", shooterMotor.getPower());
            telemetry.addData("Angle", "%.3f", angle);
            telemetry.addData("Angle Servo Pos", "%.3f", angleServo.getPosition());
            telemetry.addData("Distance", "%.3f", rangeSensor.cmUltrasonic() * 1.08644 + 17.20917); // magic numbers from desmos
        }

    }

    public class Intake {
        public boolean on = false, reversed = false;
        public double power = 1.0;

        public void init() {
            on = false;
            intakeMotor.setPower(0);
        }

        public void update() {
            if (intakeMotor != null)
                intakeMotor.setPower(on ? (reversed ? power * -1 : power) : 0);
        }

        public void telemetry(Telemetry telemetry) {
            telemetry.addLine("=== INTAKE STATUS ===");
            telemetry.addData("On", on);
            telemetry.addData("Reversed", reversed);
            telemetry.addData("Power", "%.2f", power);
            telemetry.addData("Actual Power", "%.2f", intakeMotor.getPower());
        }
    }

    public class Loader {
        public double speed;
        public boolean reversed = false;

        public void init() {
            loaderServo.setPosition(0.5);
        }

        public void update() {
            if(!reversed)
                loaderServo.setPosition((speed + 1) / 2);
            else
                loaderServo.setPosition(1 - ((speed + 1) / 2));
        }

        public void telemetry(Telemetry telemetry) {
            telemetry.addLine("=== LOADER STATUS ===");
            telemetry.addData("Speed", speed);
            telemetry.addData("Reversed", reversed);
            telemetry.addData("Servo Position", "%.3f", loaderServo.getPosition());
        }

    }

    public class Turret {
        /** Angle in radians from straight ahead */
        public double angle = 0;
        private double servoPosition;

        public void init() {
            angle = 0;
            servoPosition = 0.5;
        }

        public void update() {
            if (turretServo != null) {
                double robotRelativeAngle = angle - (gyroscope.getHeading() * 0.01745329); // deg to radian
                servoPosition = robotRelativeAngle * TURRET_SERVO_UNITS_PER_RAD + 0.5;


                turretServo.setPosition(Math.clamp(servoPosition, TURRET_SERVO_MIN, TURRET_SERVO_MAX));
            }
        }

        public void telemetry(Telemetry telemetry) {
            telemetry.addLine("=== TURRET STATUS ===");
            telemetry.addData("Target Angle", "%.3f", angle);
            telemetry.addData("Robot Heading", "%.4f", gyroscope.getHeading());
            telemetry.addData("Servo Position", "%.3f", turretServo.getPosition());
            telemetry.addData("Servo Range", "%.3f - %.3f", TURRET_SERVO_MIN, TURRET_SERVO_MAX);
        }
    }

    public class ShootClose {

        public void activate() {
            // Turn shooter on
            outtake.on = true;

            // Set shooter velocity
            outtake.velocity = minVelocity;

            // Set angle servo
            outtake.angle = ANGLE_SERVO_CLOSE;

            // Set turret position
            //turret.angle = TURRET_SERVO_MIN;
        }

        public void deactivate() {
            outtake.on = false;
        }
    }

    public class ShootFar {

        public void activate() {
            outtake.on = true;
            outtake.velocity = maxVelocity;
            outtake.angle = ANGLE_SERVO_MAX;
            //turret.angle = TURRET_SERVO_MAX;
        }

        public void deactivate() {
            outtake.on = false;
        }
    }

    public class Shoot {
        private boolean lastRange = false;
        public void activateClose() {
            outtake.on = true;
            outtake.velocity = minVelocity;
            outtake.angle = ANGLE_SERVO_CLOSE;

            lastRange = false;
        }
        public void activateFar() {
            outtake.on = true;
            outtake.velocity = maxVelocity;
            outtake.angle = ANGLE_SERVO_FAR;
            //turret.angle = TURRET_SERVO_MAX;

            lastRange = true;
        }
        public void activateLast() {
            if(!lastRange)
                activateClose();
            else
                activateFar();
        }

        public void deactivate() {
            outtake.on = false;
            outtake.velocity = 0;
            outtake.angle = ANGLE_SERVO_MIN;
        }
    }


    public class Wheels{
        public void init(){

        }
        public void update(){

        }

        public void telemetry(Telemetry telemetry) {
            telemetry.addLine("=== WHEELS STATUS ===");
            telemetry.addData("Left Front Power", "%.2f", motors.leftFront.getPower());
            telemetry.addData("Right Front Power", "%.2f", motors.rightFront.getPower());
            telemetry.addData("Left Rear Power", "%.2f", motors.leftRear.getPower());
            telemetry.addData("Right Rear Power", "%.2f", motors.rightRear.getPower());
        }
    }
}
