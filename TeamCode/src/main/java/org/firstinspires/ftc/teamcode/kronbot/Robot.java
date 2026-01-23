package org.firstinspires.ftc.teamcode.kronbot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kronbot.utils.detection.AprilTagWebcam;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ANGLE_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ANGLE_SERVO_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LOADER_SERVO_REVERSED;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TURRET_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TURRET_SERVO_MIN;
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
    
    // Private constructor
    public Robot() {
        this.outtake = new Outtake();
        this.intake = new Intake();
        this.loader = new Loader();
        this.turret = new Turret();
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

            angleServo.setPosition(Math.clamp(angle, ANGLE_SERVO_MIN, ANGLE_SERVO_MAX));
        }

        public void telemetry(Telemetry telemetry) {
            telemetry.addLine("=== OUTTAKE STATUS ===");
            telemetry.addData("On", on);
            telemetry.addData("Reversed", reversed);
            telemetry.addData("Target Velocity", "%.0f", velocity);
            telemetry.addData("Actual Velocity", "%.0f", shooterMotor.getVelocity());
            telemetry.addData("Angle", "%.3f", angle);
            telemetry.addData("Angle Servo Pos", "%.3f", angleServo.getPosition());
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
        public  boolean on=false, reversed=false;
        private final double stopped = 0.5, forward = 1.0, reverse = 0;

        public void init() {
            loaderServo.setPosition(stopped);
        }

        public void update() {
                if(on)
                    loaderServo.setPosition(reversed ? reverse : forward);
                else
                    loaderServo.setPosition(stopped);
        }

        public void telemetry(Telemetry telemetry) {
            telemetry.addLine("=== LOADER STATUS ===");
            telemetry.addData("On", on);
            telemetry.addData("Reversed", reversed);
            telemetry.addData("Servo Position", "%.3f", loaderServo.getPosition());
            String state = on ? (reversed ? "Reverse" : "Forward") : "Stopped";
            telemetry.addData("State", state);
        }

    }

    public class Turret {
        public double angle = 0.5;

        public void init() {
            angle = 0.5;
        }

        public void update() {
            if (turretServo != null) {
                turretServo.setPosition(Math.clamp(angle, TURRET_SERVO_MIN, TURRET_SERVO_MAX));
            }
        }

        public void telemetry(Telemetry telemetry) {
            telemetry.addLine("=== TURRET STATUS ===");
            telemetry.addData("Target Angle", "%.3f", angle);
            telemetry.addData("Servo Position", "%.3f", turretServo.getPosition());
            telemetry.addData("Servo Range", "%.3f - %.3f", TURRET_SERVO_MIN, TURRET_SERVO_MAX);
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
