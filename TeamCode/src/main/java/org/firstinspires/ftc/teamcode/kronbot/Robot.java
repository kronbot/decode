package org.firstinspires.ftc.teamcode.kronbot;

import com.qualcomm.robotcore.hardware.HardwareMap;
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
    
    /** Get the singleton instance */
    public static Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        return instance;
    }
    
    /** Initialize robot and all systems */
    public void init(HardwareMap hardwareMap) {
        super.init(hardwareMap);
        initSystems();
    }
    
    public void initSystems() {
        outtake.init();
        intake.init();
        loader.init();
        turret.init();
    }
    
    /** Updates all systems */
    public void updateAllSystems() {
        outtake.update();
        intake.update();
        loader.update();
        turret.update();
    }

    public static class Outtake {
        public boolean on = false;
        public float dist = 0;
        public double angle = 0;
        public boolean reversed = false;

        public void init() {
            on = false;
            reversed = false;
        }

        public void update(){
            if(on){

            }
        }

    }

    public class Intake {
        public boolean on = false, reversed = false;
        public double power = 1.0;

        public void init() {
            on = false;
            power = 0;
            intakeMotor.setPower(0);
        }

        public void update() {
            if (intakeMotor != null)
                intakeMotor.setPower(on ? (reversed ? power * -1 : power) : 0);
        }
    }

    public class Loader {
        /** A value from -1 to 1 to control a Continuous Servo, with 0 being off */
        public double speed = 0;

        /** If true, the servo will spin reversed to how it normally would.<br>
         * When false the servo will spin clockwise when speed is 1.
         * When true, the servo will spin counterclockwise when speed is 1.
         */
        public boolean reversed = false;

        public void init() {
            loaderServo.setPosition(0.5);
        }

        public void update() {
            if (loaderServo != null) { // Este necesar daca este accesat deja in init fara a verifica sa nu fie null?
                loaderServo.setPosition((speed + 1) / 2);
            }

        }

    }

    public class Turret {
        private double turretPosition = 0.5;
        private double anglePosition = 0.5;

        public void init() {
            turretPosition = 0.5;
            anglePosition = 0.5;
        }

        public void update() {
            if (turretServo != null) {
                turretServo.setPosition(turretPosition);
            }
            if (angleServo != null) {
                angleServo.setPosition(anglePosition);
            }
        }
    }

    public class Wheels{

        public void update(){

        }
    }
}
