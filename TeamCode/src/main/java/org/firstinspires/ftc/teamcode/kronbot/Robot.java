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
    }
    
    // Updates all systems
    public void updateAllSystems() {
        outtake.update();
        intake.update();
        loader.update();
        turret.update();
    }

    public class Outtake {
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
        public  boolean on=false, reversed=false;
        private final double stopped = 0.5, forward = 1.0, reverse = 0;

        public void init() {
            loaderServo.setPosition(stopped);
        }

        public void update() {
            if (loaderServo != null)
                if(on)
                    loaderServo.setPosition(reversed ? reverse : forward);
                else
                    loaderServo.setPosition(stopped);
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
