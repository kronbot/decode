package org.firstinspires.ftc.teamcode.kronbot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.kronbot.utils.PoseStorage;
import org.firstinspires.ftc.teamcode.kronbot.utils.detection.AprilTagWebcam;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ANGLE_SERVO_CLOSE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ANGLE_SERVO_FAR;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ANGLE_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ANGLE_SERVO_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.FLAP_CLOSED;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.FLAP_OPEN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LOADER_SERVO_REVERSED;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.OUT_MOTOR_KD;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.OUT_MOTOR_KF;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.OUT_MOTOR_KI;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.OUT_MOTOR_KP;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_1_ANGLE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_1_VELOCITY;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_2_ANGLE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_2_VELOCITY;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_3_ANGLE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_3_VELOCITY;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_4_ANGLE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_4_VELOCITY;
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
    public final Flap flap;
    public final Shoot shoot;
    public Follower follower;

    
    // Private constructor
    public Robot() {
        this.outtake = new Outtake();
        this.intake = new Intake();
        this.loader = new Loader();
        this.turret = new Turret();
        this.shoot = new Shoot();
        this.flap = new Flap();
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
        initSystems(hardwareMap);
    }
    
    public void initSystems(HardwareMap hardwareMap) {
        outtake.init();
        intake.init();
        loader.init();
        turret.init();
        flap.init();

        //Add other intis here
        Pose startingPose = PoseStorage.loadPose();
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
    }
    
    // Updates all systems
    public void updateAllSystems() {
        outtake.update();
        intake.update();
        loader.update();
        turret.update();
        flap.update();


//        gyroscope.updateOrientation();

        //Add other updates here
//        webcam.update();
        follower.update();
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

        /** Configures the launch angle and launch motor speed for the given distance.<br>
         *  Returns true if a good configuration is possible (If the distance is in the correct range)
         * @param distance The distance, measured horizontally, from the tower wall to the center of the turret.
         * @return  Returns true if a configuration is possible
         */
        public boolean configureDistance(double distance) {
            if(distance < 20)
                return false;
            double shooterVel = 0;
            double servoAngle = 0;

            // magic numbers for quadratics from desmos
            if(distance < 46)
                servoAngle = 0;
            else if(distance < 150)
                servoAngle = -0.0000557692 * (distance * distance) + 0.0181423 * distance - 0.716538;
            else
                servoAngle = 0.75;

            if(distance < 215) {
                shooterVel = -0.00460596 * (distance * distance) + 3.42411 * distance + 752.43325;
            }
            else if(distance > 250 && distance < 375)
                shooterVel = 1400; // todo: check if this speed works

            on = true;
            velocity = shooterVel;
            angle = servoAngle;

            return true;
        }

        public void update(){

            leftOuttake.setVelocityPIDFCoefficients(
                    OUT_MOTOR_KP,   // P - main stabilizer
                    OUT_MOTOR_KI,   // I - usually 0
                    OUT_MOTOR_KD,   // D - reduces overshoot
                    OUT_MOTOR_KF    // F - feedforward (VERY important)
            );

            rightOuttake.setVelocityPIDFCoefficients(
                    OUT_MOTOR_KP,   // P - main stabilizer
                    OUT_MOTOR_KI,   // I - usually 0
                    OUT_MOTOR_KD,   // D - reduces overshoot
                    OUT_MOTOR_KF    // F - feedforward (VERY important)
            );


            if(on){
                leftOuttake.setPower(1);
                leftOuttake.setVelocity(velocity);
                rightOuttake.setPower(1);
                rightOuttake.setVelocity(velocity);
            } else {
                if(leftOuttake.getVelocity() < 21) {
                    leftOuttake.setPower(0);
                }
                else if(leftOuttake.getVelocity() < 200) {
                    leftOuttake.setPower(-0.15);
                }
                else if(leftOuttake.getVelocity() < 300) {
                    leftOuttake.setPower(-0.1);
                }
                else if(leftOuttake.getVelocity() < 500) {
                    leftOuttake.setPower(0.05);
                }
                else
                    leftOuttake.setPower(0);

                if(rightOuttake.getVelocity() < 21) {
                    rightOuttake.setPower(0);
                }
                else if(rightOuttake.getVelocity() < 200) {
                    rightOuttake.setPower(-0.15);
                }
                else if(rightOuttake.getVelocity() < 300) {
                    rightOuttake.setPower(-0.1);
                }
                else if(rightOuttake.getVelocity() < 500) {
                    rightOuttake.setPower(0.05);
                }
                else
                    rightOuttake.setPower(0);
            }

            angleServo.setPosition(Math.min(Math.max(angle, ANGLE_SERVO_MIN), ANGLE_SERVO_MAX));
        }

        public void telemetry(Telemetry telemetry) {
            telemetry.addLine("=== OUTTAKE STATUS ===");
            telemetry.addData("On", on);
            telemetry.addData("Reversed", reversed);
            telemetry.addData("Target Velocity", "%.0f", velocity);
            telemetry.addData("Actual Velocity", "%.0f", leftOuttake.getVelocity());
            telemetry.addData("Motor Power", "%.3f", leftOuttake.getPower());
            telemetry.addData("Angle", "%.3f", angle);
            telemetry.addData("Angle Servo Pos", "%.3f", angleServo.getPosition());
            telemetry.addData("Distance", "%.3f", rangeSensor.cmUltrasonic() * 1.08644 + 17.20917); // magic numbers from desmos
        }

    }

    public class Intake {
        public boolean reversed = false;
        public double speed = 0;
        public double power = 1.0;

        public void init() {
            speed = 0;
            intakeMotor.setPower(0);
        }

        public void update() {
            if (intakeMotor != null) {
                double output = speed * power;
                if(reversed)
                    output = -output;
                intakeMotor.setPower(output);
            }
        }

        public void telemetry(Telemetry telemetry) {
            telemetry.addLine("=== INTAKE STATUS ===");
            telemetry.addData("Speed", speed);
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
                if(angle > Math.PI)
                    angle = -2 * Math.PI + angle;
                if(angle < -Math.PI)
                    angle = 2 * Math.PI + angle;

                double robotRelativeAngle = angle - (follower.getHeading()/* * 0.01745329*/); // deg to radian
                servoPosition = robotRelativeAngle * TURRET_SERVO_UNITS_PER_RAD + 0.5;


                turretServo.setPosition(Math.clamp(servoPosition, TURRET_SERVO_MIN, TURRET_SERVO_MAX));
            }
        }

        public void telemetry(Telemetry telemetry) {
            telemetry.addLine("=== TURRET STATUS ===");
            telemetry.addData("Target Angle", "%.3f", angle);
            telemetry.addData("Robot Heading", "%.4f", follower.getHeading());
            telemetry.addData("Servo Position", "%.3f", turretServo.getPosition());
            telemetry.addData("Servo Range", "%.3f - %.3f", TURRET_SERVO_MIN, TURRET_SERVO_MAX);
        }
    }

    public class Flap {
        public boolean open = false;

        public void init(){
            flapsServo.setPosition(FLAP_CLOSED);
        }

        public void update(){
            if(open)
                flapsServo.setPosition(FLAP_OPEN);
            else
                flapsServo.setPosition(FLAP_CLOSED);
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
        public void activateRange(int range, Gamepad gamepad) {
            switch (range) {
                case 1:
                    outtake.on = true;
                    outtake.velocity = RANGE_1_VELOCITY;
                    outtake.angle = RANGE_1_ANGLE;
                    //if(outtake.velocity>=RANGE_1_VELOCITY-100)
                    //   gamepad.rumble(1, 0, 100);
                    break;
                case 2:
                    outtake.on = true;
                    outtake.velocity = RANGE_2_VELOCITY;
                    outtake.angle = RANGE_2_ANGLE;
                    //if(outtake.velocity>=RANGE_2_VELOCITY-100)
                    //    gamepad.rumble(1, 0, 100);
                    break;
                case 3:
                    outtake.on = true;
                    outtake.velocity = RANGE_3_VELOCITY;
                    outtake.angle = RANGE_3_ANGLE;
                    //if(outtake.velocity>=RANGE_3_VELOCITY-100)
                    //    gamepad.rumble(1, 0, 100);
                    break;
                case 4:
                    outtake.on = true;
                    outtake.velocity = RANGE_4_VELOCITY;
                    outtake.angle = RANGE_4_ANGLE;
                    //if(outtake.velocity>=RANGE_4_VELOCITY-100)
                    //    gamepad.rumble(1, 0, 100);
            }

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
