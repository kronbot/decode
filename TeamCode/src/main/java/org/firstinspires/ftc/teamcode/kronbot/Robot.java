package org.firstinspires.ftc.teamcode.kronbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kronbot.utils.detection.AprilTagWebcam;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ANGLE_SERVO_CLOSE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ANGLE_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ANGLE_SERVO_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.DELTA_THRESHOLD;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.FLAP_CLOSED;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.FLAP_OPEN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.OUT_MOTOR_KD;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.OUT_MOTOR_KF;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.OUT_MOTOR_KI;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.OUT_MOTOR_KP;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_1_ANGLE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_1_KS;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_1_VELOCITY;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_2_ANGLE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_2_KS;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_2_VELOCITY;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_3_ANGLE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_3_KS;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_3_VELOCITY;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_4_ANGLE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_4_KS;
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
    public final Heading heading;



    // Private constructor
    public Robot() {
        this.outtake = new Outtake();
        this.intake = new Intake();
        this.loader = new Loader();
        this.turret = new Turret();
        this.shoot = new Shoot();
        this.flap = new Flap();
        this.heading = new Heading();
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
        super.initHardware(hardwareMap);
        initSystems(hardwareMap);
    }

    public void initSystems(HardwareMap hardwareMap) {
        if(follower == null)
            initFollower(hardwareMap);
        follower.update();
        outtake.init();
        intake.init();
        loader.init();
        turret.init();
        flap.init();
        heading.init();

        //Add other inits here

    }

    // Updates all systems
    // Except pedro
    public void updateAllSystems() {
        outtake.update();
        intake.update();
        loader.update();
        turret.update();
        flap.update();

        double rawHeading = follower.getHeading();
        heading.update(rawHeading);
        double filtered = heading.get();
        /**
         eg usage for turret calculations:
         double robotRelativeAngle = fieldRelativeAngle - filtered;
         */


//        gyroscope.updateOrientation();

        //Add other updates here
//        webcam.update();
    }

    public class Outtake {
        public boolean on = false;
        public double angle = 0;
        public double autoAimAngle = 0;
        public double velocity;
        public double kS = 0;
        public boolean reversed = false;

        boolean braking = false;
        double lastVelocity = 0;

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

            if(on){
                leftOuttake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightOuttake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if(leftOuttake.getVelocity() < velocity * 1) {
                    leftOuttake.setPower(1);
                    rightOuttake.setPower(1);
                }
                else if(leftOuttake.getVelocity() > velocity * 1.1) {
                    if(braking) {
                        leftOuttake.setPower(0);
                        rightOuttake.setPower(0);
                    }
                    leftOuttake.setPower(kS * 0.8);
                    rightOuttake.setPower(kS * 0.8);
                }
                else {
                    braking = false;
                    leftOuttake.setPower(kS);
                    rightOuttake.setPower(kS);
                }

            } else {
                if(leftOuttake.getVelocity() < 21) {
                    leftOuttake.setPower(0);
                    rightOuttake.setPower(0);

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
                }
                else if(leftOuttake.getVelocity() < 200) {
                    leftOuttake.setPower(-0.15);
                    rightOuttake.setPower(-0.15);
                }
                else if(leftOuttake.getVelocity() < 300) {
                    leftOuttake.setPower(-0.1);
                    rightOuttake.setPower(-0.1);
                }
                else if(leftOuttake.getVelocity() < 500) {
                    leftOuttake.setPower(0.05);
                    rightOuttake.setPower(0.05);
                }
                else {
                    leftOuttake.setPower(0);
                    rightOuttake.setPower(0);
                }
                /*
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
                 */
            }

            angleServo.setPosition(Math.min(Math.max(angle, ANGLE_SERVO_MIN), ANGLE_SERVO_MAX));
        }

        public void telemetry(Telemetry telemetry) {
            telemetry.addLine("=== OUTTAKE STATUS ===");
            telemetry.addData("On", on);
            telemetry.addData("Reversed", reversed);
            telemetry.addData("Target Velocity", "%.0f", velocity);
            telemetry.addData("Left Velocity", "%.0f", leftOuttake.getVelocity());
            telemetry.addData("Left Power", "%.3f", leftOuttake.getPower());
            telemetry.addData("Right Velocity", "%.0f", rightOuttake.getVelocity());
            telemetry.addData("Right Power", "%.3f", rightOuttake.getPower());
            telemetry.addData("Angle", "%.3f", angle);
            telemetry.addData("Angle Servo Pos", "%.3f", angleServo.getPosition());
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

        public void init() {
            loaderMotor.setPower(0);
        }

        public void update() {
            loaderMotor.setPower(speed);
        }

        public void telemetry(Telemetry telemetry) {
            telemetry.addLine("=== LOADER STATUS ===");
            telemetry.addData("Speed", speed);
        }

    }

    public class Turret {
        /** Angle in radians from straight ahead */
        public double angle = 0;
        public double driverOffset = 0;
        private double servoPosition;

        public void init() {
            angle = 0;
            servoPosition = 0.5;
        }

        public void update() {
            if (turretServo != null && follower != null) {
                if(angle > Math.PI)
                    angle = -2 * Math.PI + angle;
                if(angle < -Math.PI)
                    angle = 2 * Math.PI + angle;

                if(driverOffset > Math.PI)
                    driverOffset = -2 * Math.PI + driverOffset;
                if(driverOffset < -Math.PI)
                    driverOffset = 2 * Math.PI + driverOffset;

                double fieldRelativeAngle = angle + driverOffset;


                if(fieldRelativeAngle > Math.PI)
                    fieldRelativeAngle = -2 * Math.PI + fieldRelativeAngle;
                if(fieldRelativeAngle < -Math.PI)
                    fieldRelativeAngle = 2 * Math.PI + fieldRelativeAngle;

                double robotRelativeAngle = fieldRelativeAngle - (follower.getHeading()/* * 0.01745329*/); // deg to radian
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


    public class Heading {

        private double lastRawHeading = 0.0;       // last reading from PinPoint
        private double lastFilteredRate = 0.0;     // filtered angular velocity
        private double filteredHeading = 0.0;      // output heading
        private final double lpAlpha = 0.2;        // smoothing factor, (0.05 - 0.2 for tuning?)
        private final double loopDt = 0.02;        // control loop period (sec)


        /** Prevents IMU from PinPoint from drifting
         * Calculates the angular velocity (heading / time) from the PinPoint IMU
         * Applies a low-pass filter to the velocity — tiny changes caused by vibration we believe?
         * Integrates the filtered angular velocity back into a stable heading
         */

        public void init(){
            filteredHeading = 0;
        }

        public void update (double rawHeading) {
            //the raw angular vel
            double delta = rawHeading - lastRawHeading;
//
//            //delta for PI
//            delta = Math.atan2(Math.sin(delta), Math.cos(delta));
//
//            double rawRate = delta / loopDt;
//
//            //the low pass filter
//            double filteredRate = lpAlpha * rawRate + (1 - lpAlpha) * lastFilteredRate;
//
//            //integrate back to get filtered heading
//            filteredHeading += filteredRate * loopDt;
                //save for next iteration
                //  lastFilteredRate = filteredRate;

            //simple threshold filter
            //if delta larger than threshold, we add it to our filtered heading, if not, we ignore it
            if(delta>DELTA_THRESHOLD || delta<-DELTA_THRESHOLD) {
                //wrap filtered heading -PI..PI
//                filteredHeading = Math.atan2(Math.sin(filteredHeading), Math.cos(filteredHeading));
                filteredHeading += delta;
            }
            lastRawHeading = rawHeading;
        }
        /**
         *@return Returns the current filtered heading
         */
        public double get() {
            return filteredHeading;
        }

        public void telemetry(Telemetry telemetry) {
            telemetry.addLine("=== HEADING STATUS ===");
            telemetry.addData("Raw Heading", "%.4f", lastRawHeading);
            telemetry.addData("Filtered Heading", "%.4f", filteredHeading);
            telemetry.addData("Filtered Rate", "%.4f rad/s", lastFilteredRate);
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
                    outtake.kS = RANGE_1_KS;
                    //if(outtake.velocity>=RANGE_1_VELOCITY-100)
                    //   gamepad.rumble(1, 0, 100);
                    break;
                case 2:
                    outtake.on = true;
                    outtake.velocity = RANGE_2_VELOCITY;
                    outtake.angle = RANGE_2_ANGLE;
                    outtake.kS = RANGE_2_KS;
                    //if(outtake.velocity>=RANGE_2_VELOCITY-100)
                    //    gamepad.rumble(1, 0, 100);
                    break;
                case 3:
                    outtake.on = true;
                    outtake.velocity = RANGE_3_VELOCITY;
                    outtake.angle = RANGE_3_ANGLE;
                    outtake.kS = RANGE_3_KS;
                    //if(outtake.velocity>=RANGE_3_VELOCITY-100)
                    //    gamepad.rumble(1, 0, 100);
                    break;
                case 4:
                    outtake.on = true;
                    outtake.velocity = RANGE_4_VELOCITY;
                    outtake.angle = RANGE_4_ANGLE;
                    outtake.kS = RANGE_4_KS;
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
        }
    }
}