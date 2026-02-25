package org.firstinspires.ftc.teamcode.kronbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.kronbot.utils.detection.AprilTagWebcam;
import org.opencv.core.Mat;

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
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_1;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_1_ANGLE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_1_KS;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_1_VELOCITY;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_2;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_2_ANGLE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_2_KS;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_2_VELOCITY;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_3;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_3_ANGLE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_3_KS;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_3_VELOCITY;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_4;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_4_ANGLE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_4_KS;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_4_VELOCITY;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TURRET_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TURRET_SERVO_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TURRET_SERVO_UNITS_PER_RAD;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.maxVelocity;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.minVelocity;

import android.util.Pair;

import java.util.ArrayList;
import java.util.Dictionary;
import java.util.Enumeration;
import java.util.Map;
import java.util.TreeMap;

public class Robot extends KronBot {
    // Singleton instance

    double dx;
    double dy;
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

    double distance;

    public static class RangeConfig {
        public double angle;
        public double velocity;
        public double kS;

        public RangeConfig(double angle, double velocity, double kS) {
            this.angle = angle;
            this.velocity = velocity;
            this.kS = kS;
        }
    }



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
        double rawHeading = follower.getHeading();
        heading.update(rawHeading);

        outtake.update();
        intake.update();
        loader.update();
        turret.update();
        flap.update();
        follower.update();

        /**
         eg usage for turret calculations:
         double robotRelativeAngle = fieldRelativeAngle - filtered;
         */


//        gyroscope.updateOrientation();

        //Add other updates here
//        webcam.update();
    }

    static final double basket_X = 130;
    static final double basket_Y = 135;


    public class Outtake {
        public boolean on = false;
        public RangeConfig activeConfig;
//        RangeConfig autoAimConfig;
        public boolean reversed = false;

        boolean braking = false;
        private double selectedRange1, selectedRange2;
//        double lastVelocity = 0;
        private TreeMap<Double, RangeConfig> ranges = new TreeMap<>();

        public void init() {
            on = false;
            reversed = false;
            activeConfig = new RangeConfig(0,0,0);

            //Initialize Range based shooter settings
            ranges.put(RANGE_1, new RangeConfig(RANGE_1_ANGLE, RANGE_1_VELOCITY, RANGE_1_KS));
            ranges.put(RANGE_2, new RangeConfig(RANGE_2_ANGLE, RANGE_2_VELOCITY, RANGE_2_KS));
            ranges.put(RANGE_3, new RangeConfig(RANGE_3_ANGLE, RANGE_3_VELOCITY,RANGE_3_KS));
            ranges.put(RANGE_4, new RangeConfig(RANGE_4_ANGLE, RANGE_4_VELOCITY, RANGE_4_KS));
        }



        /** Configures the launch angle and launch motor speed for the given distance.<br>
         *  Returns true if a good configuration is possible (If the distance is in the correct range)
         * @return  Returns true if a configuration is possible
         */
//        public boolean configureDistance(double distance) {
//            if(distance < 20)
//                return false;
//            double shooterVel = 0;
//            double servoAngle = 0;
//
//            // magic numbers for quadratics from desmos
//            if(distance < 46)
//                servoAngle = 0;
//            else if(distance < 150)
//                servoAngle = -0.0000557692 * (distance * distance) + 0.0181423 * distance - 0.716538;
//            else
//                servoAngle = 0.75;
//
//            if(distance < 215) {
//                shooterVel = -0.00460596 * (distance * distance) + 3.42411 * distance + 752.43325;
//            }
//            else if(distance > 250 && distance < 375)
//                shooterVel = 1400; // todo: check if this speed works
//
//            on = true;
//            velocity = shooterVel;
//            angle = servoAngle;
//
//            return true;
//        }


        public void update(){

            if(on){
                leftOuttake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightOuttake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if(leftOuttake.getVelocity() < activeConfig.velocity * 1) {
                    leftOuttake.setPower(1);
                    rightOuttake.setPower(1);
                }
                else if(leftOuttake.getVelocity() > activeConfig.velocity * 1.1) {
                    if(braking) {
                        leftOuttake.setPower(0);
                        rightOuttake.setPower(0);
                    }
                    leftOuttake.setPower(activeConfig.kS * 0.8);
                    rightOuttake.setPower(activeConfig.kS * 0.8);
                }
                else {
                    braking = false;
                    leftOuttake.setPower(activeConfig.kS);
                    rightOuttake.setPower(activeConfig.kS);
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

            angleServo.setPosition(Math.min(Math.max(activeConfig.angle, ANGLE_SERVO_MIN), ANGLE_SERVO_MAX));
        }



        public RangeConfig interpolateRange() {
            double robot_X = follower.getPose().getX();
            double robot_Y = follower.getPose().getY();

            dx = basket_X - robot_X;
            dy = basket_Y - robot_Y;

            distance = Math.sqrt(dx*dx + dy*dy);

            if(distance <= ranges.firstKey())
                return new RangeConfig(ranges.get(ranges.firstKey()).angle, ranges.get(ranges.firstKey()).velocity, ranges.get(ranges.firstKey()).kS);

            if(distance >= ranges.lastKey())
                return new RangeConfig(ranges.get(ranges.lastKey()).angle, ranges.get(ranges.lastKey()).velocity, ranges.get(ranges.lastKey()).kS);

            //I used TreeMaps as its ordered and offers floorEntry (the biggest entry smaller than the value searched) and ceilingEntry (the opposite)
            Map.Entry<Double, RangeConfig> lower = ranges.floorEntry(distance);
            Map.Entry<Double, RangeConfig> upper = ranges.ceilingEntry(distance);

            // Null checks moved before d1/d2 are accessed
            if (lower == null && upper != null) return upper.getValue();
            if (upper == null && lower != null) return lower.getValue();
            if (lower == null) return new RangeConfig(0, 0, 0);

            //Linear interpolation between the two ranges
            double d1 = lower.getKey();
            double d2 = upper.getKey();
            selectedRange1=d1; selectedRange2=d2;


            if (lower.getKey().equals(upper.getKey()))
                return lower.getValue();


            double angle1 = lower.getValue().angle;
            double vel1   = lower.getValue().velocity;
            double kS1 = lower.getValue().kS;

            double angle2 = upper.getValue().angle;
            double vel2   = upper.getValue().velocity;
            double kS2 = upper.getValue().kS;

            double t = (distance - d1) / (d2 - d1);

            double interpAngle = angle1 + (angle2 - angle1) * t;
            double interpVel = vel1 + (vel2   - vel1) * t;
            double interpKs = kS1 + (kS2 - kS1) * t;

            //Clamp manual ca Math.Clamp nu merge cu double
            interpAngle = Math.max(ANGLE_SERVO_MIN, interpAngle);
            interpAngle = Math.min(ANGLE_SERVO_MAX, interpAngle);

            return new RangeConfig(interpAngle, interpVel, interpKs);
        }


        public void telemetry(Telemetry telemetry) {
            telemetry.addLine("=== OUTTAKE STATUS ===");
            telemetry.addData("d1", selectedRange1);
            telemetry.addData("d2", selectedRange2);
            telemetry.addData("Distance", distance);
            telemetry.addData("On", on);
            telemetry.addData("Reversed", reversed);
            telemetry.addData("Target Velocity", "%.0f", activeConfig.velocity);
            telemetry.addData("Left Velocity", "%.0f", leftOuttake.getVelocity());
            telemetry.addData("Left Power", "%.3f", leftOuttake.getPower());
            telemetry.addData("Right Velocity", "%.0f", rightOuttake.getVelocity());
            telemetry.addData("Right Power", "%.3f", rightOuttake.getPower());
            telemetry.addData("Angle", "%.3f", activeConfig.angle);
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


        public boolean autoAimEnabled = true;
        static final double basket_X = 130;
        static final double basket_Y = 135;

        public void init() {
            angle = 0;
            servoPosition = 0.5;
        }


        public void update() {
            //angle to the basket
            if (turretServo == null || follower == null) return;

            if(autoAimEnabled) {

                //Turret angle
                double robot_X = follower.getPose().getX();
                double robot_Y = follower.getPose().getY();
                double robotHeading = heading.get();

                double dx = basket_X - robot_X;
                double dy = basket_Y - robot_Y;

                double targetFieldAngle = Math.atan2(dy, dx);

                //calculate
                double robotRelativeAngle = targetFieldAngle - robotHeading + driverOffset;

                //normalize
                robotRelativeAngle = Math.atan2(
                        Math.sin(robotRelativeAngle),
                        Math.cos(robotRelativeAngle)
                );

                servoPosition = robotRelativeAngle * TURRET_SERVO_UNITS_PER_RAD + 0.5;


            } else {
                servoPosition =
                        driverOffset * TURRET_SERVO_UNITS_PER_RAD + 0.5;
            }

            turretServo.setPosition(
                    Math.clamp(servoPosition, TURRET_SERVO_MIN, TURRET_SERVO_MAX)
            );


//            if (turretServo != null && follower != null) {
//                if(angle > Math.PI)
//                    angle = -2 * Math.PI + angle;
//                if(angle < -Math.PI)
//                    angle = 2 * Math.PI + angle;
//
//                if(driverOffset > Math.PI)
//                    driverOffset = -2 * Math.PI + driverOffset;
//                if(driverOffset < -Math.PI)
//                    driverOffset = 2 * Math.PI + driverOffset;
//
//                double fieldRelativeAngle = angle + driverOffset;
//
//
//                if(fieldRelativeAngle > Math.PI)
//                    fieldRelativeAngle = -2 * Math.PI + fieldRelativeAngle;
//                if(fieldRelativeAngle < -Math.PI)
//                    fieldRelativeAngle = 2 * Math.PI + fieldRelativeAngle;
//
//                double robotRelativeAngle = fieldRelativeAngle - (follower.getHeading()/* * 0.01745329*/); // deg to radian
//                servoPosition = robotRelativeAngle * TURRET_SERVO_UNITS_PER_RAD + 0.5;
//
//
//                turretServo.setPosition(Math.clamp(servoPosition, TURRET_SERVO_MIN, TURRET_SERVO_MAX));
//            }
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
            double delta = rawHeading - lastRawHeading;

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


    public class Shoot {
        public void activateRange(int range) {
            RangeConfig config = new RangeConfig(0,0,0);
            switch (range) {
                case 1:
                    outtake.on = true;
                    config = new RangeConfig(RANGE_1_ANGLE, RANGE_1_VELOCITY, RANGE_1_KS);
                    //if(outtake.velocity>=RANGE_1_VELOCITY-100)
                    //   gamepad.rumble(1, 0, 100);
                    break;
                case 2:
                    outtake.on = true;
                    config = new RangeConfig(RANGE_2_ANGLE, RANGE_2_VELOCITY, RANGE_2_KS);
                    //if(outtake.velocity>=RANGE_2_VELOCITY-100)
                    //    gamepad.rumble(1, 0, 100);
                    break;
                case 3:
                    outtake.on = true;
                    config = new RangeConfig(RANGE_3_ANGLE, RANGE_3_VELOCITY, RANGE_3_KS);
                    //if(outtake.velocity>=RANGE_3_VELOCITY-100)
                    //    gamepad.rumble(1, 0, 100);
                    break;
                case 4:
                    outtake.on = true;
                    config = new RangeConfig(RANGE_4_ANGLE, RANGE_4_VELOCITY, RANGE_4_KS);
                    //if(outtake.velocity>=RANGE_4_VELOCITY-100)
                    //    gamepad.rumble(1, 0, 100);
                    break;
                case 0:
                    outtake.on = true;
                    config = outtake.interpolateRange();
                    break;
            }
            outtake.activeConfig=config;
        }

        public void deactivate() {
            outtake.on = false;
            outtake.activeConfig = new RangeConfig(0,0,0);
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