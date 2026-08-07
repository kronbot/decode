package org.firstinspires.ftc.teamcode.kronbot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.kronbot.utils.detection.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.kronbot.utils.misc.EMA;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ANGLE_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ANGLE_SERVO_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.BLUE_BASKET_X;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.COMMON_BASKET_Y;
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
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RED_BASKET_X;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TURRET_RADS_PER_TICK;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TURRET_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TURRET_SERVO_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TURRET_SERVO_UNITS_PER_RAD;

import java.util.List;
import java.util.Map;
import java.util.TreeMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.util.ElapsedTime;

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
    public final Limelight limelight;

    public boolean blueTarget = false;

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
        this.limelight = new Limelight();
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
//        if(follower == null)
//            initFollower(hardwareMap, true);



//        follower.update();
        outtake.init();
        intake.init();
        loader.init();
        turret.init();
        flap.init();
        heading.init();

        //Add other inits here

    }

    public void onStart() {
        turret.zeroPositionTicks = turretEncoder.getCurrentPosition();
    }

    // Updates all systems
    // Except pedro
    public void updateAllSystems() {
        double rawHeading = follower.getHeading();
        heading.update(rawHeading);
        limelight.update();

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

    public class Limelight {

        private static final int POLL_RATE_HZ = 30;
        private static final int PIPELINE_INDEX = 7;
        private static final long STALE_RESULT_MS = 150;
        private static final int PIPELINE_RED = 8;
        private static final int PIPELINE_BLUE = 7;
        private static final long TARGET_LOST_GRACE_MS = 300;

        private boolean usingBluePipeline = false;

        private Limelight3A limelight;
        private Telemetry telemetry;
        private LLResult result;
        private long lastFreshTargetTimeMs = 0;
        private boolean initialized = false;
        private String lastFault = null;

        // Call this once, from your OpMode's init(), passing in its hardwareMap and telemetry
        public void init(HardwareMap hardwareMap, Telemetry telemetry) {
            this.telemetry = telemetry;
            try {
                limelight = hardwareMap.get(Limelight3A.class, "limelight");
                limelight.setPollRateHz(POLL_RATE_HZ);
                limelight.pipelineSwitch(PIPELINE_RED); // default to red at init
                usingBluePipeline = false;
                limelight.start();
                initialized = true;
                lastFault = null;
            } catch (RuntimeException e) {
                initialized = false;
                lastFault = e.getClass().getSimpleName() + ": " + e.getMessage();
            }
        }

        public void switchPipeline(boolean blue) {
            if (!initialized || limelight == null) return;
            try {
//                limelight.pipelineSwitch(blue ? PIPELINE_RED : PIPELINE_BLUE);
                limelight.pipelineSwitch(blue ? PIPELINE_BLUE : PIPELINE_RED);
                usingBluePipeline = blue;
                lastFault = null;
            } catch (RuntimeException e) {
                lastFault = e.getClass().getSimpleName() + ": " + e.getMessage();
            }
        }

        public void togglePipeline() {
            switchPipeline(!usingBluePipeline);
        }
        public boolean isUsingBluePipeline() {
            return usingBluePipeline;
        }



        // Call this once per loop() BEFORE calling telemetry(), so 'result' is fresh
        public void update() {
            if (!initialized || limelight == null) {
                return;
            }

            try {
                if (!limelight.isConnected()) {
                    lastFault = "Disconnected";
                    result = null;
                    return;
                }

                limelight.updateRobotOrientation(heading.get());
                result = limelight.getLatestResult();
                if (isFreshTarget(result)) {
                    lastFreshTargetTimeMs = System.currentTimeMillis();
                }
                lastFault = null;
            } catch (RuntimeException e) {
                result = null;
                lastFault = e.getClass().getSimpleName() + ": " + e.getMessage();
            }
        }

        public void telemetry() {
            telemetry.addLine("=== LIMELIGHT STATUS ===");

            if (!initialized || limelight == null) {
                telemetry.addData("Limelight", "Not initialized");
                if (lastFault != null) {
                    telemetry.addData("Fault", lastFault);
                }
                return;
            }

            try {
                telemetry.addData("Connected", limelight.isConnected());
                telemetry.addData("Last Update", limelight.getTimeSinceLastUpdate() + " ms");
            } catch (RuntimeException e) {
                lastFault = e.getClass().getSimpleName() + ": " + e.getMessage();
            }

            if (lastFault != null) {
                telemetry.addData("Fault", lastFault);
            }

            if (result == null) {
                telemetry.addData("Limelight", "No data yet");
                return;
            }

            long staleness = result.getStaleness();
            if (staleness > STALE_RESULT_MS) {
                telemetry.addData("Limelight", "Stale data (" + staleness + " ms)");
                return;
            }

            if (result.isValid()) {
                double tx = result.getTx(); // left/right (degrees)
                double ty = result.getTy(); // up/down (degrees)
                double ta = result.getTa(); // target size (0-100%)

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);

                // First, tell Limelight which way your robot is facing
                double robotYaw = heading.get();
                limelight.updateRobotOrientation(robotYaw);
                if (result != null && result.isValid()) {
                    Pose3D botpose_mt2 = result.getBotpose_MT2();
                    if (botpose_mt2 != null) {
                        double x = botpose_mt2.getPosition().x;
                        double y = botpose_mt2.getPosition().y;
                        telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                    }
                }

                Pose3D botpose = result.getBotpose();
                if (botpose != null) {
                    double x = botpose.getPosition().x;
                    double y = botpose.getPosition().y;
                    telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
                }
            } else {
                telemetry.addData("Limelight", "No Targets");
                return;
            }

            List<LLResultTypes.ColorResult> colorTargets = result.getColorResults();
            for (LLResultTypes.ColorResult colorTarget : colorTargets) {
                double x = colorTarget.getTargetXDegrees();
                double y = colorTarget.getTargetYDegrees();
                double area = colorTarget.getTargetArea();
                telemetry.addData("Color Target", "x=" + x + " y=" + y + " area=" + area + "%");
            }

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                double x = fiducial.getTargetXDegrees();
                double y = fiducial.getTargetYDegrees();
                Pose3D poseInTargetSpace = fiducial.getRobotPoseTargetSpace();
                double distance = poseInTargetSpace != null ? poseInTargetSpace.getPosition().y : -1;
                telemetry.addData("Fiducial " + id, "x=" + x + " y=" + y + " dist=" + distance + "m");
            }

            List<LLResultTypes.BarcodeResult> barcodes = result.getBarcodeResults();
            for (LLResultTypes.BarcodeResult barcode : barcodes) {
                String data = barcode.getData();
                String family = barcode.getFamily();
                telemetry.addData("Barcode", data + " (" + family + ")");
            }

            List<LLResultTypes.ClassifierResult> classifications = result.getClassifierResults();
            for (LLResultTypes.ClassifierResult classification : classifications) {
                String className = classification.getClassName();
                double confidence = classification.getConfidence();
                telemetry.addData("I see a", className + " (" + confidence + "%)");
            }

            if (staleness < 100) {
                telemetry.addData("Data", "Good");
            } else {
                telemetry.addData("Data", "Old (" + staleness + " ms)");
            }
        }

        public LLResult getResult() {
            return result;
        }

        public LLResult getFreshResult() {
            return isFreshTarget(result) ? result : null;
        }

        public boolean hasFreshTarget() {
            return getFreshResult() != null;
        }

        public boolean hasRecentTarget() {
            return System.currentTimeMillis() - lastFreshTargetTimeMs <= TARGET_LOST_GRACE_MS;
        }

        public long getTimeSinceFreshTargetMs() {
            return System.currentTimeMillis() - lastFreshTargetTimeMs;
        }

        private boolean isFreshTarget(LLResult result) {
            return result != null && result.isValid() && result.getStaleness() <= STALE_RESULT_MS;
        }

        public void stop() {
            if (limelight != null) {
                try {
                    limelight.stop();
                } catch (RuntimeException e) {
                    lastFault = e.getClass().getSimpleName() + ": " + e.getMessage();
                }
            }
        }
    }

    public class Outtake {
        public boolean on = false;
        public RangeConfig activeConfig;
        //        RangeConfig autoAimConfig;
        public boolean reversed = false;

        boolean braking = false;
        private double selectedRange1, selectedRange2;
        private double distance;
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
        private String aimSource = "Odometry";
        private double limelightTx = 0;
        private double limelightCorrection = 0;
        private boolean limelightTargetActive = false;
        private final ElapsedTime limelightControlTimer = new ElapsedTime();
        public int zeroPositionTicks;

        private EMA headingDriftEMA = EMA.ofPeriod(20);
        private double pinpointTurretAngle = 0;
        private double limelightTurretAngle = 0;
        private double targetBearing = 0;


        public boolean autoAimEnabled = true;

        public void init() {
            angle = 0;
            servoPosition = 0.5;
            resetLimelightController();
            headingDriftEMA.add(0);
        }

        private double getAngle() {
            return (turretEncoder.getCurrentPosition() - zeroPositionTicks) * TURRET_RADS_PER_TICK;
        }


        public void update() {
            //angle to the basket
            if (turretServo == null || follower == null) return;



            if(autoAimEnabled) {
                LLResult limelightResult = limelight.getFreshResult();
                Pose robotPose = follower.getPose().setHeading(heading.get());
                double dx = (blueTarget ? BLUE_BASKET_X : RED_BASKET_X) - robotPose.getX();
                double dy = COMMON_BASKET_Y - robotPose.getY();
                targetBearing = Math.atan2(dy, dx);

                double turretAngle = getAngle();

                pinpointTurretAngle = targetBearing - robotPose.getHeading();


                if (limelightResult != null) {
                    aimSource = "Limelight & Pinpoint";
                    limelightTx = limelightResult.getTx();

                    limelightTurretAngle = turretAngle + Math.toRadians(limelightTx);

                    limelightCorrection = limelightTurretAngle;
                    angle = limelightTurretAngle;

                } else if (limelight.hasRecentTarget()) {
                    aimSource = "Limelight old & Pinpoint";
                } else {
                    aimSource = "Pinpoint";
                    limelightTx = 0;
                    limelightCorrection = 0;

                    angle = pinpointTurretAngle;
                }

            } else {
                aimSource = "Driver Offset";
                limelightTx = 0;
                limelightCorrection = 0;
                resetLimelightController();
                angle = driverOffset;
            }

            angle = Math.atan2(
                    Math.sin(angle),
                    Math.cos(angle)
            );
            servoPosition = Math.min(Math.max(TURRET_SERVO_MIN, angle * TURRET_SERVO_UNITS_PER_RAD + 0.5), TURRET_SERVO_MAX);
            turretServo.setPosition(servoPosition);

        }

        private void resetLimelightController() {
            limelightTargetActive = false;
            limelightControlTimer.reset();
        }

        public void telemetry(Telemetry telemetry) {
            telemetry.addLine("=== TURRET STATUS ===");
            telemetry.addData("Command Angle", "%.3f", angle);
            telemetry.addData("Target Bearing", "%.3f", targetBearing);
            telemetry.addData("Turret Angle", "%.3f", getAngle());
            telemetry.addData("Aim Source", aimSource);
            telemetry.addData("Limelight Target", limelight.hasFreshTarget());
            telemetry.addData("Limelight Tx", "%.2f", limelightTx);
            telemetry.addData("Limelight Correction", "%.4f", limelightCorrection);
            telemetry.addData("Last Limelight Target", limelight.getTimeSinceFreshTargetMs() + " ms");
            telemetry.addData("Robot Heading", "%.4f", follower.getHeading());
            telemetry.addData("Servo Position", "%.3f", turretServo.getPosition());
            telemetry.addData("Servo Range", "%.3f - %.3f", TURRET_SERVO_MIN, TURRET_SERVO_MAX);
            telemetry.addData("Target Coordinates", "%.3f, %.3f", (blueTarget ? BLUE_BASKET_X : RED_BASKET_X), COMMON_BASKET_Y);
            telemetry.addData("PinPoint Angle", "%.3f", pinpointTurretAngle);
            telemetry.addData("LimeLight Angle", "%.3f", limelightTurretAngle);
            telemetry.addData("Angle Error", "%.3f", pinpointTurretAngle - limelightTurretAngle);
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
                    config = new RangeConfig(0, 0, 0);
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