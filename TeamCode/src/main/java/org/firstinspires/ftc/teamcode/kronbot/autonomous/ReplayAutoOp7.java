package org.firstinspires.ftc.teamcode.kronbot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.kronbot.Robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Replay Auto V7 — Driver Input Feedforward + Continuous Blending + Proper Constraints
 *
 * ARCHITECTURE:
 *   Feedforward = driver's actual gamepad stick inputs (already in [-1, 1])
 *   Correction  = small PD on pose error (rotated to robot frame)
 *   Blending    = continuous exponential based on pose error magnitude
 *   Battery     = time scaling only (no voltage compensation on power)
 *   Constraints = proper mecanum normalization, slew rate limiting
 *
 * WHY THIS IS THE BEST APPROACH:
 *   - Feedforward is CAUSAL: the driver's intent, not derived from effect
 *   - No derivation noise, no scaling factors, no max velocity calibration
 *   - Identical mecanum mixing path to TeleOp
 *   - Continuous blending is smooth and physically meaningful
 *   - Time scaling naturally compensates battery
 *
 * CSV Format (from DataRecordingOp7):
 *   Time,X,Y,Heading,Voltage,GamepadFwd,GamepadStr,GamepadTurn,IntakePwr,LoaderPwr,LShooterPwr,RShooterPwr,TurretPos,AnglePos,FlapPos
 */
@Autonomous(name = "Replay Auto V7", group = "Replay")
public class ReplayAutoOp7 extends LinearOpMode {

    private static final String CSV_PATH = "/sdcard/robot_data_v7.csv";

    // ========== TUNABLE CONSTANTS ==========

    /** Continuous blending: how fast FF weight drops as error grows.
     *  k_blend = 0.3 means at 3.3cm error, FF weight is ~50%.
     *  Higher = more aggressive correction at small errors.
     *  Lower = more trust in feedforward.
     *  Start with 0.25–0.40. */
    private static final double BLEND_K = 0.30;

    /** Minimum FF weight (even at huge error). Prevents pure PD oscillation. */
    private static final double MIN_FF_WEIGHT = 0.35;

    /** PD gains — LOW because driver input does the heavy lifting. */
    private static final double kP_TRANSLATION = 0.018;
    private static final double kD_TRANSLATION = 0.010;
    private static final double kP_ROTATION = 0.25;
    private static final double kD_ROTATION = 0.06;

    /** Max power the correction term can add. */
    private static final double MAX_CORRECTION = 0.25;

    /** Lookahead: follow a point this many seconds ahead in the recording. */
    private static final double LOOKAHEAD_TIME = 0.08;

    /** Time scaling for battery compensation.
     *  Lower battery → stretch time so robot has more time to execute.
     *  NO velocity scaling — time scaling handles everything. */
    private static final double TIME_SCALE_FACTOR = 0.55;
    private static final double MAX_TIME_SCALE = 1.6;
    private static final double MIN_TIME_SCALE = 0.85;

    /** D-term low-pass filter. */
    private static final double D_FILTER_ALPHA = 0.45;

    /** Slew rate limit: max change in command per second.
     *  Human inputs are naturally smooth, but interpolation + correction
     *  can create discontinuities. 5.0 = full range in 0.4s. */
    private static final double MAX_SLEW_RATE = 5.0;

    /** Voltage sensor refresh rate. */
    private static final double VOLTAGE_REFRESH_SEC = 0.1;

    // ========== MOTOR DIRECTIONS ==========
    // COPY THESE EXACTLY FROM THE TELEMETRY OUTPUT OF DataRecordingOp7
    private static final DcMotorSimple.Direction LF_DIR = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction RF_DIR = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction LR_DIR = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction RR_DIR = DcMotorSimple.Direction.FORWARD;

    // ========== STATE ==========
    private final Robot robot = Robot.getInstance();
    private final ElapsedTime runtime = new ElapsedTime();
    private final List<RobotFrame> recordedFrames = new ArrayList<>();

    private DcMotorEx leftFront, rightFront, leftRear, rightRear;

    // PD state (all in ROBOT FRAME)
    private double prevErrorX = 0, prevErrorY = 0, prevErrorHeading = 0;
    private double prevTime = 0;
    private double filteredDx = 0, filteredDy = 0, filteredDh = 0;

    // Slew rate limiter state
    private double prevRobotFwd = 0, prevRobotStr = 0, prevRobotTurn = 0;

    // Voltage
    private double cachedVoltage = 12.0;
    private double lastVoltageReadTime = -999;
    private double recordedVoltage = 12.0;
    private double timeScale = 1.0;

    // Telemetry stats
    private double maxWheelPowerSeen = 0;
    private int clipCount = 0;
    private double avgFFWeight = 0;
    private int loopCount = 0;

    // -------------------------------------------------------------------------
    // DATA MODEL
    // -------------------------------------------------------------------------
    private static class RobotFrame {
        double timestamp;
        double x, y, heading;
        double voltage;
        double gpFwd, gpStr, gpTurn;   // DRIVER INPUTS in [-1, 1] (robot-frame)
        double intakePwr, loaderPwr, leftShtrPwr, rightShtrPwr;
        double turretPos, anglePos, flapPos;

        RobotFrame(String[] d) {
            timestamp   = Double.parseDouble(d[0]);
            x           = Double.parseDouble(d[1]);
            y           = Double.parseDouble(d[2]);
            heading     = Double.parseDouble(d[3]);
            voltage     = Double.parseDouble(d[4]);
            gpFwd       = Double.parseDouble(d[5]);
            gpStr       = Double.parseDouble(d[6]);
            gpTurn      = Double.parseDouble(d[7]);
            intakePwr   = Double.parseDouble(d[8]);
            loaderPwr   = Double.parseDouble(d[9]);
            leftShtrPwr = Double.parseDouble(d[10]);
            rightShtrPwr= Double.parseDouble(d[11]);
            turretPos   = Double.parseDouble(d[12]);
            anglePos    = Double.parseDouble(d[13]);
            flapPos     = Double.parseDouble(d[14]);
        }
    }

    // -------------------------------------------------------------------------
    // MAIN
    // -------------------------------------------------------------------------
    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing Replay Auto V7...");
        telemetry.update();

        robot.initFollower(hardwareMap, true);
        robot.init(hardwareMap);

        try {
            robot.follower.getPoseTracker().resetIMU();
        } catch (InterruptedException e) {
            telemetry.addLine("IMU Reset Interrupted");
        }

        // Drive motors with verified directions
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear   = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear  = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftFront.setDirection(LF_DIR);
        rightFront.setDirection(RF_DIR);
        leftRear.setDirection(LR_DIR);
        rightRear.setDirection(RR_DIR);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        try {
            loadRecordedData();
            if (recordedFrames.isEmpty()) {
                telemetry.addLine("ERROR: No recorded data found!");
                telemetry.update();
                return;
            }
            calculateRecordedVoltage();
        } catch (Exception e) {
            telemetry.addData("ERROR", e.toString());
            telemetry.update();
            sleep(3000);
            return;
        }

        RobotFrame first = recordedFrames.get(0);
        robot.follower.setPose(new Pose(first.x, first.y, first.heading));

        telemetry.addLine("=== Replay Auto V7 Ready ===");
        telemetry.addData("Frames", recordedFrames.size());
        telemetry.addData("Duration", "%.2f s",
                recordedFrames.get(recordedFrames.size()-1).timestamp - first.timestamp);
        telemetry.addData("Rec Voltage", "%.1f V", recordedVoltage);
        telemetry.addData("Blend K", "%.2f", BLEND_K);
        telemetry.addData("Lookahead", "%.2f s", LOOKAHEAD_TIME);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        executePlayback();

        stopRobot();
        stopMechanisms();
    }

    // -------------------------------------------------------------------------
    // PLAYBACK LOOP
    // -------------------------------------------------------------------------
    private void executePlayback() {
        runtime.reset();

        double startTs = recordedFrames.get(0).timestamp;
        double endTs   = recordedFrames.get(recordedFrames.size() - 1).timestamp;
        double duration = endTs - startTs;

        // Reset state
        prevTime = 0;
        prevErrorX = prevErrorY = prevErrorHeading = 0;
        filteredDx = filteredDy = filteredDh = 0;
        prevRobotFwd = prevRobotStr = prevRobotTurn = 0;
        maxWheelPowerSeen = 0;
        clipCount = 0;
        avgFFWeight = 0;
        loopCount = 0;

        int idx = 0;

        while (opModeIsActive() && idx < recordedFrames.size() - 1) {
            double now = runtime.seconds();

            // Refresh voltage and compute time scaling
            refreshVoltage(now);
            updateTimeScaling();

            // Current position in recording, with lookahead and time scaling
            double recordingTime = (now / timeScale) + LOOKAHEAD_TIME;
            double targetTs = startTs + recordingTime;

            // Advance index to frame just before targetTs
            while (idx < recordedFrames.size() - 1 &&
                    recordedFrames.get(idx + 1).timestamp <= targetTs) {
                idx++;
            }

            // Interpolate between frames
            RobotFrame fA = recordedFrames.get(idx);
            RobotFrame fB = (idx + 1 < recordedFrames.size()) ? recordedFrames.get(idx + 1) : fA;

            double segDur = fB.timestamp - fA.timestamp;
            double t = (segDur > 1e-6)
                    ? Range.clip((targetTs - fA.timestamp) / segDur, 0.0, 1.0)
                    : 0.0;

            // Interpolated target pose
            double targetX = lerp(fA.x, fB.x, t);
            double targetY = lerp(fA.y, fB.y, t);
            double targetH = lerpAngle(fA.heading, fB.heading, t);

            // ===== FEEDFORWARD: DRIVER'S ACTUAL INPUTS =====
            // These are already in [-1, 1] robot-frame power units
            // Interpolated at the SAME target frame as lookahead
            double ffFwd  = lerp(fA.gpFwd,  fB.gpFwd,  t);
            double ffStr  = lerp(fA.gpStr,  fB.gpStr,  t);
            double ffTurn = lerp(fA.gpTurn, fB.gpTurn, t);

            // Clamp to [-1, 1] (should already be, but safety)
            ffFwd  = Range.clip(ffFwd,  -1.0, 1.0);
            ffStr  = Range.clip(ffStr,  -1.0, 1.0);
            ffTurn = Range.clip(ffTurn, -1.0, 1.0);

            // Update localization
            robot.follower.getPoseTracker().update();
            Pose cur = robot.follower.getPose();
            double curH = cur.getHeading();

            // ===== POSE ERROR (in FIELD FRAME) =====
            double errX_Field = targetX - cur.getX();
            double errY_Field = targetY - cur.getY();
            double errH = normalizeAngle(targetH - curH);

            // ===== ROTATE ERROR INTO ROBOT FRAME =====
            double cosH = Math.cos(curH);
            double sinH = Math.sin(curH);
            double errX_Robot =  cosH * errX_Field + sinH * errY_Field;
            double errY_Robot = -sinH * errX_Field + cosH * errY_Field;

            double dt = now - prevTime;

            // Filtered derivatives for D-term (in ROBOT FRAME)
            double rawDx = dt > 1e-6 ? (errX_Robot - prevErrorX) / dt : 0;
            double rawDy = dt > 1e-6 ? (errY_Robot - prevErrorY) / dt : 0;
            double rawDh = dt > 1e-6 ? (errH - prevErrorHeading) / dt : 0;

            filteredDx = filteredDx + D_FILTER_ALPHA * (rawDx - filteredDx);
            filteredDy = filteredDy + D_FILTER_ALPHA * (rawDy - filteredDy);
            filteredDh = filteredDh + D_FILTER_ALPHA * (rawDh - filteredDh);

            // PD correction (in ROBOT FRAME)
            double corrX_Robot = errX_Robot * kP_TRANSLATION + filteredDx * kD_TRANSLATION;
            double corrY_Robot = errY_Robot * kP_TRANSLATION + filteredDy * kD_TRANSLATION;
            double corrH = errH * kP_ROTATION + filteredDh * kD_ROTATION;

            // Clamp correction
            double corrMag = Math.hypot(corrX_Robot, corrY_Robot);
            if (corrMag > MAX_CORRECTION) {
                corrX_Robot *= MAX_CORRECTION / corrMag;
                corrY_Robot *= MAX_CORRECTION / corrMag;
            }
            corrH = Range.clip(corrH, -MAX_CORRECTION, MAX_CORRECTION);

            // Update PD state
            prevErrorX = errX_Robot;
            prevErrorY = errY_Robot;
            prevErrorHeading = errH;
            prevTime = now;

            // ===== CONTINUOUS BLENDING BASED ON ERROR =====
            // w = exp(-k * error) → 1.0 at zero error, drops as error grows
            double posError = Math.hypot(errX_Field, errY_Field);
            double ffWeight = Math.exp(-BLEND_K * posError);
            ffWeight = Range.clip(ffWeight, MIN_FF_WEIGHT, 1.0);

            // ===== COMBINE FEEDFORWARD + CORRECTION (both in ROBOT FRAME) =====
            double robotFwd  = ffFwd  * ffWeight + corrX_Robot * (1 - ffWeight);
            double robotStr  = ffStr  * ffWeight + corrY_Robot * (1 - ffWeight);
            double robotTurn = ffTurn * ffWeight + corrH      * (1 - ffWeight);

            // ===== SLEW RATE LIMITING =====
            double maxDelta = MAX_SLEW_RATE * dt;
            robotFwd  = prevRobotFwd  + Range.clip(robotFwd  - prevRobotFwd,  -maxDelta, maxDelta);
            robotStr  = prevRobotStr  + Range.clip(robotStr  - prevRobotStr,  -maxDelta, maxDelta);
            robotTurn = prevRobotTurn + Range.clip(robotTurn - prevRobotTurn, -maxDelta, maxDelta);

            prevRobotFwd = robotFwd;
            prevRobotStr = robotStr;
            prevRobotTurn = robotTurn;

            // ===== MECANUM MIXING (identical to TeleOp path) =====
            double fl = robotFwd + robotStr + robotTurn;
            double fr = robotFwd - robotStr - robotTurn;
            double bl = robotFwd - robotStr + robotTurn;
            double br = robotFwd + robotStr - robotTurn;

            // ===== PROPER MECANUM NORMALIZATION =====
            // Normalize so that |fwd| + |strafe| + |turn| <= 1 preserves ratios
            double maxSum = Math.abs(robotFwd) + Math.abs(robotStr) + Math.abs(robotTurn);
            if (maxSum > 1.0) {
                clipCount++;
                double scale = 1.0 / maxSum;
                fl *= scale;
                fr *= scale;
                bl *= scale;
                br *= scale;
            }
            maxWheelPowerSeen = Math.max(maxWheelPowerSeen, Math.max(Math.abs(fl),
                    Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

            leftFront.setPower(fl);
            rightFront.setPower(fr);
            leftRear.setPower(bl);
            rightRear.setPower(br);

            // Mechanisms
            controlMechanisms(fA, fB, t);

            // Stats
            avgFFWeight += ffWeight;
            loopCount++;

            // Telemetry
            telemetry.addLine("=== Replay V7 ===");
            telemetry.addData("Time", "%.2f/%.2f s (scale %.2f)", now, duration * timeScale, timeScale);
            telemetry.addData("Frame", "%d/%d (lerp %.2f)", idx, recordedFrames.size(), t);
            telemetry.addData("FF wt", "%.0f%% (avg %.0f%%)", ffWeight * 100, (avgFFWeight / loopCount) * 100);
            telemetry.addData("FF cmd", "fwd=%.2f str=%.2f turn=%.2f", ffFwd, ffStr, ffTurn);
            telemetry.addData("Corr", "x=%.2f y=%.2f h=%.2f", corrX_Robot, corrY_Robot, corrH);
            telemetry.addData("Slew", "fwd=%.2f str=%.2f turn=%.2f", robotFwd, robotStr, robotTurn);
            telemetry.addData("PosErr", "%.2f", posError);
            telemetry.addData("Target", "(%.1f, %.1f) h=%.1f°", targetX, targetY, Math.toDegrees(targetH));
            telemetry.addData("Current", "(%.1f, %.1f) h=%.1f°", cur.getX(), cur.getY(), Math.toDegrees(curH));
            telemetry.addData("Voltage", "%.1f V (rec %.1f V)", cachedVoltage, recordedVoltage);
            telemetry.addData("MaxWheel", "%.2f (clips %d)", maxWheelPowerSeen, clipCount);
            telemetry.update();

            idle();
        }
    }

    // -------------------------------------------------------------------------
    // MECHANISMS
    // -------------------------------------------------------------------------
    private void controlMechanisms(RobotFrame a, RobotFrame b, double t) {
        // Servos — interpolated
        robot.turretServo.setPosition(lerp(a.turretPos, b.turretPos, t));
        robot.angleServo.setPosition(lerp(a.anglePos, b.anglePos, t));
        robot.flapsServo.setPosition(lerp(a.flapPos, b.flapPos, t));

        // Motors — snap to nearest frame (fast response, interpolation not critical)
        RobotFrame src = (t < 0.5) ? a : b;

        if (robot.intakeMotor != null) robot.intakeMotor.setPower(src.intakePwr);
        if (robot.loaderMotor != null) robot.loaderMotor.setPower(src.loaderPwr);
        if (robot.leftOuttake != null) robot.leftOuttake.setPower(src.leftShtrPwr);
        if (robot.rightOuttake != null) robot.rightOuttake.setPower(src.rightShtrPwr);
    }

    private void stopMechanisms() {
        if (robot.intakeMotor != null) robot.intakeMotor.setPower(0);
        if (robot.loaderMotor != null) robot.loaderMotor.setPower(0);
        if (robot.leftOuttake != null) robot.leftOuttake.setPower(0);
        if (robot.rightOuttake != null) robot.rightOuttake.setPower(0);
    }

    private void stopRobot() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    // -------------------------------------------------------------------------
    // DATA LOADING
    // -------------------------------------------------------------------------
    private void loadRecordedData() throws IOException {
        File f = new File(CSV_PATH);
        if (!f.exists()) throw new IOException("CSV not found: " + CSV_PATH);

        try (BufferedReader r = new BufferedReader(new FileReader(f))) {
            r.readLine(); // skip header
            String line;
            while ((line = r.readLine()) != null) {
                String[] d = line.split(",");
                if (d.length >= 15) recordedFrames.add(new RobotFrame(d));
            }
        }
    }

    private void calculateRecordedVoltage() {
        double total = 0;
        int count = 0;
        for (RobotFrame f : recordedFrames) {
            if (f.voltage > 5) { total += f.voltage; count++; }
        }
        if (count > 0) recordedVoltage = total / count;
    }

    private void refreshVoltage(double now) {
        if (now - lastVoltageReadTime >= VOLTAGE_REFRESH_SEC) {
            cachedVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            lastVoltageReadTime = now;
        }
    }

    private void updateTimeScaling() {
        double ratio = cachedVoltage / recordedVoltage;
        if (ratio < 1.0) {
            timeScale = 1.0 + TIME_SCALE_FACTOR * (1.0 - ratio);
        } else {
            timeScale = 1.0;
        }
        timeScale = Range.clip(timeScale, MIN_TIME_SCALE, MAX_TIME_SCALE);
    }

    // -------------------------------------------------------------------------
    // UTILITIES
    // -------------------------------------------------------------------------
    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private static double lerpAngle(double a, double b, double t) {
        return normalizeAngle(a + normalizeAngle(b - a) * t);
    }

    private static double normalizeAngle(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
