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
 * Replay Auto V8 — Exact Match to MainDrivingOp
 *
 * FIXES from V7:
 *   1. Bug fix: getInterpolatedFrame uses lightweight InterpolatedInputs class
 *   2. Mechanisms behave IDENTICALLY to MainDrivingOp (auto-aim, toggles, etc.)
 *   3. Normalization: max wheel power
 *   4. Blending: additive (ff + scaled corr)
 *   5. Dynamic correction cap
 *   6. Disable lookahead near stop
 *   7. Clamp dt
 *   8. Lerp motor powers
 *
 * CSV Format (from DataRecordingOp8):
 *   Time,X,Y,Heading,Voltage,GamepadFwd,GamepadStr,GamepadTurn,
 *   IntakePwr,LoaderPwr,LShooterPwr,RShooterPwr,TurretPos,AnglePos,FlapPos,AutoAim,BlueTarget
 */
@Autonomous(name = "Replay Auto V8", group = "Replay")
public class ReplayAutoOp8 extends LinearOpMode {

    private static final String CSV_PATH = "/sdcard/robot_data_v8.csv";

    // ========== TUNABLE CONSTANTS ==========

    /** Continuous blending: how fast correction weight grows with error. */
    private static final double BLEND_K = 0.35;

    /** PD gains — LOW because driver input does the heavy lifting. */
    private static final double kP_TRANSLATION = 0.018;
    private static final double kD_TRANSLATION = 0.010;
    private static final double kP_ROTATION = 0.28;
    private static final double kD_ROTATION = 0.07;

    /** Base correction cap (small error). */
    private static final double MIN_CORRECTION_CAP = 0.20;
    /** Max correction cap (large error). */
    private static final double MAX_CORRECTION_CAP = 0.60;
    /** Error scale for dynamic cap transition. */
    private static final double CORRECTION_ERROR_SCALE = 8.0;

    /** Lookahead: follow a point this many seconds ahead. */
    private static final double LOOKAHEAD_TIME = 0.08;
    /** Threshold to disable lookahead (near stop). */
    private static final double LOOKAHEAD_DISABLE_THRESH = 0.04;

    /** Time scaling for battery compensation. */
    private static final double TIME_SCALE_FACTOR = 0.55;
    private static final double MAX_TIME_SCALE = 1.6;
    private static final double MIN_TIME_SCALE = 0.85;

    /** D-term low-pass filter. */
    private static final double D_FILTER_ALPHA = 0.45;

    /** Slew rate limit: max change in command per second. */
    private static final double MAX_SLEW_RATE = 5.0;

    /** Voltage sensor refresh rate. */
    private static final double VOLTAGE_REFRESH_SEC = 0.1;

    /** dt clamp to prevent derivative spikes from timing jitter. */
    private static final double MIN_DT = 0.008;
    private static final double MAX_DT = 0.050;

    // ========== MOTOR DIRECTIONS ==========
    // COPY THESE EXACTLY FROM THE TELEMETRY OUTPUT OF DataRecordingOp8
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
    private double avgCorrWeight = 0;
    private int loopCount = 0;

    // -------------------------------------------------------------------------
    // DATA MODEL
    // -------------------------------------------------------------------------
    private static class RobotFrame {
        double timestamp;
        double x, y, heading;
        double voltage;
        double gpFwd, gpStr, gpTurn;   // DRIVER INPUTS in [-1, 1]
        double intakePwr, loaderPwr, leftShtrPwr, rightShtrPwr;
        double turretPos, anglePos, flapPos;
        boolean autoAimEnabled;
        boolean blueTarget;

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
            autoAimEnabled = Integer.parseInt(d[15]) != 0;
            blueTarget  = Integer.parseInt(d[16]) != 0;
        }
    }

    /** Lightweight class for interpolated inputs — BUG FIX from V7 */
    private static class InterpolatedInputs {
        double gpFwd, gpStr, gpTurn;
        double intakePwr, loaderPwr, leftShtrPwr, rightShtrPwr;
        double turretPos, anglePos, flapPos;
        boolean autoAimEnabled;
        boolean blueTarget;
    }

    // -------------------------------------------------------------------------
    // MAIN
    // -------------------------------------------------------------------------
    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing Replay Auto V8...");
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
        robot.Blue_Target = first.blueTarget;

        telemetry.addLine("=== Replay Auto V8 Ready ===");
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
        avgCorrWeight = 0;
        loopCount = 0;

        int idx = 0;

        while (opModeIsActive() && idx < recordedFrames.size() - 1) {
            double now = runtime.seconds();

            // Refresh voltage and compute time scaling
            refreshVoltage(now);
            updateTimeScaling();

            // ===== LOOKAHEAD (disable near stop) =====
            double currentRecordingTime = now / timeScale;
            InterpolatedInputs currentInputs = getInterpolatedInputs(currentRecordingTime, startTs);
            double currentInputMag = Math.abs(currentInputs.gpFwd)
                    + Math.abs(currentInputs.gpStr)
                    + Math.abs(currentInputs.gpTurn);

            double lookahead = (currentInputMag > LOOKAHEAD_DISABLE_THRESH) ? LOOKAHEAD_TIME : 0.0;

            // Current position in recording, with lookahead and time scaling
            double recordingTime = currentRecordingTime + lookahead;
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
            double ffFwd  = lerp(fA.gpFwd,  fB.gpFwd,  t);
            double ffStr  = lerp(fA.gpStr,  fB.gpStr,  t);
            double ffTurn = lerp(fA.gpTurn, fB.gpTurn, t);

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

            // ===== CLAMPED dt =====
            double rawDt = now - prevTime;
            double dt = Range.clip(rawDt, MIN_DT, MAX_DT);

            // Filtered derivatives for D-term (in ROBOT FRAME)
            double rawDx = (errX_Robot - prevErrorX) / dt;
            double rawDy = (errY_Robot - prevErrorY) / dt;
            double rawDh = (errH - prevErrorHeading) / dt;

            filteredDx = filteredDx + D_FILTER_ALPHA * (rawDx - filteredDx);
            filteredDy = filteredDy + D_FILTER_ALPHA * (rawDy - filteredDy);
            filteredDh = filteredDh + D_FILTER_ALPHA * (rawDh - filteredDh);

            // PD correction (in ROBOT FRAME)
            double corrX_Robot = errX_Robot * kP_TRANSLATION + filteredDx * kD_TRANSLATION;
            double corrY_Robot = errY_Robot * kP_TRANSLATION + filteredDy * kD_TRANSLATION;
            double corrH = errH * kP_ROTATION + filteredDh * kD_ROTATION;

            // ===== DYNAMIC CORRECTION CAP =====
            double posError = Math.hypot(errX_Field, errY_Field);
            double corrScale = Math.min(posError / CORRECTION_ERROR_SCALE, 1.0);
            double dynamicMaxCorr = lerp(MIN_CORRECTION_CAP, MAX_CORRECTION_CAP, corrScale);

            double corrMag = Math.hypot(corrX_Robot, corrY_Robot);
            if (corrMag > dynamicMaxCorr) {
                double scale = dynamicMaxCorr / corrMag;
                corrX_Robot *= scale;
                corrY_Robot *= scale;
            }
            corrH = Range.clip(corrH, -dynamicMaxCorr, dynamicMaxCorr);

            // Update PD state
            prevErrorX = errX_Robot;
            prevErrorY = errY_Robot;
            prevErrorHeading = errH;
            prevTime = now;

            // ===== CONTINUOUS BLENDING (additive) =====
            double corrWeight = 1.0 - Math.exp(-BLEND_K * posError);
            corrWeight = Range.clip(corrWeight, 0.0, 1.0);

            // ===== ADDITIVE BLENDING: u = u_ff + corrWeight * u_fb =====
            double robotFwd  = ffFwd  + corrWeight * corrX_Robot;
            double robotStr  = ffStr  + corrWeight * corrY_Robot;
            double robotTurn = ffTurn + corrWeight * corrH;

            // Clamp combined command to [-1, 1] before mixing
            double combinedMag = Math.hypot(robotFwd, robotStr);
            if (combinedMag > 1.0) {
                robotFwd /= combinedMag;
                robotStr /= combinedMag;
            }
            robotTurn = Range.clip(robotTurn, -1.0, 1.0);

            // ===== SLEW RATE LIMITING =====
            double maxDelta = MAX_SLEW_RATE * dt;
            robotFwd  = prevRobotFwd  + Range.clip(robotFwd  - prevRobotFwd,  -maxDelta, maxDelta);
            robotStr  = prevRobotStr  + Range.clip(robotStr  - prevRobotStr,  -maxDelta, maxDelta);
            robotTurn = prevRobotTurn + Range.clip(robotTurn - prevRobotTurn, -maxDelta, maxDelta);

            prevRobotFwd = robotFwd;
            prevRobotStr = robotStr;
            prevRobotTurn = robotTurn;

            // ===== MECANUM MIXING =====
            double fl = robotFwd + robotStr + robotTurn;
            double fr = robotFwd - robotStr - robotTurn;
            double bl = robotFwd - robotStr + robotTurn;
            double br = robotFwd + robotStr - robotTurn;

            // ===== NORMALIZE: max wheel power =====
            double maxWheel = Math.max(1.0,
                    Math.max(Math.abs(fl),
                            Math.max(Math.abs(fr),
                                    Math.max(Math.abs(bl), Math.abs(br)))));

            if (maxWheel > 1.0) {
                clipCount++;
                fl /= maxWheel;
                fr /= maxWheel;
                bl /= maxWheel;
                br /= maxWheel;
            }
            maxWheelPowerSeen = Math.max(maxWheelPowerSeen, maxWheel);

            leftFront.setPower(fl);
            rightFront.setPower(fr);
            leftRear.setPower(bl);
            rightRear.setPower(br);

            // ===== MECHANISMS — EXACT MATCH TO MainDrivingOp =====
            controlMechanisms(fA, fB, t);

            // Stats
            avgCorrWeight += corrWeight;
            loopCount++;

            // Telemetry
            telemetry.addLine("=== Replay V8 ===");
            telemetry.addData("Time", "%.2f/%.2f s (scale %.2f)", now, duration * timeScale, timeScale);
            telemetry.addData("Frame", "%d/%d (lerp %.2f)", idx, recordedFrames.size(), t);
            telemetry.addData("Lookahead", "%.3f s", lookahead);
            telemetry.addData("Corr%", "%.0f%% (avg %.0f%%)", corrWeight * 100, (avgCorrWeight / loopCount) * 100);
            telemetry.addData("FF cmd", "fwd=%.2f str=%.2f turn=%.2f", ffFwd, ffStr, ffTurn);
            telemetry.addData("Corr", "x=%.2f y=%.2f h=%.2f (cap %.2f)", corrX_Robot, corrY_Robot, corrH, dynamicMaxCorr);
            telemetry.addData("Final", "fwd=%.2f str=%.2f turn=%.2f", robotFwd, robotStr, robotTurn);
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
    // MECHANISMS — EXACT MATCH TO MainDrivingOp LOGIC
    // -------------------------------------------------------------------------
    private void controlMechanisms(RobotFrame a, RobotFrame b, double t) {
        // Interpolate all mechanism values
        double intakePwr = lerp(a.intakePwr, b.intakePwr, t);
        double loaderPwr = lerp(a.loaderPwr, b.loaderPwr, t);
        double leftShtrPwr = lerp(a.leftShtrPwr, b.leftShtrPwr, t);
        double rightShtrPwr = lerp(a.rightShtrPwr, b.rightShtrPwr, t);
        double turretPos = lerp(a.turretPos, b.turretPos, t);
        double anglePos = lerp(a.anglePos, b.anglePos, t);
        double flapPos = lerp(a.flapPos, b.flapPos, t);

        // Auto-aim state — use nearest frame (state changes are discrete)
        boolean autoAim = (t < 0.5) ? a.autoAimEnabled : b.autoAimEnabled;
        boolean blueTarget = (t < 0.5) ? a.blueTarget : b.blueTarget;

        // Apply to robot — EXACT same as MainDrivingOp
        if (robot.intakeMotor != null) robot.intakeMotor.setPower(intakePwr);
        if (robot.loaderMotor != null) robot.loaderMotor.setPower(loaderPwr);
        if (robot.leftOuttake != null) robot.leftOuttake.setPower(leftShtrPwr);
        if (robot.rightOuttake != null) robot.rightOuttake.setPower(rightShtrPwr);

        robot.turretServo.setPosition(turretPos);
        robot.angleServo.setPosition(anglePos);
        robot.flapsServo.setPosition(flapPos);

        // Set robot state variables for turret auto-aim logic
        robot.Blue_Target = blueTarget;
        // Note: autoAim state is used by turret.update() if it checks this
        // The recorded turretPos already includes the result of auto-aim calculations
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
    // HELPERS — BUG FIX: lightweight InterpolatedInputs class
    // -------------------------------------------------------------------------

    /**
     * Get interpolated inputs at a given recording time.
     * Uses lightweight InterpolatedInputs instead of RobotFrame constructor.
     */
    private InterpolatedInputs getInterpolatedInputs(double recordingTime, double startTs) {
        double targetTs = startTs + recordingTime;
        InterpolatedInputs result = new InterpolatedInputs();

        for (int i = 0; i < recordedFrames.size() - 1; i++) {
            RobotFrame a = recordedFrames.get(i);
            RobotFrame b = recordedFrames.get(i + 1);
            if (b.timestamp > targetTs) {
                double segDur = b.timestamp - a.timestamp;
                double t = (segDur > 1e-6)
                        ? Range.clip((targetTs - a.timestamp) / segDur, 0.0, 1.0)
                        : 0.0;
                result.gpFwd = lerp(a.gpFwd, b.gpFwd, t);
                result.gpStr = lerp(a.gpStr, b.gpStr, t);
                result.gpTurn = lerp(a.gpTurn, b.gpTurn, t);
                return result;
            }
        }

        // Fallback: return last frame
        RobotFrame last = recordedFrames.get(recordedFrames.size() - 1);
        result.gpFwd = last.gpFwd;
        result.gpStr = last.gpStr;
        result.gpTurn = last.gpTurn;
        return result;
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
                if (d.length >= 17) recordedFrames.add(new RobotFrame(d));
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
