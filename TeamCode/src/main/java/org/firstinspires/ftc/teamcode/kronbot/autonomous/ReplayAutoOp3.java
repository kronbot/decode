package org.firstinspires.ftc.teamcode.kronbot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
 * Replay Autonomous — fixed version of the original simple architecture.
 *
 * Reads the V3 CSV format produced by DataRecordingOp3:
 *   Time, LR, RR, LF, RF, X, Y, Heading, Voltage,
 *   IntakeVel, LoaderVel, LeftShtrVel, RightShtrVel,
 *   TurretPos, AnglePos, FlapPos
 *
 * Fixes applied over the original ReplayAutoOp:
 *   1. sleep(20) removed — replaced with idle() so loop runs at hardware speed
 *   2. D-term prevTime bug fixed — prevTime updated once per loop after ALL PD calculations
 *   3. Frame interpolation — lerps between adjacent frames for smooth pose targets
 *   4. Mechanism velocities — reads IntakeVel/LoaderVel columns and converts to power
 *   5. Voltage cached — sensor read at most every 100ms, not every loop
 *
 * Architecture is intentionally kept identical to the original:
 *   setTeleOpDrive() is used for all motor output, PedroPathing handles localization.
 *   No direct motor writes — avoids any conflict with PedroPathing's internal update().
 */
@Autonomous(name = "Replay Auto V3", group = "Autonomous")
public class ReplayAutoOp3 extends LinearOpMode {

    private static final String CSV_PATH = "/sdcard/robot_data.csv";

    // PD gains — same as original
    private static final double MAX_POWER       = 0.8;
    private static final double kP_translation  = 0.08;
    private static final double kD_translation  = 0.02;
    private static final double kP_rotation     = 1.5;
    private static final double kD_rotation     = 0.1;

    // Lookahead: follow a target this many seconds ahead in the recording
    // Keeps the robot moving smoothly instead of stalling at near-zero error
    private static final double LOOKAHEAD_TIME  = 0.15;

    // Derivative smoothing — prevents D-term spikes at start/stop transitions
    private static final double D_FILTER_ALPHA     = 0.5;   // low-pass filter (0 = ignore D, 1 = raw)
    private static final double MAX_D_CONTRIBUTION = 0.15;  // max power the D-term alone can add

    // Velocity-transition ramp — ramps output 0→1 over this period whenever the
    // target transitions from stationary to moving (fires at every stop→start, not just opmode start)
    private static final double RAMP_UP_TIME       = 0.3;   // seconds
    private static final double TARGET_STILL_THRESH = 5.0;  // cm/sec — target speed below this = "still"

    // Battery compensation
    private static final double NOMINAL_VOLTAGE      = 12.0;
    private static final double TIME_SCALING_FACTOR  = 0.7;

    // Voltage sensor is slow (I2C) — only re-read every 100ms
    private static final double VOLTAGE_REFRESH_SEC  = 0.1;

    // Intake/loader velocity → power scaling
    // Tune this if intake feels too weak or too strong during replay
    private static final double INTAKE_VEL_SCALE  = 500.0; // ticks/sec that maps to power 1.0
    private static final double LOADER_VEL_SCALE  = 500.0;

    // Shooter
    private static final double SHOOTER_VEL_THRESHOLD = 50.0;  // below this = off
    private static final double SHOOTER_MAX_VEL       = 2000.0; // tune to your motor's free-spin vel

    private final Robot robot = Robot.getInstance();
    private final ElapsedTime runtime = new ElapsedTime();
    private final List<RobotFrame> recordedFrames = new ArrayList<>();

    // PD state
    private double prevErrorX       = 0;
    private double prevErrorY       = 0;
    private double prevErrorHeading = 0;
    private double prevTime         = 0;
    // Filtered derivatives — smoothed across loops to prevent spikes
    private double filteredDx       = 0;
    private double filteredDy       = 0;
    private double filteredDh       = 0;

    // Velocity-transition ramp state
    private double prevTargetX      = 0;
    private double prevTargetY      = 0;
    private boolean targetWasStill  = true;   // was the target stationary last loop?
    private double motionStartTime  = 0;      // timestamp when target started moving

    // Voltage caching
    private double cachedVoltage       = NOMINAL_VOLTAGE;
    private double lastVoltageReadTime = -999;

    // Battery compensation result
    private double recordedVoltage   = NOMINAL_VOLTAGE;
    private double timeScalingFactor = 1.0;

    // -------------------------------------------------------------------------
    // Data model — matches the V3 CSV written by DataRecordingOp3
    // -------------------------------------------------------------------------
    private static class RobotFrame {
        double timestamp;
        // Drive motor powers (columns 1-4, used as feedforward reference only — not replayed directly)
        double lrPow, rrPow, lfPow, rfPow;
        // Pose
        double x, y, heading;
        double voltage;
        // Mechanism velocities
        double intakeVel, loaderVel, leftShtrVel, rightShtrVel;
        // Servo positions
        double turretPos, anglePos, flapPos;

        RobotFrame(String[] d) {
            timestamp   = Double.parseDouble(d[0]);
            lrPow       = Double.parseDouble(d[1]);
            rrPow       = Double.parseDouble(d[2]);
            lfPow       = Double.parseDouble(d[3]);
            rfPow       = Double.parseDouble(d[4]);
            x           = Double.parseDouble(d[5]);
            y           = Double.parseDouble(d[6]);
            heading     = Double.parseDouble(d[7]);
            voltage     = Double.parseDouble(d[8]);
            intakeVel   = Double.parseDouble(d[9]);
            loaderVel   = Double.parseDouble(d[10]);
            leftShtrVel = Double.parseDouble(d[11]);
            rightShtrVel= Double.parseDouble(d[12]);
            turretPos   = Double.parseDouble(d[13]);
            anglePos    = Double.parseDouble(d[14]);
            flapPos     = Double.parseDouble(d[15]);
        }
    }

    // -------------------------------------------------------------------------
    // OpMode
    // -------------------------------------------------------------------------
    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing Replay Auto...");
        telemetry.update();

        robot.initFollower(hardwareMap, true);
        robot.init(hardwareMap);

        try {
            robot.follower.getPoseTracker().resetIMU();
        } catch (InterruptedException e) {
            telemetry.addLine("IMU Reset Interrupted");
        }

        try {
            loadRecordedData();

            if (recordedFrames.isEmpty()) {
                telemetry.addLine("ERROR: No recorded data found!");
                telemetry.update();
                return;
            }

            calculateRecordedVoltage();

            RobotFrame first = recordedFrames.get(0);
            robot.follower.setPose(new Pose(first.x, first.y, first.heading));

            telemetry.addLine("Ready.");
            telemetry.addData("Frames", recordedFrames.size());
            telemetry.addData("Duration", "%.2f s",
                    recordedFrames.get(recordedFrames.size() - 1).timestamp - first.timestamp);
            telemetry.addData("Recorded voltage", "%.1f V", recordedVoltage);
            telemetry.update();

            waitForStart();
            if (isStopRequested()) return;

            executePlayback();

            robot.follower.setTeleOpDrive(0, 0, 0, true);
            robot.follower.update();
            stopMechanisms();

        } catch (Exception e) {
            telemetry.addData("ERROR", e.toString());
            telemetry.update();
            robot.follower.setTeleOpDrive(0, 0, 0, true);
            robot.follower.update();
            stopMechanisms();
            sleep(3000);
        }
    }

    // -------------------------------------------------------------------------
    // Main loop
    // -------------------------------------------------------------------------
    private void executePlayback() {
        runtime.reset();

        double startTs = recordedFrames.get(0).timestamp;
        double endTs   = recordedFrames.get(recordedFrames.size() - 1).timestamp;
        double duration = endTs - startTs;

        int idx = 0;
        prevTime         = 0;
        prevErrorX       = 0;
        prevErrorY       = 0;
        prevErrorHeading = 0;
        filteredDx       = 0;
        filteredDy       = 0;
        filteredDh       = 0;

        // Init velocity-transition ramp with first frame's position
        RobotFrame firstFrame = recordedFrames.get(0);
        prevTargetX      = firstFrame.x;
        prevTargetY      = firstFrame.y;
        targetWasStill   = true;
        motionStartTime  = 0;

        robot.follower.startTeleopDrive();

        while (opModeIsActive() && idx < recordedFrames.size() - 1) {
            double now = runtime.seconds();

            // Refresh voltage sensor at most every 100ms
            refreshVoltage(now);
            updateTimeScaling();

            // Current position in the recording, with lookahead added
            double recordingTime = (now / timeScalingFactor) + LOOKAHEAD_TIME;
            double targetTs = startTs + recordingTime;

            // Advance index to the frame just before targetTs
            while (idx < recordedFrames.size() - 1
                    && recordedFrames.get(idx + 1).timestamp <= targetTs) {
                idx++;
            }

            // Interpolate between frame[idx] and frame[idx+1]
            RobotFrame fA = recordedFrames.get(idx);
            RobotFrame fB = (idx + 1 < recordedFrames.size())
                    ? recordedFrames.get(idx + 1) : fA;

            double t = 0;
            if (fB.timestamp > fA.timestamp) {
                t = (targetTs - fA.timestamp) / (fB.timestamp - fA.timestamp);
                t = Range.clip(t, 0.0, 1.0);
            }

            // Smooth interpolated target pose
            double targetX = lerp(fA.x, fB.x, t);
            double targetY = lerp(fA.y, fB.y, t);
            double targetH = lerpAngle(fA.heading, fB.heading, t);
            Pose target = new Pose(targetX, targetY, targetH);

            // Update localizer
            robot.follower.update();
            Pose cur = robot.follower.getPose();

            // --- PD with filtered derivative ---
            double dt = now - prevTime;

            // Translation PD (field-centric)
            double ex   = target.getX() - cur.getX();
            double ey   = target.getY() - cur.getY();

            double rawDx = dt > 1e-6 ? (ex - prevErrorX) / dt : 0;
            double rawDy = dt > 1e-6 ? (ey - prevErrorY) / dt : 0;

            // Low-pass filter the derivatives to eliminate spikes
            filteredDx = filteredDx + D_FILTER_ALPHA * (rawDx - filteredDx);
            filteredDy = filteredDy + D_FILTER_ALPHA * (rawDy - filteredDy);

            // Clamp derivative contribution
            double dxClamped = Range.clip(filteredDx * kD_translation, -MAX_D_CONTRIBUTION, MAX_D_CONTRIBUTION);
            double dyClamped = Range.clip(filteredDy * kD_translation, -MAX_D_CONTRIBUTION, MAX_D_CONTRIBUTION);

            double fx = ex * kP_translation + dxClamped;
            double fy = ey * kP_translation + dyClamped;

            // Rotate field-centric correction into robot frame
            double cosH    =  Math.cos(cur.getHeading());
            double sinH    =  Math.sin(cur.getHeading());
            double fwdCmd  =  cosH * fx + sinH * fy;
            double strCmd  = -sinH * fx + cosH * fy;

            // Clamp translation magnitude
            double norm = Math.hypot(fwdCmd, strCmd);
            if (norm > MAX_POWER) {
                fwdCmd *= MAX_POWER / norm;
                strCmd *= MAX_POWER / norm;
            }

            // Rotation PD with filtered derivative
            double eh   = normalizeAngle(target.getHeading() - cur.getHeading());
            double rawDh = dt > 1e-6 ? (eh - prevErrorHeading) / dt : 0;
            filteredDh = filteredDh + D_FILTER_ALPHA * (rawDh - filteredDh);
            double dhClamped = Range.clip(filteredDh * kD_rotation, -MAX_D_CONTRIBUTION, MAX_D_CONTRIBUTION);

            double turnCmd = Range.clip(
                    eh * kP_rotation + dhClamped,
                    -MAX_POWER, MAX_POWER
            );

            // Velocity-transition ramp — detect when target goes from still → moving
            double targetDist = Math.hypot(targetX - prevTargetX, targetY - prevTargetY);
            double targetSpeed = dt > 1e-6 ? targetDist / dt : 0;  // cm/sec, independent of loop rate
            boolean targetIsStill = targetSpeed < TARGET_STILL_THRESH;

            if (targetWasStill && !targetIsStill) {
                // Target just started moving — begin a new ramp and reset stale derivatives
                motionStartTime = now;
                filteredDx = 0;
                filteredDy = 0;
                filteredDh = 0;
            }
            targetWasStill = targetIsStill;
            prevTargetX = targetX;
            prevTargetY = targetY;

            double timeSinceMotionStart = now - motionStartTime;
            double ramp = Range.clip(timeSinceMotionStart / RAMP_UP_TIME, 0.0, 1.0);
            fwdCmd  *= ramp;
            strCmd  *= ramp;
            turnCmd *= ramp;

            // Update PD state — ONCE, after all derivative calculations
            prevErrorX       = ex;
            prevErrorY       = ey;
            prevErrorHeading = eh;
            prevTime         = now;

            // Drive via PedroPathing (same as original — no motor bypass)
            robot.follower.setTeleOpDrive(fwdCmd, strCmd, turnCmd, false);

            // Mechanisms
            controlMechanisms(fA, fB, t);

            // Telemetry
            telemetry.addData("Time",    "%.2f / %.2f s  (scale %.2f)", now, duration, timeScalingFactor);
            telemetry.addData("Frame",   "%d / %d  (t=%.2f)", idx, recordedFrames.size(), t);
            telemetry.addData("PosErr",  "%.2f cm",  Math.hypot(ex, ey));
            telemetry.addData("HeadErr", "%.1f °",   Math.toDegrees(Math.abs(eh)));
            telemetry.addData("Cmd",     "fwd=%.2f str=%.2f turn=%.2f", fwdCmd, strCmd, turnCmd);
            telemetry.addData("Voltage", "%.1f V  (rec %.1f V)", cachedVoltage, recordedVoltage);
            telemetry.update();

            // FIX #1: no sleep(20) — yield to SDK scheduler only
            idle();
        }
    }

    // -------------------------------------------------------------------------
    // Mechanisms
    // -------------------------------------------------------------------------
    private void controlMechanisms(RobotFrame a, RobotFrame b, double t) {
        // Servos — interpolated
        robot.turretServo.setPosition(lerp(a.turretPos, b.turretPos, t));
        robot.angleServo.setPosition( lerp(a.anglePos,  b.anglePos,  t));
        robot.flapsServo.setPosition( lerp(a.flapPos,   b.flapPos,   t));

        // Shooter — simple FF+P velocity controller
        double targetVel = lerp(a.leftShtrVel, b.leftShtrVel, t);
        if (Math.abs(targetVel) > SHOOTER_VEL_THRESHOLD) {
            double ffPower    = targetVel / SHOOTER_MAX_VEL;
            double velError   = targetVel - robot.leftOuttake.getVelocity();
            double shootPower = Range.clip(ffPower + velError * 0.0008, 0.0, 1.0);
            robot.leftOuttake.setPower(shootPower);
            robot.rightOuttake.setPower(shootPower);
        } else {
            // Gentle brake, same as original TeleOp behaviour
            double braking = robot.leftOuttake.getVelocity() > 21 ? -0.1 : 0.0;
            robot.leftOuttake.setPower(braking);
            robot.rightOuttake.setPower(braking);
        }

        // Intake and loader — velocity → power
        double intakePow = velToPower(lerp(a.intakeVel, b.intakeVel, t), INTAKE_VEL_SCALE);
        double loaderPow = velToPower(lerp(a.loaderVel, b.loaderVel, t), LOADER_VEL_SCALE);
        robot.intakeMotor.setPower(intakePow);
        robot.loaderMotor.setPower(loaderPow);
    }

    private void stopMechanisms() {
        robot.intakeMotor.setPower(0);
        robot.loaderMotor.setPower(0);
        robot.leftOuttake.setPower(0);
        robot.rightOuttake.setPower(0);
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------
    private void loadRecordedData() throws IOException {
        File f = new File(CSV_PATH);
        if (!f.exists()) throw new IOException("CSV not found: " + CSV_PATH);
        try (BufferedReader r = new BufferedReader(new FileReader(f))) {
            r.readLine(); // skip header
            String line;
            while ((line = r.readLine()) != null) {
                String[] d = line.split(",");
                if (d.length >= 16) recordedFrames.add(new RobotFrame(d));
            }
        }
    }

    private void calculateRecordedVoltage() {
        double total = 0; int n = 0;
        for (RobotFrame f : recordedFrames) {
            if (f.voltage > 0) { total += f.voltage; n++; }
        }
        if (n > 0) recordedVoltage = total / n;
    }

    private void refreshVoltage(double now) {
        if (now - lastVoltageReadTime >= VOLTAGE_REFRESH_SEC) {
            cachedVoltage      = hardwareMap.voltageSensor.iterator().next().getVoltage();
            lastVoltageReadTime = now;
        }
    }

    private void updateTimeScaling() {
        double ratio = cachedVoltage / recordedVoltage;
        timeScalingFactor = ratio < 1.0
                ? Range.clip(1.0 + TIME_SCALING_FACTOR * (1.0 - ratio), 1.0, 2.0)
                : 1.0;
    }

    /** Converts a recorded velocity to a clamped power value, preserving sign. */
    private static double velToPower(double vel, double scale) {
        return Range.clip(Math.signum(vel) * Math.min(Math.abs(vel) / scale, 1.0), -1.0, 1.0);
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private static double lerpAngle(double a, double b, double t) {
        return normalizeAngle(a + normalizeAngle(b - a) * t);
    }

    private static double normalizeAngle(double a) {
        while (a >  Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}