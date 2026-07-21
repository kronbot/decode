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
 * Replay Auto V4 — simplified, accurate point‑following.
 *
 * Removed:
 *   - D‑term (and all filtering / smoothing)
 *   - Lookahead (follows the exact interpolated pose)
 *   - Velocity ramp (start/stop transitions are smooth enough with P only)
 *
 * Kept:
 *   - Voltage‑based time scaling (compensates for battery sag without exceeding 1.0 power)
 *   - Frame interpolation (lerp between adjacent recorded frames)
 *   - Mechanism replay (intake/loader/shooter via velocity → power)
 *
 * Use this if V3 was overshooting, oscillating, or lagging behind.
 */
@Autonomous(name = "FINAL REPLAY LETS GO", group = "Autonomous")
public class ReplayAuto3_3 extends LinearOpMode {

    private static final String CSV_PATH = "/sdcard/robot_data.csv";

    // P gains (tune these on your robot)
    private static final double MAX_POWER       = 0.8;
    private static final double kP_translation = 0.10;   // cm⁻¹
    private static final double kP_rotation    = 1.8;    // rad⁻¹

    // Time scaling: when battery is lower than recorded, we run the replay faster
    private static final double TIME_SCALING_FACTOR = 0.7;
    private static final double NOMINAL_VOLTAGE     = 12.0;
    private static final double VOLTAGE_REFRESH_SEC = 0.1;

    // Mechanism velocity → power scaling (same as before)
    private static final double INTAKE_VEL_SCALE  = 500.0;
    private static final double LOADER_VEL_SCALE  = 500.0;
    private static final double SHOOTER_VEL_THRESHOLD = 50.0;
    private static final double SHOOTER_MAX_VEL       = 2000.0;

    private final Robot robot = Robot.getInstance();
    private final ElapsedTime runtime = new ElapsedTime();
    private final List<RobotFrame> recordedFrames = new ArrayList<>();

    // Voltage caching
    private double cachedVoltage       = NOMINAL_VOLTAGE;
    private double lastVoltageReadTime = -999;
    private double recordedVoltage     = NOMINAL_VOLTAGE;
    private double timeScalingFactor   = 1.0;

    // -------------------------------------------------------------------------
    // Data model (matches V3 CSV)
    // -------------------------------------------------------------------------
    private static class RobotFrame {
        double timestamp;
        double lrPow, rrPow, lfPow, rfPow;   // not used for driving, only stored
        double x, y, heading;
        double voltage;
        double intakeVel, loaderVel, leftShtrVel, rightShtrVel;
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

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing Replay Auto V4...");
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
    // Main playback loop
    // -------------------------------------------------------------------------
    private void executePlayback() {
        runtime.reset();

        double startTs = recordedFrames.get(0).timestamp;
        double endTs   = recordedFrames.get(recordedFrames.size() - 1).timestamp;
        double duration = endTs - startTs;

        int idx = 0;

        robot.follower.startTeleopDrive();

        while (opModeIsActive() && idx < recordedFrames.size() - 1) {
            double now = runtime.seconds();

            // Refresh voltage (cached)
            refreshVoltage(now);
            updateTimeScaling();

            // Current playback time (scaled)
            double recordingTime = now / timeScalingFactor;
            double targetTs = startTs + recordingTime;

            // Advance index to frame just before targetTs
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

            double targetX = lerp(fA.x, fB.x, t);
            double targetY = lerp(fA.y, fB.y, t);
            double targetH = lerpAngle(fA.heading, fB.heading, t);

            // Get current pose
            robot.follower.update();
            Pose cur = robot.follower.getPose();

            // ---- Pure P control (no D, no lookahead) ----
            double ex = targetX - cur.getX();
            double ey = targetY - cur.getY();

            // Field‑centric correction → robot‑centric
            double cosH = Math.cos(cur.getHeading());
            double sinH = Math.sin(cur.getHeading());
            double fwdCmd = cosH * ex * kP_translation + sinH * ey * kP_translation;
            double strCmd = -sinH * ex * kP_translation + cosH * ey * kP_translation;

            // Clamp translation magnitude
            double norm = Math.hypot(fwdCmd, strCmd);
            if (norm > MAX_POWER) {
                fwdCmd *= MAX_POWER / norm;
                strCmd *= MAX_POWER / norm;
            }

            // Rotation
            double eh = normalizeAngle(targetH - cur.getHeading());
            double turnCmd = Range.clip(eh * kP_rotation, -MAX_POWER, MAX_POWER);

            // Send to PedroPathing (no direct motor writes)
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

            idle();  // yield to scheduler
        }
    }

    // -------------------------------------------------------------------------
    // Mechanisms (unchanged from V3)
    // -------------------------------------------------------------------------
    private void controlMechanisms(RobotFrame a, RobotFrame b, double t) {
        robot.turretServo.setPosition(lerp(a.turretPos, b.turretPos, t));
        robot.angleServo.setPosition( lerp(a.anglePos,  b.anglePos,  t));
        robot.flapsServo.setPosition( lerp(a.flapPos,   b.flapPos,   t));

        double targetVel = lerp(a.leftShtrVel, b.leftShtrVel, t);
        if (Math.abs(targetVel) > SHOOTER_VEL_THRESHOLD) {
            double ffPower    = targetVel / SHOOTER_MAX_VEL;
            double velError   = targetVel - robot.leftOuttake.getVelocity();
            double shootPower = Range.clip(ffPower + velError * 0.0008, 0.0, 1.0);
            robot.leftOuttake.setPower(shootPower);
            robot.rightOuttake.setPower(shootPower);
        } else {
            double braking = robot.leftOuttake.getVelocity() > 21 ? -0.1 : 0.0;
            robot.leftOuttake.setPower(braking);
            robot.rightOuttake.setPower(braking);
        }

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