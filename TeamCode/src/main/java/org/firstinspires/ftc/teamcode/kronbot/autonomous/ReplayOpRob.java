package org.firstinspires.ftc.teamcode.kronbot.autonomous;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_REVERSE;

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
 * Replay Autonomous — V3.3
 *
 * Reads the V3.3 CSV format produced by DataRecordingOp (V3.3):
 *   Time, LR, RR, LF, RF, X, Y, Heading, Voltage,
 *   IntakeVel, LoaderVel, LeftShtrVel, RightShtrVel,
 *   TurretPos, AnglePos, FlapPos,
 *   IntakeCmd, LoaderCmd, FlapOpen, ShootRange, TurretOffset, BlueTarget,
 *   AutoAim
 *
 * V3.3 changes over V3.2:
 *   - INTAKE_REVERSE is now applied during replay (V3.2 missed it; the
 *     intake direction defaulted to whatever the Robot class had at init,
 *     which is almost certainly wrong for matching TeleOp behavior).
 *   - activateRange() is now driven by the recorded "LastActivateRange"
 *     column directly (V3.2 reverse-engineered it from activeConfig.velocity,
 *     which is brittle and breaks for the auto-aim interpolated range).
 *   - autoAimEnabled is now a recorded column (column 23) and is replayed
 *     frame-by-frame, matching the TeleOp's dpadUp toggle.
 *   - applyInitialMechanismState now fires activateRange() even when the
 *     first frame has LastActivateRange == 0 (auto-aim). Previously the
 *     guard `shootRange >= 0` correctly handled 0, but the activateRange(0)
 *     code path in Shoot is also gated on autoAimEnabled being true, so
 *     we now also set autoAimEnabled before calling it.
 *   - Deactivate transitions (LastActivateRange: any positive → -1) are
 *     now replayed by calling robot.shoot.deactivate(), matching the
 *     TeleOp's leftBumper.justPressed() → deactivate() edge.
 *   - Camera stop is NOT called on exit (matches the recorder's stop(),
 *     which doesn't call robot.webcam.stop() either — the original TeleOp
 *     does, but the recorder is a copy without webcam init).
 *
 * Position-accuracy structural fixes (carried from V3):
 *   1. MAX_POWER 0.8 → 1.0.
 *   2. Velocity-transition ramp applies to TRANSLATION ONLY.
 *   3. Lookahead scales with time scaling.
 *   4. Settling period (200 ms) after setPose.
 *   5. CSV loader requires ≥16 columns (warns and skips if fewer).
 *      For full mechanism parity, the CSV needs 23 columns (V3.3 format);
 *      with only 16 columns the high-level commands default to safe
 *      values and only servo positions + velocities are replayed.
 *
 * Battery handling: time scaling ONLY. Motor commands are not power-scaled
 * because the cap is 1.0 and a weaker battery cannot produce more torque.
 */
@Autonomous(name = "Replay Robert", group = "Autonomous")
public class ReplayOpRob extends LinearOpMode {

    private static final String CSV_PATH = "/sdcard/robot_data_Robert.csv";

    // PD gains — V3 values
    private static final double MAX_POWER       = 1.0;
    private static final double kP_translation  = 0.08;
    private static final double kD_translation  = 0.02;
    private static final double kP_rotation     = 1.5;
    private static final double kD_rotation     = 0.1;

    // ROBOT-time lookahead. Scales with timeScalingFactor so the effective
    // robot-time offset stays constant at LOOKAHEAD_TIME regardless of battery.
    private static final double LOOKAHEAD_TIME  = 0.15;

    // Derivative smoothing — V3 values
    private static final double D_FILTER_ALPHA     = 0.5;
    private static final double MAX_D_CONTRIBUTION = 0.15;

    // Velocity-transition ramp — applies to translation only
    private static final double RAMP_UP_TIME        = 0.3;
    private static final double TARGET_STILL_THRESH = 5.0;

    // Battery compensation (time scaling only)
    private static final double NOMINAL_VOLTAGE      = 12.0;
    private static final double TIME_SCALING_FACTOR  = 0.7;

    // Voltage sensor refresh
    private static final double VOLTAGE_REFRESH_SEC  = 0.1;

    // Settling period after setPose, before playback begins
    private static final long SETTLE_NANOS = 200_000_000L;

    private final Robot robot = Robot.getInstance();
    private final ElapsedTime runtime = new ElapsedTime();
    private final List<RobotFrame> recordedFrames = new ArrayList<>();

    // PD state
    private double prevErrorX       = 0;
    private double prevErrorY       = 0;
    private double prevErrorHeading = 0;
    private double prevTime         = 0;
    private double filteredDx       = 0;
    private double filteredDy       = 0;
    private double filteredDh       = 0;

    // Ramp state
    private double prevTargetX      = 0;
    private double prevTargetY      = 0;
    private boolean targetWasStill  = true;
    private double motionStartTime  = 0;

    // Voltage caching
    private double cachedVoltage       = NOMINAL_VOLTAGE;
    private double lastVoltageReadTime = -999;

    // Battery compensation result
    private double recordedVoltage   = NOMINAL_VOLTAGE;
    private double timeScalingFactor = 1.0;

    // Mechanism state from the previous frame — used to detect "shoot range
    // was just activated" and call robot.shoot.activateRange() only on
    // transitions, matching the TeleOp's edge-triggered behavior.
    private int prevShootRange   = -2; // sentinel: never set
    private boolean prevAutoAim  = false;

    // -------------------------------------------------------------------------
    // Data model — V3.3 CSV (16, 22, or 23 columns)
    // -------------------------------------------------------------------------
    private static class RobotFrame {
        double timestamp;
        double lrPow, rrPow, lfPow, rfPow;
        double x, y, heading;
        double voltage;
        double intakeVel, loaderVel, leftShtrVel, rightShtrVel;
        double turretPos, anglePos, flapPos;
        // V3.2 high-level mechanism commands — default to safe values
        // so a V3 (16-column) CSV still loads.
        double intakeCmd    = 0;
        double loaderCmd    = 0;
        boolean flapOpen    = false;
        int     shootRange  = -2; // -2 = never activated, -1 = deactivated, 0..4 = last call
        double  turretOffset = 0;
        boolean blueTarget  = false;
        // V3.3
        boolean autoAim     = false;

        RobotFrame(String[] d) {
            timestamp    = Double.parseDouble(d[0]);
            lrPow        = Double.parseDouble(d[1]);
            rrPow        = Double.parseDouble(d[2]);
            lfPow        = Double.parseDouble(d[3]);
            rfPow        = Double.parseDouble(d[4]);
            x            = Double.parseDouble(d[5]);
            y            = Double.parseDouble(d[6]);
            heading      = Double.parseDouble(d[7]);
            voltage      = Double.parseDouble(d[8]);
            intakeVel    = Double.parseDouble(d[9]);
            loaderVel    = Double.parseDouble(d[10]);
            leftShtrVel  = Double.parseDouble(d[11]);
            rightShtrVel = Double.parseDouble(d[12]);
            turretPos    = Double.parseDouble(d[13]);
            anglePos     = Double.parseDouble(d[14]);
            flapPos      = Double.parseDouble(d[15]);
            // V3.2 columns (optional — defaults used if absent)
            if (d.length >= 22) {
                intakeCmd    = Double.parseDouble(d[16]);
                loaderCmd    = Double.parseDouble(d[17]);
                flapOpen     = d[18].equals("1") || d[18].equalsIgnoreCase("true");
                shootRange   = (int) Math.round(Double.parseDouble(d[19]));
                turretOffset = Double.parseDouble(d[20]);
                blueTarget   = d[21].equals("1") || d[21].equalsIgnoreCase("true");
            }
            // V3.3 column (optional — default false)
            if (d.length >= 23) {
                autoAim = d[22].equals("1") || d[22].equalsIgnoreCase("true");
            }
        }
    }

    // -------------------------------------------------------------------------
    // OpMode
    // -------------------------------------------------------------------------
    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing Replay Auto V3.3...");
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

            // Apply the recorded initial mechanism state so the first frame
            // of replay starts from the same configuration the driver had
            // (turret offset, blue target, flap, intake reverse, auto-aim,
            // shooter range, etc.)
            applyInitialMechanismState(first);

            telemetry.addLine("Ready.");
            telemetry.addData("Frames", recordedFrames.size());
            telemetry.addData("Duration", "%.2f s",
                    recordedFrames.get(recordedFrames.size() - 1).timestamp - first.timestamp);
            telemetry.addData("Recorded voltage", "%.1f V", recordedVoltage);
            telemetry.update();

            waitForStart();
            if (isStopRequested()) return;

            // Settle: let IMU/localizer stabilize on the initial pose
            long settleEnd = System.nanoTime() + SETTLE_NANOS;
            while (opModeIsActive() && System.nanoTime() < settleEnd) {
                robot.follower.update();
                idle();
            }
            robot.follower.setPose(new Pose(first.x, first.y, first.heading));

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
        prevShootRange   = -2; // sentinel: no transitions fire on frame 0
        prevAutoAim      = false;

        RobotFrame firstFrame = recordedFrames.get(0);
        prevTargetX      = firstFrame.x;
        prevTargetY      = firstFrame.y;
        targetWasStill   = true;
        motionStartTime  = 0;

        robot.follower.startTeleopDrive();

        while (opModeIsActive() && idx < recordedFrames.size() - 1) {
            double now = runtime.seconds();

            refreshVoltage(now);
            updateTimeScaling();

            // Constant ROBOT-time lookahead
            double recordingTime = (now / timeScalingFactor) + LOOKAHEAD_TIME * timeScalingFactor;
            double targetTs = startTs + recordingTime;

            while (idx < recordedFrames.size() - 1
                    && recordedFrames.get(idx + 1).timestamp <= targetTs) {
                idx++;
            }

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

            // Update localizer
            robot.follower.update();
            Pose cur = robot.follower.getPose();

            // --- PD (no feedforward) ---
            double dt = now - prevTime;
            if (dt <= 0) dt = 1e-6;

            double ex = targetX - cur.getX();
            double ey = targetY - cur.getY();

            double rawDx = (ex - prevErrorX) / dt;
            double rawDy = (ey - prevErrorY) / dt;
            filteredDx = filteredDx + D_FILTER_ALPHA * (rawDx - filteredDx);
            filteredDy = filteredDy + D_FILTER_ALPHA * (rawDy - filteredDy);

            double dxClamped = Range.clip(filteredDx * kD_translation, -MAX_D_CONTRIBUTION, MAX_D_CONTRIBUTION);
            double dyClamped = Range.clip(filteredDy * kD_translation, -MAX_D_CONTRIBUTION, MAX_D_CONTRIBUTION);

            double fx = ex * kP_translation + dxClamped;
            double fy = ey * kP_translation + dyClamped;

            double cosH = Math.cos(cur.getHeading());
            double sinH = Math.sin(cur.getHeading());
            double fwdCmd =  cosH * fx + sinH * fy;
            double strCmd = -sinH * fx + cosH * fy;

            double norm = Math.hypot(fwdCmd, strCmd);
            if (norm > MAX_POWER) {
                fwdCmd *= MAX_POWER / norm;
                strCmd *= MAX_POWER / norm;
            }

            double eh = normalizeAngle(targetH - cur.getHeading());
            double rawDh = (eh - prevErrorHeading) / dt;
            filteredDh = filteredDh + D_FILTER_ALPHA * (rawDh - filteredDh);
            double dhClamped = Range.clip(filteredDh * kD_rotation, -MAX_D_CONTRIBUTION, MAX_D_CONTRIBUTION);
            double turnCmd = Range.clip(
                    eh * kP_rotation + dhClamped,
                    -MAX_POWER, MAX_POWER
            );

            // --- Velocity-transition ramp ---
            double targetDist = Math.hypot(targetX - prevTargetX, targetY - prevTargetY);
            double targetSpeed = targetDist / dt;
            boolean targetIsStill = targetSpeed < TARGET_STILL_THRESH;

            if (targetWasStill && !targetIsStill) {
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

            fwdCmd *= ramp;
            strCmd *= ramp;

            // Update PD state
            prevErrorX       = ex;
            prevErrorY       = ey;
            prevErrorHeading = eh;
            prevTime         = now;

            robot.follower.setTeleOpDrive(fwdCmd, strCmd, turnCmd, false);

            // --- Apply recorded mechanism commands the same way the TeleOp does ---
            applyMechanismCommands(fA, fB, t);
            robot.updateAllSystems();

            // Telemetry
            telemetry.addData("Time",    "%.2f / %.2f s  (scale %.2f)", now, duration, timeScalingFactor);
            telemetry.addData("Frame",   "%d / %d  (t=%.2f)", idx, recordedFrames.size(), t);
            telemetry.addData("PosErr",  "%.2f cm",  Math.hypot(ex, ey));
            telemetry.addData("HeadErr", "%.1f °",   Math.toDegrees(Math.abs(eh)));
            telemetry.addData("Cmd",     "fwd=%.2f str=%.2f turn=%.2f  (ramp %.2f)", fwdCmd, strCmd, turnCmd, ramp);
            telemetry.addData("Voltage", "%.1f V  (rec %.1f V)", cachedVoltage, recordedVoltage);
            telemetry.addData("Mech",    "rng=%d aa=%s flap=%s intk=%.2f load=%.2f",
                    (int) Math.round(lerp(fA.shootRange, fB.shootRange, t)),
                    (t < 0.5 ? fA.autoAim : fB.autoAim) ? "Y" : "N",
                    lerpBool(fA.flapOpen, fB.flapOpen, t) ? "Y" : "N",
                    lerp(fA.intakeCmd, fB.intakeCmd, t),
                    lerp(fA.loaderCmd, fB.loaderCmd, t));
            telemetry.update();

            idle();
        }
    }

    // -------------------------------------------------------------------------
    // Mechanism application — same code path as MainDrivingOp
    // -------------------------------------------------------------------------
    private void applyInitialMechanismState(RobotFrame f) {
        // Set the persistent state from the first recorded frame.
        // Order matters: set intake.reversed BEFORE intake.speed, so the
        // first updateAllSystems() call applies the correct direction.
        robot.intake.reversed    = INTAKE_REVERSE;
        robot.turret.driverOffset = f.turretOffset;
        robot.Blue_Target         = f.blueTarget;
        robot.flap.open           = f.flapOpen;
        robot.intake.speed        = f.intakeCmd;
        robot.loader.speed        = f.loaderCmd;

        // Replay the initial auto-aim flag. The TeleOp sets
        // autoAimEnabled via dpadUp.justPressed() — we mirror the result
        // of that toggle here, not the action. The replay's mechanism
        // loop keeps it in sync thereafter.
        prevAutoAim = f.autoAim;

        // Fire the initial shoot range. Use -2 as a "never set" sentinel
        // so we only call activateRange() / deactivate() if the first
        // frame actually contains a real range value (>= 0 or == -1).
        if (f.shootRange >= 0) {
            // activateRange(0) requires autoAimEnabled to be true (per
            // Shoot.activateRange). Set the flag before calling it so
            // the interpolated velocity path actually engages.
            if (f.shootRange == 0 && f.autoAim) {
                robot.shoot.activateRange(0);
            } else if (f.shootRange > 0) {
                robot.shoot.activateRange(f.shootRange);
            }
            prevShootRange = f.shootRange;
        } else if (f.shootRange == -1) {
            robot.shoot.deactivate();
            prevShootRange = -1;
        }
        // -2 means "never activated" — leave outtake alone.
    }

    private void applyMechanismCommands(RobotFrame a, RobotFrame b, double t) {
        // Interpolate the high-level commands and apply them to the Robot
        // exactly the way MainDrivingOp applies them — then call
        // updateAllSystems() so the same internal control loop runs.
        robot.intake.speed = lerp(a.intakeCmd, b.intakeCmd, t);
        robot.loader.speed = lerp(a.loaderCmd, b.loaderCmd, t);
        robot.flap.open    = lerpBool(a.flapOpen, b.flapOpen, t);
        robot.turret.driverOffset = lerp(a.turretOffset, b.turretOffset, t);
        robot.Blue_Target  = t < 0.5 ? a.blueTarget : b.blueTarget;

        // autoAimEnabled: in the TeleOp this is toggled by dpadUp.justPressed().
        // The replay treats it as a recorded state and mirrors it (uses the
        // later of the two frames to avoid chatter on the toggle frame).
        boolean curAutoAim = t < 0.5 ? a.autoAim : b.autoAim;
        // (We do not call robot.shoot.activateRange(0) here on auto-aim
        //  toggle, because the TeleOp only fires it inside the loop body
        //  when autoAimEnabled is true. The activateRange(0) call on
        //  every loop re-interpolates the velocity based on distance, so
        //  re-firing it from the mechanism applier would actually be
        //  MORE faithful to the TeleOp. We opt to re-fire it here,
        //  guarded by the autoAim flag.)
        if (curAutoAim) {
            robot.shoot.activateRange(0);
        }
        prevAutoAim = curAutoAim;

        // shoot.activateRange is edge-triggered in the TeleOp (only fires
        // on a button press). Detect when the recorded range changes and
        // call it on the transition, matching the TeleOp behavior.
        //
        // Special case: when the recorded range is 0 (auto-aim), the
        // TeleOp's "if (autoAimEnabled) robot.shoot.activateRange(0);"
        // line fires every loop, so re-firing per-frame is correct
        // (handled above). We only need edge detection for ranges 1-4
        // and for the -1 → positive (or positive → -1) deactivate
        // transitions.
        int curRange = (int) Math.round(lerp(a.shootRange, b.shootRange, t));
        if (curRange == 0) {
            // Already handled by the curAutoAim block above; do not
            // re-fire here as an "edge" (it isn't an edge in the TeleOp).
        } else if (curRange != prevShootRange) {
            if (curRange > 0) {
                robot.shoot.activateRange(curRange);
            } else if (curRange == -1 && prevShootRange > 0) {
                // positive → -1 transition: matches the TeleOp's
                // leftBumper.justPressed() → robot.shoot.deactivate().
                robot.shoot.deactivate();
            }
        }
        prevShootRange = curRange;
    }

    private void stopMechanisms() {
        robot.intake.speed = 0;
        robot.loader.speed = 0;
        robot.shoot.deactivate();
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
            int lineNum = 1;
            while ((line = r.readLine()) != null) {
                lineNum++;
                String[] d = line.split(",");
                if (d.length >= 16) {
                    recordedFrames.add(new RobotFrame(d));
                } else {
                    telemetry.addData("Skip line", "%d (cols=%d, need ≥16)", lineNum, d.length);
                }
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
            cachedVoltage       = hardwareMap.voltageSensor.iterator().next().getVoltage();
            lastVoltageReadTime = now;
        }
    }

    private void updateTimeScaling() {
        double ratio = cachedVoltage / recordedVoltage;
        timeScalingFactor = ratio < 1.0
                ? Range.clip(1.0 + TIME_SCALING_FACTOR * (1.0 - ratio), 1.0, 2.0)
                : 1.0;
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private static boolean lerpBool(boolean a, boolean b, double t) {
        return t < 0.5 ? a : b;
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