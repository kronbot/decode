package org.firstinspires.ftc.teamcode.kronbot.autonomous;

import static org.firstinspires.ftc.teamcode.kronbot.autonomous.AutonomousConstants.MAX_CORRECTION_CAP;
import static org.firstinspires.ftc.teamcode.kronbot.autonomous.AutonomousConstants.MIN_CORRECTION_CAP;
import static org.firstinspires.ftc.teamcode.kronbot.autonomous.AutonomousConstants.CORRECTION_ERROR_SCALE;
import static org.firstinspires.ftc.teamcode.kronbot.autonomous.AutonomousConstants.D_FILTER_ALPHA;
import static org.firstinspires.ftc.teamcode.kronbot.autonomous.AutonomousConstants.HEADING_ERROR_SCALE;
import static org.firstinspires.ftc.teamcode.kronbot.autonomous.AutonomousConstants.kD_rotation;
import static org.firstinspires.ftc.teamcode.kronbot.autonomous.AutonomousConstants.kP_rotation;
import static org.firstinspires.ftc.teamcode.kronbot.autonomous.AutonomousConstants.kP_translation;
import static org.firstinspires.ftc.teamcode.kronbot.autonomous.AutonomousConstants.kD_translation;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.kronbot.Robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Replay Autonomous — V3.4 (Feedforward + Odometry Correction)
 *
 * Reads the V3.6 CSV format produced by FinalRecorderOp. Older V3.3-V3.5
 * CSVs still load; missing drive commands and velocities are reconstructed.
 *
 * This version uses the recorded servo positions (turretPos, anglePos)
 * directly during replay instead of re-calculating them with live auto-aim.
 * Drive playback uses recorded TeleOp commands as voltage-compensated
 * feedforward plus field-frame pose and velocity correction.
 */
@Autonomous(name = "Replay Vlad", group = "Replay")
public class FinalReplayOp extends LinearOpMode {

    private final Robot robot = Robot.getInstance();

    // -------------------------------------------------------------------------
    // Configuration
    // -------------------------------------------------------------------------
    private static final String CSV_PATH = "/sdcard/robot_data_Vlad.csv";

    // Playback tuning
    private static final double VOLTAGE_REFRESH_SEC  = 0.10;
    private static final double MIN_VOLTAGE_FF_SCALE = 0.90;
    private static final double MAX_VOLTAGE_FF_SCALE = 1.10;
    private static final double MIN_DT               = 0.008;

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------
    private final List<RobotFrame> recordedFrames = new ArrayList<>();
    private final ElapsedTime runtime = new ElapsedTime();

    private double cachedVoltage   = 13.0;
    private double lastVoltageReadTime = -10;

    private double prevTime;
    private double prevCurrentHeading;
    private double filteredVelErrorX, filteredVelErrorY, filteredOmegaError;

    private int     prevShootRange = -2;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.initFollower(hardwareMap, false);

        telemetry.addData("Status", "Loading CSV...");
        telemetry.update();

        try {
            loadRecordedData();

            if (recordedFrames.isEmpty()) {
                telemetry.addData("ERROR", "CSV is empty!");
                telemetry.update();
                return;
            }

            telemetry.addData("Status", "Loaded %d frames. Ready.", recordedFrames.size());
            telemetry.update();

            // Wait for start
            while (!isStarted() && !isStopRequested()) {
                idle();
            }

            if (isStopRequested()) return;

            // The localizer has already been stationary throughout init. Do
            // not consume 250 ms of the autonomous period with a dead sleep.
            robot.follower.update();
            robot.follower.setPose(new Pose(recordedFrames.get(0).x, recordedFrames.get(0).y, recordedFrames.get(0).heading));
            robot.follower.update();

            applyInitialMechanismState(recordedFrames.get(0));

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
        prevCurrentHeading = robot.follower.getPose().getHeading();
        filteredVelErrorX = 0;
        filteredVelErrorY = 0;
        filteredOmegaError = 0;

        double posErrorSum = 0;
        double posErrorPeak = 0;
        int posErrorSamples = 0;

        robot.follower.startTeleopDrive();

        while (opModeIsActive() && idx < recordedFrames.size() - 1) {
            double now = runtime.seconds();

            refreshVoltage(now);

            // Voltage must not warp trajectory time or mechanism events.
            double targetTs = startTs + now;

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

            double recordedFrameVoltage = lerp(fA.voltage, fB.voltage, t);
            double voltageFfScale = cachedVoltage > 1.0 && recordedFrameVoltage > 1.0
                    ? Range.clip(recordedFrameVoltage / cachedVoltage,
                    MIN_VOLTAGE_FF_SCALE, MAX_VOLTAGE_FF_SCALE)
                    : 1.0;
            double ffFwd = Range.clip(lerp(fA.driveFwd, fB.driveFwd, t) * voltageFfScale, -1.0, 1.0);
            double ffStr = Range.clip(lerp(fA.driveStr, fB.driveStr, t) * voltageFfScale, -1.0, 1.0);
            double ffTurn = Range.clip(lerp(fA.driveTurn, fB.driveTurn, t) * voltageFfScale, -1.0, 1.0);

            double targetVelX = lerp(fA.velocityX, fB.velocityX, t);
            double targetVelY = lerp(fA.velocityY, fB.velocityY, t);
            double targetOmega = lerp(fA.angularVelocity, fB.angularVelocity, t);

            robot.follower.update();
            Pose cur = robot.follower.getPose();
            double curH = cur.getHeading();
            Vector currentVelocity = robot.follower.getVelocity();

            double elapsedDt = prevTime > 0 ? now - prevTime : MIN_DT;

            double exField = targetX - cur.getX();
            double eyField = targetY - cur.getY();
            double eh = normalizeAngle(targetH - curH);

            // Calculate velocity error in the fixed field frame. Differentiating
            // robot-frame position error made pure rotation look like X/Y motion.
            double velErrorX = targetVelX - currentVelocity.getXComponent();
            double velErrorY = targetVelY - currentVelocity.getYComponent();
            double actualOmega = prevTime > 0
                    ? normalizeAngle(curH - prevCurrentHeading) / Math.max(elapsedDt, 1e-6)
                    : targetOmega;
            double omegaError = targetOmega - actualOmega;

            filteredVelErrorX += D_FILTER_ALPHA * (velErrorX - filteredVelErrorX);
            filteredVelErrorY += D_FILTER_ALPHA * (velErrorY - filteredVelErrorY);
            filteredOmegaError += D_FILTER_ALPHA * (omegaError - filteredOmegaError);

            double corrFieldX = exField * kP_translation
                    + filteredVelErrorX * kD_translation;
            double corrFieldY = eyField * kP_translation
                    + filteredVelErrorY * kD_translation;

            // Rotate the completed correction into the robot frame once.
            double cosH = Math.cos(curH);
            double sinH = Math.sin(curH);
            double corrFwd =  cosH * corrFieldX + sinH * corrFieldY;
            double corrStr = -sinH * corrFieldX + cosH * corrFieldY;
            double corrTurn = eh * kP_rotation + filteredOmegaError * kD_rotation;

            double posError = Math.hypot(exField, eyField);
            posErrorSum += posError;
            posErrorSamples++;
            posErrorPeak = Math.max(posErrorPeak, posError);
            double posErrorMean = posErrorSum / posErrorSamples;

            double corrScale = Math.min(posError / CORRECTION_ERROR_SCALE, 1.0);
            double dynamicMaxCorr = lerp(MIN_CORRECTION_CAP, MAX_CORRECTION_CAP, corrScale);

            double corrMag = Math.hypot(corrFwd, corrStr);
            if (corrMag > dynamicMaxCorr) {
                corrFwd *= dynamicMaxCorr / corrMag;
                corrStr *= dynamicMaxCorr / corrMag;
            }
            double headingCorrScale = Math.min(Math.abs(eh) / HEADING_ERROR_SCALE, 1.0);
            double dynamicMaxTurnCorr = lerp(
                    MIN_CORRECTION_CAP, MAX_CORRECTION_CAP, headingCorrScale);
            corrTurn = Range.clip(corrTurn, -dynamicMaxTurnCorr, dynamicMaxTurnCorr);

            // P already goes to zero with error; fading it again allowed small
            // errors to accumulate for most of the replay.
            double robotFwd = ffFwd + corrFwd;
            double robotStr = ffStr + corrStr;
            double robotTurn = ffTurn + corrTurn;

            double driveMag = Math.hypot(robotFwd, robotStr);
            if (driveMag > 1.0) {
                robotFwd /= driveMag;
                robotStr /= driveMag;
            }
            robotTurn = Range.clip(robotTurn, -1.0, 1.0);

            prevCurrentHeading = curH;
            prevTime         = now;

            robot.follower.setTeleOpDrive(robotFwd, robotStr, robotTurn, true);

            applyMechanismCommands(fA, fB, t);
            robot.updateAllSystems();

            // Hardware Overrides: Ensure servos follow the recording exactly, bypassing live auto-aim.
            robot.turretServo.setPosition(lerp(fA.turretPos, fB.turretPos, t));
            robot.angleServo.setPosition(lerp(fA.anglePos, fB.anglePos, t));
            robot.flapsServo.setPosition(fA.flapPos);

            // Telemetry
            telemetry.addData("Time",    "%.2f / %.2f s", now, duration);
            telemetry.addData("PosErr",  "%.2f in",  posError);
            telemetry.addData("PosErr Mean", "%.2f in", posErrorMean);
            telemetry.addData("PosErr Peak", "%.2f in", posErrorPeak);
            telemetry.addData("HeadErr", "%.1f °",   Math.toDegrees(Math .abs(eh)));
            telemetry.addData("FF", "fwd=%.2f str=%.2f turn=%.2f scale=%.2f",
                    ffFwd, ffStr, ffTurn, voltageFfScale);
            telemetry.addData("Corr", "fwd=%.2f str=%.2f turn=%.2f",
                    corrFwd, corrStr, corrTurn);
            telemetry.addData("Cmd", "fwd=%.2f str=%.2f turn=%.2f", robotFwd, robotStr, robotTurn);
            telemetry.addData("Mech",    "rng=%d aa=%s flap=%s",
                    fA.shootRange,
                    fA.autoAim ? "Y" : "N",
                    fA.flapOpen ? "Y" : "N");
            telemetry.update();

            idle();
        }
    }

    // -------------------------------------------------------------------------
    // Mechanism application
    // -------------------------------------------------------------------------
    private void applyInitialMechanismState(RobotFrame f) {
        robot.intake.reversed    = INTAKE_REVERSE;
        robot.turret.driverOffset = f.turretOffset;
        robot.Blue_Target         = f.blueTarget;
        robot.flap.open           = f.flapOpen;
        robot.intake.speed        = f.intakeCmd;
        robot.loader.speed        = f.loaderCmd;

        // Force auto-aim off for Replay; we use recorded positions.
        robot.turret.autoAimEnabled = false;

        if (f.shootRange >= 0) {
            if (f.shootRange == 0) {
                // Recorded as Auto-Aim: get a live kS fallback, then force the
                // recorded interpolated velocity and angle.
                robot.shoot.activateRange(0);
                robot.outtake.activeConfig.velocity = f.leftShtrVel;
                robot.outtake.activeConfig.angle    = f.anglePos;
                if (f.outtakeKs > 0) {
                    robot.outtake.activeConfig.kS = f.outtakeKs;
                }
            } else {
                robot.shoot.activateRange(f.shootRange);
            }
            prevShootRange = f.shootRange;
        } else if (f.shootRange == -1) {
            robot.shoot.deactivate();
            prevShootRange = -1;
        }
    }

    private void applyMechanismCommands(RobotFrame a, RobotFrame b, double t) {
        robot.intake.speed = lerp(a.intakeCmd, b.intakeCmd, t);
        robot.loader.speed = lerp(a.loaderCmd, b.loaderCmd, t);
        // Discrete controls are step functions. Interpolating an enum such as
        // ShootRange can activate ranges the driver never selected.
        robot.flap.open    = a.flapOpen;
        robot.turret.driverOffset = lerp(a.turretOffset, b.turretOffset, t);
        robot.Blue_Target  = a.blueTarget;

        // Ensure auto-aim is off.
        robot.turret.autoAimEnabled = false;

        int curRange = a.shootRange;
        if (curRange == 0) {
            if (curRange != prevShootRange) {
                robot.shoot.activateRange(0);
            }
            // Recorded as Auto-Aim: use recorded velocity/angle after the
            // one-time activation rather than recomputing the range each loop.
            robot.outtake.activeConfig.velocity = lerp(a.leftShtrVel, b.leftShtrVel, t);
            robot.outtake.activeConfig.angle    = lerp(a.anglePos, b.anglePos, t);
            double ks = lerp(a.outtakeKs, b.outtakeKs, t);
            if (ks > 0) {
                robot.outtake.activeConfig.kS = ks;
            }
        } else if (curRange != prevShootRange) {
            if (curRange > 0) {
                robot.shoot.activateRange(curRange);
            } else if (curRange == -1) {
                robot.shoot.deactivate();
            }
        }
        prevShootRange = curRange;
    }

    private void stopMechanisms() {
        robot.intake.speed = 0;
        robot.loader.speed = 0;
        robot.flap.open = false;
        robot.shoot.deactivate();
        robot.updateAllSystems();
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
                if (d.length >= 16) {
                    recordedFrames.add(new RobotFrame(d));
                }
            }
        }
        populateMissingVelocities();
    }

    // V3.4 and earlier recordings do not contain localizer velocity. Preserve
    // compatibility by estimating it in the field frame from neighboring poses.
    private void populateMissingVelocities() {
        for (int i = 0; i < recordedFrames.size(); i++) {
            RobotFrame frame = recordedFrames.get(i);
            if ((frame.hasRecordedVelocity && frame.hasRecordedAngularVelocity)
                    || recordedFrames.size() < 2) continue;

            int beforeIndex = Math.max(0, i - 1);
            int afterIndex = Math.min(recordedFrames.size() - 1, i + 1);
            RobotFrame before = recordedFrames.get(beforeIndex);
            RobotFrame after = recordedFrames.get(afterIndex);
            double dt = after.timestamp - before.timestamp;
            if (dt > 0) {
                if (!frame.hasRecordedVelocity) {
                    frame.velocityX = (after.x - before.x) / dt;
                    frame.velocityY = (after.y - before.y) / dt;
                }
                if (!frame.hasRecordedAngularVelocity) {
                    frame.angularVelocity = normalizeAngle(after.heading - before.heading) / dt;
                }
            }
        }
    }

    private void refreshVoltage(double now) {
        if (now - lastVoltageReadTime >= VOLTAGE_REFRESH_SEC) {
            cachedVoltage       = hardwareMap.voltageSensor.iterator().next().getVoltage();
            lastVoltageReadTime = now;
        }
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

    private static class RobotFrame {
        double timestamp;
        double lrPow, rrPow, lfPow, rfPow;
        double x, y, heading;
        double voltage;
        double intakeVel, loaderVel, leftShtrVel, rightShtrVel;
        double turretPos, anglePos, flapPos;
        double intakeCmd    = 0;
        double loaderCmd    = 0;
        boolean flapOpen    = false;
        int     shootRange  = -2;
        double  turretOffset = 0;
        boolean blueTarget  = false;
        boolean autoAim     = false;
        double driveFwd     = 0;
        double driveStr     = 0;
        double driveTurn    = 0;
        double outtakeKs    = 0;
        double velocityX    = 0;
        double velocityY    = 0;
        double angularVelocity = 0;
        boolean hasRecordedVelocity = false;
        boolean hasRecordedAngularVelocity = false;

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
            if (d.length >= 22) {
                intakeCmd    = Double.parseDouble(d[16]);
                loaderCmd    = Double.parseDouble(d[17]);
                flapOpen     = d[18].equals("1") || d[18].equalsIgnoreCase("true");
                shootRange   = (int) Math.round(Double.parseDouble(d[19]));
                turretOffset = Double.parseDouble(d[20]);
                blueTarget   = d[21].equals("1") || d[21].equalsIgnoreCase("true");
            }
            if (d.length >= 23) {
                autoAim = d[22].equals("1") || d[22].equalsIgnoreCase("true");
            }
            inferDriveCommandsFromWheelPowers();
            if (d.length >= 26) {
                driveFwd  = Double.parseDouble(d[23]);
                driveStr  = Double.parseDouble(d[24]);
                driveTurn = Double.parseDouble(d[25]);
            }
            if (d.length >= 27) {
                outtakeKs = Double.parseDouble(d[26]);
            }
            if (d.length >= 29) {
                velocityX = Double.parseDouble(d[27]);
                velocityY = Double.parseDouble(d[28]);
                hasRecordedVelocity = true;
            }
            if (d.length >= 30) {
                angularVelocity = Double.parseDouble(d[29]);
                hasRecordedAngularVelocity = true;
            }
        }

        private void inferDriveCommandsFromWheelPowers() {
            driveFwd = (lfPow + rfPow + lrPow + rrPow) / 4.0;
            driveStr = (lfPow - rfPow - lrPow + rrPow) / 4.0;
            driveTurn = (lfPow - rfPow + lrPow - rrPow) / 4.0;
        }
    }
}
