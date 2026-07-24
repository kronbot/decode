package org.firstinspires.ftc.teamcode.kronbot.autonomous;

import static org.firstinspires.ftc.teamcode.kronbot.autonomous.AutonomousConstants.BLEND_K;
import static org.firstinspires.ftc.teamcode.kronbot.autonomous.AutonomousConstants.MAX_CORRECTION_CAP;
import static org.firstinspires.ftc.teamcode.kronbot.autonomous.AutonomousConstants.MIN_CORRECTION_CAP;
import static org.firstinspires.ftc.teamcode.kronbot.autonomous.AutonomousConstants.CORRECTION_ERROR_SCALE;
import static org.firstinspires.ftc.teamcode.kronbot.autonomous.AutonomousConstants.D_FILTER_ALPHA;
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
 * Reads the V3.4 CSV format produced by FinalRecorderOp. Older V3.3 CSVs
 * still load; drive feedforward is approximated from recorded wheel powers.
 *
 * This version uses the recorded servo positions (turretPos, anglePos)
 * directly during replay instead of re-calculating them with live auto-aim.
 * Drive playback uses the recorded TeleOp commands as feedforward and blends
 * in Pedro odometry PD correction when the robot drifts from the recording.
 */
@Autonomous(name = "Replay Vlad", group = "Replay")
public class FinalReplayOp extends LinearOpMode {

    private final Robot robot = Robot.getInstance();

    // -------------------------------------------------------------------------
    // Configuration
    // -------------------------------------------------------------------------
    private static final String CSV_PATH = "/sdcard/robot_data_Vlad.csv";

    // Playback tuning
    private static final double LOOKAHEAD_TIME       = 0.080; // 80ms lookahead while moving
    private static final double LOOKAHEAD_DISABLE_THRESH = 0.04;
    private static final double VOLTAGE_REFRESH_SEC  = 0.10;
    private static final double TIME_SCALING_FACTOR  = 0.5;   // adjust replay speed for voltage drops

    private static final double MAX_SLEW_RATE        = 5.0;
    private static final double MIN_DT               = 0.008;
    private static final double MAX_DT               = 0.050;

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------
    private final List<RobotFrame> recordedFrames = new ArrayList<>();
    private final ElapsedTime runtime = new ElapsedTime();

    private double recordedVoltage = 13.0;
    private double cachedVoltage   = 13.0;
    private double lastVoltageReadTime = -10;
    private double timeScalingFactor   = 1.0;

    private double prevTime;
    private double prevErrorX, prevErrorY, prevErrorHeading;
    private double filteredDx, filteredDy, filteredDh;

    private double prevRobotFwd, prevRobotStr, prevRobotTurn;

    private int     prevShootRange = -2;
    private boolean prevAutoAim    = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.initFollower(hardwareMap, false);

        telemetry.addData("Status", "Loading CSV...");
        telemetry.update();

        try {
            loadRecordedData();
            calculateRecordedVoltage();

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

            // Settle localizer
            robot.follower.update();
            sleep(250);
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
        prevErrorX       = 0;
        prevErrorY       = 0;
        prevErrorHeading = 0;
        filteredDx       = 0;
        filteredDy       = 0;
        filteredDh       = 0;
        prevShootRange   = -2;
        prevAutoAim      = false;

        prevRobotFwd     = 0;
        prevRobotStr     = 0;
        prevRobotTurn    = 0;

        double posErrorSum = 0;
        double posErrorPeak = 0;
        int posErrorSamples = 0;

        robot.follower.startTeleopDrive();

        while (opModeIsActive() && idx < recordedFrames.size() - 1) {
            double now = runtime.seconds();

            refreshVoltage(now);
            updateTimeScaling();

            double recordingTime = now / timeScalingFactor;
            RobotFrame currentBaseFrame = recordedFrames.get(Math.min(idx, recordedFrames.size() - 1));
            double currentInputMag = Math.abs(currentBaseFrame.driveFwd)
                    + Math.abs(currentBaseFrame.driveStr)
                    + Math.abs(currentBaseFrame.driveTurn);
            double lookahead = currentInputMag > LOOKAHEAD_DISABLE_THRESH ? LOOKAHEAD_TIME : 0.0;
            recordingTime += lookahead;
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

            double ffFwd = Range.clip(lerp(fA.driveFwd, fB.driveFwd, t), -1.0, 1.0);
            double ffStr = Range.clip(lerp(fA.driveStr, fB.driveStr, t), -1.0, 1.0);
            double ffTurn = Range.clip(lerp(fA.driveTurn, fB.driveTurn, t), -1.0, 1.0);

            robot.follower.update();
            Pose cur = robot.follower.getPose();
            double curH = cur.getHeading();

            double dt = Range.clip(now - prevTime, MIN_DT, MAX_DT);

            double exField = targetX - cur.getX();
            double eyField = targetY - cur.getY();
            double eh = normalizeAngle(targetH - curH);

            double cosH = Math.cos(curH);
            double sinH = Math.sin(curH);
            double exRobot =  cosH * exField + sinH * eyField;
            double eyRobot = -sinH * exField + cosH * eyField;

            double rawDx = (exRobot - prevErrorX) / dt;
            double rawDy = (eyRobot - prevErrorY) / dt;
            double rawDh = (eh - prevErrorHeading) / dt;
            filteredDx = filteredDx + D_FILTER_ALPHA * (rawDx - filteredDx);
            filteredDy = filteredDy + D_FILTER_ALPHA * (rawDy - filteredDy);
            filteredDh = filteredDh + D_FILTER_ALPHA * (rawDh - filteredDh);

            double corrFwd = exRobot * kP_translation + filteredDx * kD_translation;
            double corrStr = eyRobot * kP_translation + filteredDy * kD_translation;
            double corrTurn = eh * kP_rotation + filteredDh * kD_rotation;

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
            corrTurn = Range.clip(corrTurn, -dynamicMaxCorr, dynamicMaxCorr);

            double corrWeight = Range.clip(1.0 - Math.exp(-BLEND_K * posError), 0.0, 1.0);

            double robotFwd = ffFwd + corrWeight * corrFwd;
            double robotStr = ffStr + corrWeight * corrStr;
            double robotTurn = ffTurn + corrWeight * corrTurn;

            double driveMag = Math.hypot(robotFwd, robotStr);
            if (driveMag > 1.0) {
                robotFwd /= driveMag;
                robotStr /= driveMag;
            }
            robotTurn = Range.clip(robotTurn, -1.0, 1.0);

            double maxDelta = MAX_SLEW_RATE * dt;
            robotFwd = prevRobotFwd + Range.clip(robotFwd - prevRobotFwd, -maxDelta, maxDelta);
            robotStr = prevRobotStr + Range.clip(robotStr - prevRobotStr, -maxDelta, maxDelta);
            robotTurn = prevRobotTurn + Range.clip(robotTurn - prevRobotTurn, -maxDelta, maxDelta);

            prevRobotFwd     = robotFwd;
            prevRobotStr     = robotStr;
            prevRobotTurn    = robotTurn;
            prevErrorX       = exRobot;
            prevErrorY       = eyRobot;
            prevErrorHeading = eh;
            prevTime         = now;

            robot.follower.setTeleOpDrive(robotFwd, robotStr, robotTurn, true);

            applyMechanismCommands(fA, fB, t);
            robot.updateAllSystems();

            // Hardware Overrides: Ensure servos follow the recording exactly, bypassing live auto-aim.
            robot.turretServo.setPosition(lerp(fA.turretPos, fB.turretPos, t));
            robot.angleServo.setPosition(lerp(fA.anglePos, fB.anglePos, t));
            robot.flapsServo.setPosition(lerp(fA.flapPos, fB.flapPos, t));

            // Telemetry
            telemetry.addData("Time",    "%.2f / %.2f s", now, duration);
            telemetry.addData("Lookahead", "%.3f s", lookahead);
            telemetry.addData("PosErr",  "%.2f cm",  posError);
            telemetry.addData("PosErr Mean", "%.2f cm", posErrorMean);
            telemetry.addData("PosErr Peak", "%.2f cm", posErrorPeak);
            telemetry.addData("HeadErr", "%.1f °",   Math.toDegrees(Math .abs(eh)));
            telemetry.addData("FF", "fwd=%.2f str=%.2f turn=%.2f", ffFwd, ffStr, ffTurn);
            telemetry.addData("Corr", "fwd=%.2f str=%.2f turn=%.2f w=%.0f%%",
                    corrFwd, corrStr, corrTurn, corrWeight * 100);
            telemetry.addData("Cmd", "fwd=%.2f str=%.2f turn=%.2f", robotFwd, robotStr, robotTurn);
            telemetry.addData("Mech",    "rng=%d aa=%s flap=%s",
                    (int) Math.round(lerp(fA.shootRange, fB.shootRange, t)),
                    (t < 0.5 ? fA.autoAim : fB.autoAim) ? "Y" : "N",
                    lerpBool(fA.flapOpen, fB.flapOpen, t) ? "Y" : "N");
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
        prevAutoAim = false;

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
        robot.flap.open    = lerpBool(a.flapOpen, b.flapOpen, t);
        robot.turret.driverOffset = lerp(a.turretOffset, b.turretOffset, t);
        robot.Blue_Target  = t < 0.5 ? a.blueTarget : b.blueTarget;

        // Ensure auto-aim is off.
        robot.turret.autoAimEnabled = false;

        int curRange = (int) Math.round(lerp(a.shootRange, b.shootRange, t));
        if (curRange == 0) {
            // Recorded as Auto-Aim: live kS fallback, recorded velocity/angle.
            robot.shoot.activateRange(0);
            robot.outtake.activeConfig.velocity = lerp(a.leftShtrVel, b.leftShtrVel, t);
            robot.outtake.activeConfig.angle    = lerp(a.anglePos, b.anglePos, t);
            double ks = lerp(a.outtakeKs, b.outtakeKs, t);
            if (ks > 0) {
                robot.outtake.activeConfig.kS = ks;
            }
        } else if (curRange != prevShootRange) {
            if (curRange > 0) {
                robot.shoot.activateRange(curRange);
            } else if (curRange == -1 && prevShootRange > 0) {
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
        }

        private void inferDriveCommandsFromWheelPowers() {
            driveFwd = (lfPow + rfPow + lrPow + rrPow) / 4.0;
            driveStr = (lfPow - rfPow - lrPow + rrPow) / 4.0;
            driveTurn = (lfPow - rfPow + lrPow - rrPow) / 4.0;
        }
    }
}
