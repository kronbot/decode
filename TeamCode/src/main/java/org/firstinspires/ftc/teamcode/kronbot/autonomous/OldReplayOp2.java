package org.firstinspires.ftc.teamcode.kronbot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import android.os.Environment;
import org.firstinspires.ftc.teamcode.kronbot.Robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

/**
 * Replay autonomous that follows a path recorded by DataRecordingOp2.
 *
 * Simplified approach (matching the old working ImprovedAuto pattern):
 *  - PedroPathing follower is used ONLY for odometry (getPoseTracker().update()).
 *    We never call follower.update() because that also drives the motors.
 *  - We set the starting pose to the RECORDED start pose, so all coordinates are
 *    in the same absolute frame. No coordinate transforms needed.
 *  - A simple PD controller drives toward each interpolated target pose.
 *  - Mecanum mixing converts robot-relative PD output to wheel powers.
 */
@Autonomous(name = "Drive & Mech Replay", group = "Replay")
public class OldReplayOp2 extends LinearOpMode {

    private static final String CSV_PATH = Environment.getExternalStorageDirectory().getPath() + "/robot_data.csv";

    // PD Constants
    private static final double kP_translation  = 0.08;
    private static final double kD_translation  = 0.02;
    private static final double kP_rotation     = 1.5;
    private static final double kD_rotation     = 0.1;
    private static final double MAX_POWER       = 0.8;
    private static final double TIME_SCALING_FACTOR = 0.7;

    private Robot robot;
    private Follower follower;
    private final List<ReplayFrame> recordedFrames = new ArrayList<>();
    private final ElapsedTime runtime = new ElapsedTime();

    // PD state
    private double prevErrorX = 0, prevErrorY = 0, prevErrorHeading = 0, prevTime = 0;

    // Battery compensation
    private double recordedVoltageAvg = 12.0;
    private double timeScalingFactor = 1.0;

    // Drive motors
    private DcMotorEx leftFront, rightFront, leftRear, rightRear;

    /**
     * A single recorded frame.
     * Heading is in RADIANS (from follower.getHeading() in DataRecordingOp2).
     */
    private static class ReplayFrame {
        double timestamp;
        double x, y, headingRad;
        double voltage;

        // Mechanism data
        double intakePwr, loaderPwr, leftShtrPwr, rightShtrPwr;
        double turretPos, anglePos, flapPos;

        ReplayFrame(String[] data) {
            // CSV: Time,LR,RR,LF,RF,X,Y,Heading,Voltage,IntakePwr,LoaderPwr,LeftShtrPwr,RightShtrPwr,TurretPos,AnglePos,FlapPos
            this.timestamp  = Double.parseDouble(data[0]);
            this.x          = Double.parseDouble(data[5]);
            this.y          = Double.parseDouble(data[6]);
            this.headingRad = Double.parseDouble(data[7]); // radians
            this.voltage    = Double.parseDouble(data[8]);

            if (data.length > 9) {
                this.intakePwr    = Double.parseDouble(data[9]);
                this.loaderPwr    = Double.parseDouble(data[10]);
                this.leftShtrPwr  = Double.parseDouble(data[11]);
                this.rightShtrPwr = Double.parseDouble(data[12]);
                this.turretPos    = Double.parseDouble(data[13]);
                this.anglePos     = Double.parseDouble(data[14]);
                this.flapPos      = Double.parseDouble(data[15]);
            }
        }
    }

    @Override
    public void runOpMode() {
        robot = Robot.getInstance();
        robot.initFollower(hardwareMap, true);  // add this
        robot.init(hardwareMap);
        follower = robot.follower;

        // Get drive motors and match PedroPathing's MecanumConstants directions.
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear   = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear  = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (!loadRecordedData()) {
            telemetry.addData("Error", "Failed to load data from " + CSV_PATH);
            telemetry.update();
            return;
        }

        // Set the starting pose to the RECORDED start pose so both coordinate
        // systems match — no transform needed during playback.
        ReplayFrame startFrame = recordedFrames.get(0);
        follower.setStartingPose(new Pose(startFrame.x, startFrame.y, startFrame.headingRad));

        telemetry.addData("Status", "Ready to Replay (Drive + Mechs)");
        telemetry.addData("Frames", recordedFrames.size());
        telemetry.addData("Duration", "%.2f s",
                recordedFrames.get(recordedFrames.size() - 1).timestamp - startFrame.timestamp);
        telemetry.addData("Start Pose", "(%.1f, %.1f) h=%.1f°",
                startFrame.x, startFrame.y, Math.toDegrees(startFrame.headingRad));
        telemetry.addData("Recorded Voltage", "%.1f V", recordedVoltageAvg);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        runtime.reset();
        prevTime = 0;
        executePlayback();
    }

    private void executePlayback() {
        double startTs = recordedFrames.get(0).timestamp;
        double endTs = recordedFrames.get(recordedFrames.size() - 1).timestamp;
        double duration = endTs - startTs;
        int idx = 0;

        while (opModeIsActive() && idx < recordedFrames.size() - 1) {
            double now = runtime.seconds();
            updateTimeScaling();
            double adjustedTime = now / timeScalingFactor;
            double targetTs = startTs + adjustedTime;

            // Advance to the correct frame
            while (idx < recordedFrames.size() - 1 && recordedFrames.get(idx + 1).timestamp <= targetTs) {
                idx++;
            }
            if (idx >= recordedFrames.size() - 1) break;

            // Interpolate between frame[idx] and frame[idx+1]
            ReplayFrame frameA = recordedFrames.get(idx);
            ReplayFrame frameB = recordedFrames.get(idx + 1);
            double segDur = frameB.timestamp - frameA.timestamp;
            double t = (segDur > 0.0001)
                    ? Range.clip((targetTs - frameA.timestamp) / segDur, 0.0, 1.0)
                    : 0.0;

            double targetX       = lerp(frameA.x, frameB.x, t);
            double targetY       = lerp(frameA.y, frameB.y, t);
            double targetHeading = lerpAngle(frameA.headingRad, frameB.headingRad, t);

            // Update odometry only (NOT follower.update() which also drives motors)
            follower.getPoseTracker().update();
            Pose curPose = follower.getPose();
            double curHeading = curPose.getHeading();

            // PD drive control — all in absolute coordinates, no transform needed
            applyDrivePD(curPose.getX(), curPose.getY(), curHeading,
                    targetX, targetY, targetHeading, now);

            // Replay mechanism states
            setMechanismStates(frameA, frameB, t);

            // Telemetry
            double posErr  = Math.hypot(targetX - curPose.getX(), targetY - curPose.getY());
            double headErr = Math.toDegrees(Math.abs(normalizeAngle(targetHeading - curHeading)));
            double curVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

            telemetry.addData("Time",    "%.2f/%.2f s (%.2fx)", now, duration * timeScalingFactor, timeScalingFactor);
            telemetry.addData("Frame",   "%d/%d (lerp %.2f)", idx, recordedFrames.size(), t);
            telemetry.addData("PosErr",  "%.2f in", posErr);
            telemetry.addData("HeadErr", "%.1f°", headErr);
            telemetry.addData("Target",  "(%.1f, %.1f) h=%.1f°", targetX, targetY, Math.toDegrees(targetHeading));
            telemetry.addData("Current", "(%.1f, %.1f) h=%.1f°", curPose.getX(), curPose.getY(), Math.toDegrees(curHeading));
            telemetry.addData("Voltage", "%.1f V (rec: %.1f V)", curVoltage, recordedVoltageAvg);
            telemetry.update();

            idle();
        }
        stopRobot();
        requestOpModeStop();
    }

    /**
     * PD controller — field-relative error rotated into robot frame, then mecanum mixed.
     * Same structure as the old working ImprovedAuto.
     */
    private void applyDrivePD(double curX, double curY, double curHeading,
                              double tgtX, double tgtY, double tgtHeading,
                              double now) {
        double errorX = tgtX - curX;
        double errorY = tgtY - curY;
        double dt = now - prevTime;

        double dX = (dt > 0) ? (errorX - prevErrorX) / dt : 0;
        double dY = (dt > 0) ? (errorY - prevErrorY) / dt : 0;

        // Field-relative PD output
        double fieldX = errorX * kP_translation + dX * kD_translation;
        double fieldY = errorY * kP_translation + dY * kD_translation;

        // Rotate into robot-relative frame
        double cos = Math.cos(curHeading);
        double sin = Math.sin(curHeading);
        double robotX =  cos * fieldX + sin * fieldY;
        double robotY = -sin * fieldX + cos * fieldY;

        // Clamp magnitude
        double mag = Math.hypot(robotX, robotY);
        if (mag > MAX_POWER) {
            robotX *= MAX_POWER / mag;
            robotY *= MAX_POWER / mag;
        }

        prevErrorX = errorX;
        prevErrorY = errorY;
        prevTime = now;

        // Heading PD
        double errorH = normalizeAngle(tgtHeading - curHeading);
        double dH = (dt > 0) ? (errorH - prevErrorHeading) / dt : 0;
        double rotPower = Range.clip(errorH * kP_rotation + dH * kD_rotation, -MAX_POWER, MAX_POWER);
        prevErrorHeading = errorH;

        // Mecanum mixing: robotY = forward/back, robotX = strafe, rotPower = turn
        double fl = robotY + robotX + rotPower;
        double fr = robotY - robotX - rotPower;
        double bl = robotY - robotX + rotPower;
        double br = robotY + robotX - rotPower;

        // Normalize so no wheel exceeds 1.0
        double maxPwr = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        leftFront.setPower(fl / maxPwr);
        rightFront.setPower(fr / maxPwr);
        leftRear.setPower(bl / maxPwr);
        rightRear.setPower(br / maxPwr);
    }

    /**
     * Set mechanism outputs. Servo positions are interpolated; motor powers snap to nearest frame.
     */
    private void setMechanismStates(ReplayFrame a, ReplayFrame b, double t) {
        ReplayFrame src = (t < 0.5) ? a : b;
        if (robot.intakeMotor  != null) robot.intakeMotor.setPower(src.intakePwr);
        if (robot.loaderMotor  != null) robot.loaderMotor.setPower(src.loaderPwr);
        if (robot.leftOuttake  != null) robot.leftOuttake.setPower(src.leftShtrPwr);
        if (robot.rightOuttake != null) robot.rightOuttake.setPower(src.rightShtrPwr);

        if (robot.turretServo != null) robot.turretServo.setPosition(lerp(a.turretPos, b.turretPos, t));
        if (robot.angleServo  != null) robot.angleServo.setPosition(lerp(a.anglePos,  b.anglePos,  t));
        if (robot.flapsServo  != null) robot.flapsServo.setPosition(lerp(a.flapPos,   b.flapPos,   t));
    }

    private void updateTimeScaling() {
        double currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        double voltageRatio = currentVoltage / recordedVoltageAvg;

        if (voltageRatio < 1.0) {
            timeScalingFactor = 1.0 + TIME_SCALING_FACTOR * (1.0 - voltageRatio);
        } else {
            timeScalingFactor = 1.0;
        }
        timeScalingFactor = Range.clip(timeScalingFactor, 1.0, 2.0);
    }

    private boolean loadRecordedData() {
        File file = new File(CSV_PATH);
        if (!file.exists()) return false;

        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
            br.readLine(); // Skip header
            double totalV = 0;
            int count = 0;
            String line;
            while ((line = br.readLine()) != null) {
                String[] parts = line.split(",");
                if (parts.length >= 16) {
                    ReplayFrame frame = new ReplayFrame(parts);
                    recordedFrames.add(frame);
                    if (frame.voltage > 5) { totalV += frame.voltage; count++; }
                }
            }
            if (count > 0) recordedVoltageAvg = totalV / count;
            return !recordedFrames.isEmpty();
        } catch (Exception e) {
            return false;
        }
    }

    private void stopRobot() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        if (robot.intakeMotor  != null) robot.intakeMotor.setPower(0);
        if (robot.loaderMotor  != null) robot.loaderMotor.setPower(0);
        if (robot.leftOuttake  != null) robot.leftOuttake.setPower(0);
        if (robot.rightOuttake != null) robot.rightOuttake.setPower(0);
    }

    // ---- Utility helpers ----

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private static double lerpAngle(double a, double b, double t) {
        double diff = normalizeAngle(b - a);
        return normalizeAngle(a + diff * t);
    }

    private static double normalizeAngle(double angle) {
        while (angle >  Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}