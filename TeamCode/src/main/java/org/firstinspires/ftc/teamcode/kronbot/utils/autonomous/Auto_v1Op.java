package org.firstinspires.ftc.teamcode.kronbot.utils.autonomous;

import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.*;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto_v1", group = org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TEST_GROUP)
public class Auto_v1Op extends OpMode {

    private KronBot robot;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // Define poses
    Pose startingPose = coordinates(StartingPose);
    Pose launchZone = coordinates(LaunchZone);
    Pose midPose1 = coordinates(MidPose1);
    Pose midPose2 = coordinates(MidPose2);

    // Paths and PathChains
    private PathChain goToLaunch;
    private PathChain secondPathChain;

    @Override
    public void init() {
        robot = new KronBot();
        robot.initAutonomy(hardwareMap);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        buildPaths();
        follower.setStartingPose(startingPose);

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();
    }

    /** Build all paths for the auto **/
    public void buildPaths() {
        // First PathChain: Go to launch zone
        goToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(startingPose, launchZone))
                .setLinearHeadingInterpolation(startingPose.getHeading(), launchZone.getHeading())
                .build();

        // Second PathChain: midPose1 → midPose2
        secondPathChain = follower.pathBuilder()
                .addPath(new BezierLine(launchZone, midPose1))
                .setLinearHeadingInterpolation(launchZone.getHeading(), midPose1.getHeading())
                .addPath(new BezierLine(midPose1, midPose2))
                .setLinearHeadingInterpolation(midPose1.getHeading(), midPose2.getHeading())
                .build();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Ready to start.");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        Pose currentPose = follower.getPose();
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", currentPose.getX());
        telemetry.addData("Y", currentPose.getY());
        telemetry.addData("Heading (rad)", currentPose.getHeading());
        telemetry.update();
    }

    /** Handles autonomous path progression **/
    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                // Start the first path chain
                follower.followPath(goToLaunch);
                setPathState(1);
                break;

            case 1:
                // Wait until finished with first chain
                if (!follower.isBusy()) {
                    // Example action: shoot or spin motors
                    robot.loaderServo.runContinuous(false, true);
                    robot.leftOuttake.setPower(1);
                    robot.rightOuttake.setPower(1);
                    sleep(2000);
                    robot.leftOuttake.setPower(0);
                    robot.rightOuttake.setPower(0);
                    robot.loaderServo.runContinuous(false, false);
                    sleep(5000);

                    // Start the next chain
                    follower.followPath(secondPathChain);
                    setPathState(2);
                }
                break;

            case 2:
                // Wait until done with second path chain
                if (!follower.isBusy()) {
                    // Example action: maybe retract or park
                    setPathState(-1); // Stop further actions
                }
                break;

            case -1:
                // Idle / done
                break;
        }
    }

    /** Sets new path state and resets timer **/
    public void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void stop() {
    }

}
