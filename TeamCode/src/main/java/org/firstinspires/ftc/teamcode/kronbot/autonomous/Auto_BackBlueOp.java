package org.firstinspires.ftc.teamcode.kronbot.autonomous;

import static org.firstinspires.ftc.teamcode.kronbot.autonomous.AutonomousConstants.*;

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

@Autonomous(name = "BLUE Auto_Back", group = org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TEST_GROUP)
public class Auto_BackBlueOp extends OpMode {

    private KronBot robot;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // Define poses
    Pose startingPoseBack = coordinates(StartingPoseBack);
    Pose launchZoneBack = coordinates(LaunchZoneBackBlue);
    Pose parkBack = coordinates(ParkBackBlue);


    // Paths and PathChains
    private PathChain goToLaunch;
    private PathChain goToPark;

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
        follower.setStartingPose(startingPoseBack);

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();
    }

    /** Build all paths for the auto **/
    public void buildPaths() {

        goToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(startingPoseBack, launchZoneBack))
                .setLinearHeadingInterpolation(startingPoseBack.getHeading(), launchZoneBack.getHeading())
                .build();

        goToPark = follower.pathBuilder()
                .addPath(new BezierLine(launchZoneBack, parkBack))
                .setLinearHeadingInterpolation(launchZoneBack.getHeading(), parkBack.getHeading())
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
                follower.followPath(goToLaunch);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    robot.leftOuttake.setVelocity(launchSpeedBack);
                    robot.rightOuttake.setVelocity(launchSpeedBack);
                    sleep(3000);
                    robot.loaderServo.runContinuous(false, true);
                    sleep(2000);
                    robot.loaderServo.runContinuous(false, false);
                    sleep(4000);
                    robot.loaderServo.runContinuous(false, true);
                    sleep(1000);
                    robot.loaderServo.runContinuous(false, false);
                    sleep(4000);
                    robot.loaderServo.runContinuous(false, true);
                    sleep(1000);
                    robot.leftOuttake.setPower(0);
                    robot.rightOuttake.setPower(0);
                    robot.loaderServo.runContinuous(false, false);
                    sleep(2000);

                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(goToPark);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

            case -1:
                //stop();
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

        telemetry.addLine("stopped");
    }

}
