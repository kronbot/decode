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

import org.firstinspires.ftc.teamcode.kronbot.Robot;
import org.firstinspires.ftc.teamcode.kronbot.utils.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "RED Auto_Back", group = org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TEST_GROUP)
public class Auto_BackRedOp extends OpMode {

    private Robot robot;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState, launchState;

    // Define poses
    Pose startingPose = coordinates(StartingPoseBackRed);
    Pose launchZone = coordinates(LaunchZoneBack);
    Pose parkZone = coordinates(ParkBack);


    private double motorVel;

    // Paths and PathChains
    private PathChain goToLaunch, goToPark;

    @Override
    public void init() {
        robot = Robot.getInstance();
        robot.initHardware(hardwareMap);

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


    public void buildPaths() {

        goToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(startingPose, launchZone))
                .setLinearHeadingInterpolation(startingPose.getHeading(), launchZone.getHeading())
                .build();

        goToPark = follower.pathBuilder()
                .addPath(new BezierLine(launchZone, parkZone))
                .setLinearHeadingInterpolation(launchZone.getHeading(), parkZone.getHeading())
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


        robot.turretServo.setPosition(0.5);
        robot.angleServo.setPosition(angleServoBack);
        robot.loaderServo.runContinuous(false, false);
    }

    @Override
    public void loop() {
        follower.update();

        motorVel = robot.leftOuttake.getVelocity();

        autonomousPathUpdate();

        Pose currentPose = follower.getPose();
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", currentPose.getX());
        telemetry.addData("Y", currentPose.getY());
        telemetry.addData("Heading (rad)", currentPose.getHeading());
        telemetry.addData("Shooter Motor vel", robot.leftOuttake.getVelocity());

        telemetry.update();
    }

    public void autonomousPathUpdate() {

        switch (pathState) {
            case 0:
                follower.followPath(goToLaunch);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    switch (launchState) {
                        case 0:
                            // Start outtake motors
                            if(pathTimer.getElapsedTimeSeconds() >= 0.5) {
                                robot.outtake.on = true;
                                robot.outtake.velocity = launchSpeedBack;
                                robot.outtake.kS = 0.5; // magic number from constants
                                robot.flap.open = true;
                                launchState++;
                                robot.updateAllSystems();
                                pathTimer.resetTimer();
                            }

                            break;

                        case 1:
                            // Wait for motors to reach speed and launch 1
                            if (motorVel+40 >= launchSpeedBack) {
                                robot.loaderServo.runContinuous(false, true);
                                robot.outtake.angle = 1;
                                launchState++;
                                robot.updateAllSystems();
                                pathTimer.resetTimer();
                            }
                            break;

                        case 2:
                            // Use color sensor to detect when ball is launched and stop servo (+timer for fallback safety)
                            if (pathTimer.getElapsedTimeSeconds() > 3.0) {
                                robot.loaderServo.runContinuous(false, false);
                                launchState++;
                                robot.updateAllSystems();
                                pathTimer.resetTimer();
                            }
                            break;

                        case 3:
                            // Launch 2
                            if (motorVel+40 >= launchSpeedBack) {
                                robot.loaderServo.runContinuous(false, true);
                                launchState++;
                                robot.updateAllSystems();
                                pathTimer.resetTimer();
                            }
                            break;

                        case 4:
                            // Stop servo between shots
                            if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                                robot.loaderServo.runContinuous(false, false);
                                robot.intake.reversed = true;
                                robot.intake.speed = 1;
                                launchState++;
                                robot.updateAllSystems();
                                pathTimer.resetTimer();
                            }
                            break;

                        case 5:
                            // Launch 3
                            if (motorVel+40 >= launchSpeedBack) {
                                robot.loaderServo.runContinuous(false, true);
                                launchState++;
                                robot.updateAllSystems();
                                pathTimer.resetTimer();
                            }
                            break;

                        case 6:
                            // Empty, stop motors
                            if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                                robot.outtake.on = false;
                                robot.intake.speed = 0;
                                robot.loaderServo.runContinuous(false, false);
                                launchState++;
                                robot.updateAllSystems();
                                pathTimer.resetTimer();
                            }
                            break;

                        case 7:
                            // Exit shooting loop
                            if (pathTimer.getElapsedTimeSeconds() >= 3.0) {
                                launchState = 0;
                                setPathState(2);
                            }
                            break;
                    }
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
                Pose finalPose = follower.getPose();
                PoseStorage.savePose(finalPose);

                break;
        }
    }


    public void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void stop() {
    }

}