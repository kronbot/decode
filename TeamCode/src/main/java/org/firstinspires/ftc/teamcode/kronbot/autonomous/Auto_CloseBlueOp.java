package org.firstinspires.ftc.teamcode.kronbot.autonomous;

import static org.firstinspires.ftc.teamcode.kronbot.autonomous.AutonomousConstants.*;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ANGLE_SERVO_CLOSE;


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

@Autonomous(name = "BLUE Auto_Close", group = org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TEST_GROUP)
public class Auto_CloseBlueOp extends OpMode {

    private KronBot robot;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private int launchState;

    // Define poses
    Pose startingPose = coordinates(StartingPoseClose);
    Pose launchZone = coordinates(LaunchZoneCloseBlue);

    Pose launchZone2 = coordinates(LaunchZoneClose2Blue);
    Pose parkZone = coordinates(ParkCloseBlue);


    private double motorVel;

    // Paths and PathChains
    private PathChain goToLaunch, goToPark;

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


    public void buildPaths() {

        goToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(startingPose, launchZone))
                .setLinearHeadingInterpolation(startingPose.getHeading(), launchZone.getHeading())
                .addPath(new BezierLine(launchZone, launchZone2))
                .setLinearHeadingInterpolation(launchZone.getHeading(), launchZone2.getHeading())
                .build();

        goToPark = follower.pathBuilder()
                .addPath(new BezierLine(launchZone2, parkZone))
                .setLinearHeadingInterpolation(launchZone2.getHeading(), parkZone.getHeading())
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
        robot.angleServo.setPosition(angleServoClose);
        robot.loaderServo.runContinuous(false, false);
    }

    @Override
    public void loop() {
        follower.update();

        motorVel = robot.shooterMotor.getVelocity();

        autonomousPathUpdate();

        Pose currentPose = follower.getPose();
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", currentPose.getX());
        telemetry.addData("Y", currentPose.getY());
        telemetry.addData("Heading (rad)", currentPose.getHeading());
        //telemetry.addData("Outtake Alpha", robot.outtakeColor.alpha());
        telemetry.addData("Shooter Motor vel", robot.shooterMotor.getVelocity());

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
                            robot.leftOuttake.setVelocity(launchSpeedClose);
                            robot.rightOuttake.setVelocity(launchSpeedClose);
                            launchState++;
                            pathTimer.resetTimer();
                            break;

                        case 1:
                            // Wait for motors to reach speed and launch 1
                            if (motorVel+100 >= launchSpeedClose && motorVel+100 >= launchSpeedClose && pathTimer.getElapsedTimeSeconds() > 4.0) {
                                robot.loaderServo.runContinuous(false, true);
                                launchState++;
                                pathTimer.resetTimer();
                            }
                            break;

                        case 2:
                            // Use color sensor to detect when ball is launched and stop servo (+timer for fallback safety)
                            if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                                robot.loaderServo.runContinuous(false, false);
                                launchState++;
                                pathTimer.resetTimer();
                            }
                            break;

                        case 3:
                            // Launch 2
                            if (motorVel+100 >= launchSpeedClose && motorVel+100 >= launchSpeedClose) {
                                robot.loaderServo.runContinuous(false, true);
                                launchState++;
                                pathTimer.resetTimer();
                            }
                            break;

                        case 4:
                            // Stop servo between shots
                            if (pathTimer.getElapsedTimeSeconds() > 2.0) {
                                robot.loaderServo.runContinuous(false, false);
                                robot.intakeMotor.setPower(1);
                                launchState++;
                                pathTimer.resetTimer();
                            }
                            break;

                        case 5:
                            // Launch 3
                            if (motorVel+100 >= launchSpeedClose  && motorVel+100 >= launchSpeedClose) {
                                robot.loaderServo.runContinuous(false, true);
                                launchState++;
                                pathTimer.resetTimer();
                            }
                            break;

                        case 6:
                            // Empty, stop motors
                            if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                                robot.shooterMotor.setPower(0);
                                robot.intakeMotor.setPower(0);
                                robot.loaderServo.runContinuous(false, false);
                                launchState++;
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
                // Idle / done
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