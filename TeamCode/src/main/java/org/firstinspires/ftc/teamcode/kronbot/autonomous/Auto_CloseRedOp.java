package org.firstinspires.ftc.teamcode.kronbot.autonomous;

import static org.firstinspires.ftc.teamcode.kronbot.autonomous.AutonomousConstants.*;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ANGLE_SERVO_CLOSE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.FLAP_OPEN;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.Robot;
import org.firstinspires.ftc.teamcode.kronbot.utils.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "RED Auto_Close", group = org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TEST_GROUP)
public class Auto_CloseRedOp extends OpMode {

    private Robot robot = Robot.getInstance();
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private int launchState;

    // Define poses
    Pose start = coordinates(StartingPoseCloseRed);
    Pose launch1 = coordinates(LaunchZoneClose1);
    Pose launch2 = coordinates(LaunchZoneClose2);
    Pose intake1 = coordinates(IntakeZoneClose1);
    Pose load1 = coordinates(LoadZoneClose1);

    Pose parkZone = coordinates(ParkClose);


    private double motorVel;

    // Paths and PathChains
    private PathChain goToLaunch1, goToLaunch2, goToIntake1, goToLoad1, goToPark;

    @Override
    public void init() {

        robot.init(hardwareMap);


        robot.initFollower(hardwareMap, start);
        robot.flapsServo.setPosition(FLAP_OPEN);
        robot.angleServo.setPosition(ANGLE_SERVO_CLOSE);
        robot.turretServo.setPosition(0.7);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        buildPaths();

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();
    }


    public void buildPaths() {

        goToLaunch1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(start, launch1))
                .setLinearHeadingInterpolation(start.getHeading(), launch1.getHeading())
                .build();

        goToIntake1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(launch1, intake1))
                .setLinearHeadingInterpolation(launch1.getHeading(), intake1.getHeading())
                .build();

        goToLoad1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(intake1, load1))
                .setLinearHeadingInterpolation(intake1.getHeading(), load1.getHeading())
                .build();

        goToLaunch2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(load1, launch2))
                .setLinearHeadingInterpolation(load1.getHeading(), launch2.getHeading())
                .build();


        goToPark = robot.follower.pathBuilder()
                .addPath(new BezierLine(start, parkZone))
                .setLinearHeadingInterpolation(start.getHeading(), parkZone.getHeading())
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
        robot.follower.update();

        motorVel = robot.leftOuttake.getVelocity();

        autonomousPathUpdate();

        Pose currentPose = robot.follower.getPose();
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", currentPose.getX());
        telemetry.addData("Y", currentPose.getY());
        telemetry.addData("Heading (rad)", currentPose.getHeading());
        //telemetry.addData("Outtake Alpha", robot.outtakeColor.alpha());
        telemetry.addData("Shooter Motor vel", robot.leftOuttake.getVelocity());

        telemetry.update();
    }

    public void autonomousPathUpdate() {

        switch (pathState) {
            case 0:
                //also start motors to save time
                robot.leftOuttake.setVelocity(launchSpeedClose);
                robot.rightOuttake.setVelocity(launchSpeedClose);
                //go to pose
                robot.follower.followPath(goToLaunch1);
                setPathState(1);
                break;

            case 1:
                if (!robot.follower.isBusy()) {
                    switch (launchState) {
                        case 0:
                            // Start outtake motors
                            launchState++;
                            pathTimer.resetTimer();
                            break;

                        case 1:
                            // Wait for motors to reach speed and launch first 2
                            if (motorVel + 100 >= launchSpeedClose && motorVel + 100 >= launchSpeedClose && pathTimer.getElapsedTimeSeconds() > 2.0) {
                                robot.loaderMotor.setPower(1);
                                launchState++;
                                pathTimer.resetTimer();
                            }
                            break;

                        case 2:
                            // intake on and launch the third
                            if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                                robot.intakeMotor.setPower(1);
                                launchState++;
                                pathTimer.resetTimer();
                            }
                            break;

                        case 3:
                            // timer to see when all 3 are launched
                            if (pathTimer.getElapsedTimeSeconds() > 2.0) {
                                robot.intakeMotor.setPower(0);
                                robot.loaderMotor.setPower(0);
                                launchState = -1;
                                pathTimer.resetTimer();
                            }
                            break;
                        case -1:
                            break;
                    }
                }
            case 2:
                if (!robot.follower.isBusy()) {
                    robot.follower.followPath(goToIntake1);
                    robot.intakeMotor.setPower(1);
                    robot.loaderMotor.setPower(1);
                    setPathState(3);
                }
                break;

           case 3:
                if (!robot.follower.isBusy() && pathTimer.getElapsedTimeSeconds()>2) {
                    robot.follower.followPath(goToLoad1);
                    if (!robot.follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1) {
                        robot.intakeMotor.setPower(0);
                        robot.loaderMotor.setPower(0);
                    }
                }
                break;

            case 4:
                if (!robot.follower.isBusy()) {
                    robot.follower.followPath(goToLaunch2);
                }
                break;

            case 5:
                if (!robot.follower.isBusy()) {
                    switch (launchState) {
                        case 0:
                            // Start outtake motors
                            launchState++;
                            pathTimer.resetTimer();
                            break;

                        case 1:
                            // Wait for motors to reach speed and launch first 2
                            if (motorVel + 100 >= launchSpeedClose && motorVel + 100 >= launchSpeedClose && pathTimer.getElapsedTimeSeconds() > 2.0) {
                                robot.loaderMotor.setPower(1);
                                launchState++;
                                pathTimer.resetTimer();
                            }
                            break;

                        case 2:
                            // intake on and launch the third
                            if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                                robot.intakeMotor.setPower(1);
                                launchState++;
                                pathTimer.resetTimer();
                            }
                            break;

                        case 3:
                            // timer to see when all 3 are launched
                            if (pathTimer.getElapsedTimeSeconds() > 2.0) {
                                robot.intakeMotor.setPower(0);
                                robot.loaderMotor.setPower(0);
                                launchState = -1;
                                pathTimer.resetTimer();
                            }
                            break;
                        case -1:
                            break;
                    }
                }

            case -1:
                Pose finalPose = robot.follower.getPose();
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