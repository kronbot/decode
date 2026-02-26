package org.firstinspires.ftc.teamcode.kronbot.autonomous;

import static org.firstinspires.ftc.teamcode.kronbot.autonomous.AutonomousConstants.*;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ANGLE_SERVO_CLOSE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.FLAP_CLOSED;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.FLAP_OPEN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_2_KS;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_2_VELOCITY;


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
    private int launchState=0;

    // Define poses
    Pose start = coordinates(StartingPoseCloseRed);
    Pose launch1 = coordinates(LaunchZoneClose1);
    Pose launch11 = coordinates(LaunchZoneClose11);
    Pose launch2 = coordinates(LaunchZoneClose2);
    Pose intake1 = coordinates(IntakeZoneClose1);
    Pose intake11 = coordinates(IntakeZoneClose11);

    Pose intake2 = coordinates(IntakeZoneClose2);
    Pose intake22 = coordinates(IntakeZoneClose22);
    Pose launch3 = coordinates(LaunchZoneClose3);

    Pose parkZone = coordinates(ParkClose);


    private double motorVel;

    // Paths and PathChains
    private PathChain goToLaunch1, goToLaunch11, goToLaunch2, goToLaunch3, goToIntake1, goToIntake11, goToIntake2, goToIntake22, goToPark;

    @Override
    public void init() {

        robot.init(hardwareMap);


        robot.initFollower(hardwareMap, start);

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

        goToLaunch11 = robot.follower.pathBuilder()
                .addPath(new BezierLine(launch1, launch11))
                .setLinearHeadingInterpolation(launch1.getHeading(), launch11.getHeading())
                .build();

        goToIntake1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(launch11, intake1))
                .setLinearHeadingInterpolation(launch11.getHeading(), intake1.getHeading())
                .build();
        goToIntake11 = robot.follower.pathBuilder()
                .addPath(new BezierLine(intake1, intake11))
                .setLinearHeadingInterpolation(intake1.getHeading(), intake11.getHeading())
                .setBrakingStrength(0.2)
                .build();

        goToLaunch2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(intake11, launch2))
                .setLinearHeadingInterpolation(intake11.getHeading(), launch2.getHeading())
                .build();


        goToIntake2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(launch2, intake2))
                .setLinearHeadingInterpolation(launch2.getHeading(), intake2.getHeading())
                .build();
        goToIntake22 = robot.follower.pathBuilder()
                .addPath(new BezierLine(intake2, intake22))
                .setLinearHeadingInterpolation(intake2.getHeading(), intake22.getHeading())
                .setBrakingStrength(0.2)
                .build();
        goToLaunch3 = robot.follower.pathBuilder()
                .addPath(new BezierLine(intake22, launch3))
                .setLinearHeadingInterpolation(intake22.getHeading(), launch3.getHeading())
                .build();

        goToPark = robot.follower.pathBuilder()
                .addPath(new BezierLine(launch3, parkZone))
                .setLinearHeadingInterpolation(launch3.getHeading(), parkZone.getHeading())
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

        robot.turretServo.setPosition(0);
        robot.angleServo.setPosition(angleServoClose);
        robot.flapsServo.setPosition(FLAP_OPEN);
        robot.turretServo.setPosition(0.46);
        robot.intakeMotor.setPower(-1);
        robot.loaderServo.runContinuous(false, false);
    }

    @Override
    public void loop() {
        robot.follower.update();

        motorVel = robot.leftOuttake.getVelocity();

        robot.outtake.update();

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
                robot.outtake.activeConfig.velocity = RANGE_2_VELOCITY;
                robot.outtake.activeConfig.kS = RANGE_2_KS;
                robot.outtake.on = true;
                //go to pose
                robot.follower.followPath(goToLaunch1);
                setPathState(1);
                break;

            case 1:
                //go to pose
                if (!robot.follower.isBusy()) {
                    robot.follower.followPath(goToLaunch11);
                    launchState = 0;
                    setPathState(2);
                }
                break;

            case 2:
                if (!robot.follower.isBusy()) {
                    switch (launchState) {
                        case 0:
                            launchState++;
                            pathTimer.resetTimer();
                            break;

                        case 1:
                            // Wait for motors to reach speed and launch first 2
                            if (motorVel + 50 >= launchSpeedClose && pathTimer.getElapsedTimeSeconds() > 1.0) {
                                robot.loaderMotor.setPower(0.8);
                                launchState++;
                                pathTimer.resetTimer();
                            }
                            break;

                        case 2:
                            // timer to see when all 3 are launched
                            if (pathTimer.getElapsedTimeSeconds() > 2.0) {
                                robot.intakeMotor.setPower(0);
                                //robot.loaderMotor.setPower(0);
                                launchState++;
                                pathTimer.resetTimer();
                            }
                            break;
                        case 3:
                            setPathState(3);
                            break;

                    }
                    break;

                }
            case 3:
                if (!robot.follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1) {
                    robot.flapsServo.setPosition(FLAP_CLOSED);
                    robot.follower.followPath(goToIntake1);
                    robot.intakeMotor.setPower(-1);
                    robot.loaderMotor.setPower(1);

                    pathTimer.resetTimer();
                    setPathState(4);
                }
                break;

            case 4:
                if (!robot.follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1) {
                    robot.follower.followPath(goToIntake11);
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;

            case 5:
                if (!robot.follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1) {
                    robot.follower.followPath(goToLaunch2);

                    robot.loaderMotor.setPower(0);
                    robot.loaderMotor.setPower(-0.3);
                    launchState = 0;
                    pathTimer.resetTimer();
                    setPathState(6);
                }
                break;

            case 6:
                if (!robot.follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.6) {
                    switch (launchState) {
                        case 0:
                            launchState++;
                            pathTimer.resetTimer();
                            robot.flapsServo.setPosition(FLAP_OPEN);
                            robot.turretServo.setPosition(0.5);
                            break;

                        case 1:
                            // Wait for motors to reach speed and launch first 2
                            if (motorVel + 50 >= launchSpeedClose && pathTimer.getElapsedTimeSeconds() > 2.0) {
                                robot.loaderMotor.setPower(0.7);
                                launchState++;
                                pathTimer.resetTimer();
                            }
                            break;

                        case 2:
                            // timer to see when all 3 are launched
                            if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                                robot.intakeMotor.setPower(0);
                                //robot.loaderMotor.setPower(0);
                                launchState++;
                                pathTimer.resetTimer();
                            }
                            break;
                        case 3:
                            setPathState(7);
                            break;

                    }
                    break;

                }

            case 7:
                if (!robot.follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1) {
                    robot.flapsServo.setPosition(FLAP_CLOSED);
                    robot.follower.followPath(goToIntake2);
                    robot.intakeMotor.setPower(-1);
                    robot.loaderMotor.setPower(1);

                    pathTimer.resetTimer();
                    setPathState(8);
                }
                break;

            case 8:
                if (!robot.follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1) {
                    robot.follower.followPath(goToIntake22);
                    pathTimer.resetTimer();
                    setPathState(9);
                }
                break;

            case 9:
                if (!robot.follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1) {
                    robot.follower.followPath(goToLaunch3);
                    robot.loaderMotor.setPower(0);
                    robot.loaderMotor.setPower(-0.3);
                    launchState = 0;
                    pathTimer.resetTimer();
                    setPathState(10);
                }
                break;

            case 10:
                if (!robot.follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.6) {
                    switch (launchState) {
                        case 0:
                            launchState++;
                            pathTimer.resetTimer();
                            robot.flapsServo.setPosition(FLAP_OPEN);
                            robot.turretServo.setPosition(0.47);
                            break;

                        case 1:
                            // Wait for motors to reach speed and launch first 2
                            if (motorVel + 50 >= launchSpeedClose && pathTimer.getElapsedTimeSeconds() > 2.0) {
                                robot.loaderMotor.setPower(0.7);
                                launchState++;
                                pathTimer.resetTimer();
                            }
                            break;

                        case 2:
                            // timer to see when all 3 are launched
                            if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                                robot.intakeMotor.setPower(0);
                                //robot.loaderMotor.setPower(0);
                                launchState++;
                                pathTimer.resetTimer();
                            }
                            break;
                        case 3:
                            setPathState(11);
                            break;

                    }
                    break;

                }

            case 11:
                if (!robot.follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1) {
                    robot.follower.followPath(goToPark);
                    robot.loaderMotor.setPower(0);
                    pathTimer.resetTimer();
                    setPathState(-1);
                }
                break;

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