package org.firstinspires.ftc.teamcode.kronbot.autonomous;

import static org.firstinspires.ftc.teamcode.kronbot.autonomous.AutonomousConstants.*;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.FLAP_CLOSED;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.FLAP_OPEN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_2_KS;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_2_VELOCITY;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_4_KS;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.RANGE_4_VELOCITY;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

//import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.Robot;
import org.firstinspires.ftc.teamcode.kronbot.utils.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "RED Auto_Back", group = org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TEST_GROUP)
public class Auto_BackRedOp extends OpMode {

    private Robot robot = Robot.getInstance();
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private int launchState=0;

    // Define poses
    Pose start = coordinates(StartingPoseBackRed);
    Pose launchZoneBack = coordinates(LaunchZoneBack);
    Pose launch2 = coordinates(LaunchZoneBack);

    Pose intake1 = coordinates(IntakeZoneBack1);

    Pose intake11 = coordinates(IntakeZoneBack11);
    Pose parkBack = coordinates(ParkBack);
    private double motorVel;


    // Paths and PathChains
    private PathChain goToLaunch, goToLaunch2, goToPark, goToIntake1, goToIntake11;

    @Override
    public void init() {

        robot.initFollower(hardwareMap, start);
        robot.init(hardwareMap);

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

    /** Build all paths for the auto **/
    public void buildPaths() {

        goToLaunch = robot.follower.pathBuilder()
                .addPath(new BezierLine(start, launchZoneBack))
                .setLinearHeadingInterpolation(start.getHeading(), launchZoneBack.getHeading())
                .build();

        goToIntake1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(launchZoneBack, intake1))
                .setLinearHeadingInterpolation(launchZoneBack.getHeading(), intake1.getHeading())
                .build();

        goToIntake11 = robot.follower.pathBuilder()
                .addPath(new BezierLine(intake1, intake11))
                .setLinearHeadingInterpolation(intake1.getHeading(), intake11.getHeading())
                .build();


        goToLaunch2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(intake11, launch2))
                .setLinearHeadingInterpolation(intake11.getHeading(), launch2.getHeading())
                .build();

        goToPark = robot.follower.pathBuilder()
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

        robot.turretServo.setPosition(0);
        robot.angleServo.setPosition(angleServoBack);
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
        telemetry.addData("Launchstate: ", launchState);

        telemetry.update();
    }

    public void autonomousPathUpdate() {

        switch (pathState) {
            case 0:
                //also start motors to save time
                robot.outtake.on = true;
                robot.shoot.activateRange(4);
                robot.outtake.update();
//                robot.leftOuttake.setVelocity(launchSpeedBack);
//                robot.rightOuttake.setVelocity(launchSpeedBack);
//                robot.angleServo.setPosition(angleServoBack);
//
                //go to pose
                robot.follower.followPath(goToLaunch);
                setPathState(1);
                break;

            case 1:
                if (!robot.follower.isBusy()) {
                    switch (launchState) {
                        case 0:
                            launchState++;
                            pathTimer.resetTimer();
                            break;

                        case 1:
                            // Wait for motors to reach speed and launch first 2
                            if (motorVel + 50 >= launchSpeedBack && pathTimer.getElapsedTimeSeconds() > 1.5) {
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
                            setPathState(2);
                            break;

                    }
                    break;

                }


            case 2:
                if (!robot.follower.isBusy() && robot.follower.getHeadingError()<0.1 && pathTimer.getElapsedTimeSeconds()>1) {
                    robot.flapsServo.setPosition(FLAP_CLOSED);
                    robot.follower.followPath(goToIntake1);
                    robot.intakeMotor.setPower(-1);
                    robot.loaderMotor.setPower(1);

                    pathTimer.resetTimer();
                    setPathState(3);
                }
                break;

            case 3:
                if (!robot.follower.isBusy() && robot.follower.getHeadingError()<0.1 && pathTimer.getElapsedTimeSeconds()>1) {
                    robot.follower.followPath(goToIntake11);
                    pathTimer.resetTimer();
                    setPathState(4);
                }
                break;


            case 4:
                if (!robot.follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1) {
                    robot.follower.followPath(goToLaunch2);

                    robot.loaderMotor.setPower(0);
                    robot.loaderMotor.setPower(-0.3);
                    launchState = 0;
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;

            case 5:
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
                            setPathState(6);
                            break;

                    }
                    break;

                }

            case 6:
                //go to pose
                if (!robot.follower.isBusy()) {
                    robot.follower.followPath(goToPark);
                    setPathState(7);
                    robot.outtake.on = false;
                    robot.outtake.activeConfig = new Robot.RangeConfig(0, 0, 0);
                }
                break;

            case 7:
                if (!robot.follower.isBusy()) {
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
        Pose finalPose = robot.follower.getPose();
        PoseStorage.savePose(finalPose);
    }

}