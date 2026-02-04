package org.firstinspires.ftc.teamcode.kronbot.utils.tests;
import static org.firstinspires.ftc.teamcode.kronbot.autonomous.AutonomousConstants.LaunchZoneClose;
import static org.firstinspires.ftc.teamcode.kronbot.autonomous.AutonomousConstants.StartingPoseCloseRed;
import static org.firstinspires.ftc.teamcode.kronbot.autonomous.AutonomousConstants.coordinates;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;


@Autonomous(name = "Test Autonomy")
public class TestAuto extends LinearOpMode {

    KronBot robot = new KronBot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initAutonomy(hardwareMap);
        Follower follower = Constants.createFollower(hardwareMap);

        telemetry.addLine(follower == null ? "Follower is NULL!" : "Follower created!");
        telemetry.update();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Pose startingPose = coordinates(StartingPoseCloseRed);
        Pose launchZone = coordinates(LaunchZoneClose);

        follower.setStartingPose(startingPose);

        telemetry.addData("Pedro heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();

        PathChain pathChain1;

        pathChain1 = follower.pathBuilder()
                .addPath(new BezierLine(startingPose, launchZone))
                .setLinearHeadingInterpolation(startingPose.getHeading(), launchZone.getHeading())
                .setBrakingStrength(1.3)
                .addPath(new BezierLine(launchZone, startingPose))
                .setLinearHeadingInterpolation(launchZone.getHeading(), startingPose.getHeading())
                .setBrakingStrength(1.3)
                .build();

        waitForStart();

        if(opModeIsActive()) {
            follower.followPath(pathChain1);
        }

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();

            Pose currentPose = follower.getPose();
            double coordx = currentPose.getX();
            double coordy = currentPose.getY();
            currentPose.getHeading();
            telemetry.addData("current coordx is: ", coordx);
            telemetry.addData("current coordy is: ", coordy);
            telemetry.addData("Heading", currentPose.getHeading());

            while(!follower.isBusy()) {
                robot.loaderServo.runContinuous(false, true);
                robot.leftOuttake.setPower(1);
                robot.rightOuttake.setPower(1);
                sleep(4000);
                robot.leftOuttake.setPower(0);
                robot.rightOuttake.setPower(0);
                sleep(4000);

            }
            telemetry.update();
        }
    }
}