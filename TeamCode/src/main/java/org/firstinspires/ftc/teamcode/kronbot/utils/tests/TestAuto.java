package org.firstinspires.ftc.teamcode.kronbot.utils.tests;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.LaunchZone;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.StartingPose;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.coordinates;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;


@Autonomous(name = "Test Autonomy", group = Constants.TEST_GROUP)
public class TestAuto extends LinearOpMode {

    KronBot robot = new KronBot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initAutonomy(hardwareMap);
        follower = createFollower(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Pose startingPose = coordinates(StartingPose);
        Pose launchZone = coordinates(LaunchZone);

        follower.setStartingPose(startingPose);
        //should move forward
        PathChain pathChain1 = follower.pathBuilder()
                .addPath(new BezierLine(startingPose, launchZone))
                .setLinearHeadingInterpolation(startingPose.getHeading(), launchZone.getHeading())
                .setBrakingStrength(1.3) //more precise but br br
                .addPath(new BezierLine(launchZone, startingPose))
                .setLinearHeadingInterpolation(launchZone.getHeading(), startingPose.getHeading())
                .setBrakingStrength(0.7) //may overshoot but smoother
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

            if(!follower.isBusy()) {
                telemetry.addData("current coordx is: ", coordx);
                telemetry.addData("current coordy is: ", coordy);
                telemetry.addData("Heading", currentPose.getHeading());

                telemetry.addLine("Path complete — running outtake");
                robot.leftOuttake.setPower(1);
                robot.rightOuttake.setPower(1);
                sleep(4000);
                robot.leftOuttake.setPower(0);
                robot.rightOuttake.setPower(0);
                break;
            }

            telemetry.update();
        }
    }
}