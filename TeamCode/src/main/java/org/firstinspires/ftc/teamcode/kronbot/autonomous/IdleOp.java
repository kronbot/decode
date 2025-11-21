package org.firstinspires.ftc.teamcode.kronbot.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;

@Autonomous(name = "Idle", group = Constants.TEST_GROUP)
public class IdleOp extends LinearOpMode {

    KronBot robot = new KronBot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initAutonomy(hardwareMap);

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.update();
        }
    }
}