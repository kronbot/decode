package org.firstinspires.ftc.teamcode.kronbot.baseOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Function;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kronbot.Robot;
import org.firstinspires.ftc.teamcode.kronbot.utils.misc.LpsCounter;

public abstract class BaseAuto extends OpMode {
    Robot robot;

    FtcDashboard dashboard;

    LpsCounter lpsCounter;


    @Override
    public void init() {
        lpsCounter.getLoopTime();

        robot = Robot.getInstance();
        robot.initSystems(hardwareMap);
        robot.initFollower(hardwareMap);

        try {
            robot.follower.getPoseTracker().getLocalizer().resetIMU();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        autoTelemetry();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {

    }

    void autoTelemetry() {
        telemetry.addData("LPS", "%.1f", 1 / lpsCounter.delta);

        robot.intake.telemetry(telemetry);
        robot.loader.telemetry(telemetry);
        robot.outtake.telemetry(telemetry);
        robot.turret.telemetry(telemetry);

        telemetry.update();
    }

    public void followPath(PathChain path) {

    }

    public void runIntake(double speed) {

    }

    public void runOuttake(double time, double speed, double angle) {

    }

    public void runOuttake(double time) {

    }
}
