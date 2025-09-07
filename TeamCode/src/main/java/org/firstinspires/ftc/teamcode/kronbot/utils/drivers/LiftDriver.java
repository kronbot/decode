package org.firstinspires.ftc.teamcode.kronbot.utils.drivers;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LIFT_INIT_POSITION;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LIFT_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LIFT_REST_POWER;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LIFT_REVERSE_POWER;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LIFT_TOLERANCE;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Motor;


/**
 * This class is used to control the lift of the robot.
 * It uses a PID controller to control the lift.
 * It also has a manual mode to control the lift with the gamepad.
 *
 * @version 1.0
 */
public class LiftDriver {
    double kP = 0.005, kI = 0, kD = 0.0001;
    Motor liftMotor;

    public void init(Motor liftMotor, boolean pid, String name, boolean reversed) {
        this.liftMotor = liftMotor;
        this.liftMotor.init(name, reversed, pid, true, true, true, true);
        if (pid) {
            liftMotor.holdMode(true);
            liftMotor.setTolerance(LIFT_TOLERANCE);
            liftMotor.setPositionPID(kP, kI, kD);
        }
        this.liftMotor.setTargetPosition(0);
    }

    public void run(double power) {
        if (power < 0)
            power = power * LIFT_REVERSE_POWER;

        if (liftMotor.getCurrentPosition() >= LIFT_MAX_POSITION && power > 0) {
            liftMotor.setPower(LIFT_REST_POWER);
            return;
        } else if (liftMotor.getCurrentPosition() <= LIFT_INIT_POSITION && power < 0) {
            liftMotor.setPower(LIFT_REST_POWER);
            return;
        }

        if (power == 0) setPower(Constants.LIFT_REST_POWER);
        else setPower(power);
    }

    public int getTargetPosition() { return liftMotor.getTargetPosition(); }

    public boolean isBusy() {
        return liftMotor.isBusy();
    }

    public int getCurrentPosition() {
        return liftMotor.getCurrentPosition();
    }

    public void setTargetPosition(int position) {
        liftMotor.setTargetPosition(position);
    }

    public void runToPosition() {
//        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runUsingEncoders() {
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPower(double power) {
        liftMotor.setPower(power);
    }

    public void showInfo(Telemetry telemetry) {
        telemetry.addData("Motor Mode: ", liftMotor.motor.getMode());

        telemetry.addData("Lift TargetPos: ", liftMotor.getTargetPosition());
        telemetry.addData("Lift Current Position: ", liftMotor.getCurrentPosition());

        telemetry.addData("Motor Power: ", liftMotor.motor.getPower());
        telemetry.addData("Motor Direction: ", liftMotor.motor.getDirection());
        telemetry.addData("Lift Encoder: ", liftMotor.motor.getCurrentPosition());
        telemetry.addData("Motor Vel: ", liftMotor.motor.getVelocity());
        telemetry.addData("PID Pow: ", liftMotor.getPower());

        telemetry.addData("Motor Direction: ", liftMotor.direction);
    }
}