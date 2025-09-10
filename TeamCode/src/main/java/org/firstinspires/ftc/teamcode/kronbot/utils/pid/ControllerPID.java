package org.firstinspires.ftc.teamcode.kronbot.utils.pid;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A basic PID controller
 *
 * @version 1.0
 */
public class ControllerPID {
    double kP, kI, kD;

    public ControllerPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        reset();
    }

    double lastError = 0;
    double integral = 0;

    public double calculate(double reference, double currentState) {
        double error = reference - currentState;

        double loopTime = getLoopTime();
        double derivative = (error - lastError) / loopTime;

        integral += ((error + lastError) / 2) * loopTime;

        lastError = error;

        return (kP * error) + (kI * integral) + (kD * derivative);
    }

    private void reset()
    {
        lastError = 0;
        integral = 0;
        timer.reset();
    }

    boolean started = false;
    ElapsedTime timer = new ElapsedTime();

    private double getLoopTime() {
        if (!started) {
            started = true;
            timer.reset();
        }
        double time = timer.seconds();
        timer.reset();
        return time;
    }
}
