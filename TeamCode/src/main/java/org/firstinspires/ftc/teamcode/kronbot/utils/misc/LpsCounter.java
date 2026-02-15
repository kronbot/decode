package org.firstinspires.ftc.teamcode.kronbot.utils.misc;


import com.qualcomm.robotcore.util.ElapsedTime;

public class LpsCounter {
    ElapsedTime timer;
    double lastTime;

    /** The time between the last two calls */
    public double delta;

    /**
     * Get the time between the last call of the function and this call.
     * Call once at initialization and once per loop.<br>
     * This function updates delta.
     * @return The time between calls, in seconds.
     */
    public double getLoopTime() {
        if(timer == null) {
            timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            lastTime = 0;
            return 1;
        }

        delta = timer.time() - lastTime;
        lastTime = timer.time();

        return delta;
    }

}
