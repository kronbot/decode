package org.firstinspires.ftc.teamcode.kronbot.utils.pid;

/**
 * Clock interface with nanosecond precision and no guarantee about its origin (that is, this is only suited for
 * measuring relative/elapsed time).
 */
public abstract class NanoClock {

    /**
     * Returns a NanoClock backed by System.nanoTime.
     */
    public static NanoClock system() {
        return new NanoClock() {
            @Override
            public double seconds() {
                return System.nanoTime() / 1e9;
            }

            @Override
            public double millis() {
                return System.currentTimeMillis();
            }
        };
    }

    /**
     * Returns the number of seconds since an arbitrary (yet consistent) origin.
     */
    public abstract double seconds();

    /**
     * Returns the number of milliseconds since an arbitrary (yet consistent) origin.
     */
    public abstract  double millis();
}