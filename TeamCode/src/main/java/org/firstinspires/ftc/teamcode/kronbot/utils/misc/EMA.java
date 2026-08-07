package org.firstinspires.ftc.teamcode.kronbot.utils.misc;

// vibe coded Exponential Moving Average
public class EMA {
    private final double alpha;
    private double average;
    private boolean initialized;

    /**
     * Creates an EMA using an explicit smoothing factor.
     *
     * @param alpha smoothing factor in the range (0, 1]. Higher values
     *              weight recent observations more heavily.
     */
    public EMA(double alpha) {
        if (alpha <= 0.0 || alpha > 1.0) {
            throw new IllegalArgumentException("alpha must be in the range (0, 1]");
        }
        this.alpha = alpha;
    }

    /**
     * Creates an EMA using a period (window size), converting it to a
     * smoothing factor via alpha = 2 / (period + 1). This is the common
     * convention used in technical analysis (e.g. a "20-day EMA").
     *
     * @param period number of observations the average roughly represents;
     *               must be >= 1.
     * @return a new ExponentialMovingAverage configured with the derived alpha
     */
    public static EMA ofPeriod(int period) {
        if (period < 1) {
            throw new IllegalArgumentException("period must be >= 1");
        }
        double alpha = 2.0 / (period + 1);
        return new EMA(alpha);
    }

    /**
     * Feeds a new value into the average and returns the updated EMA.
     *
     * @param value the next observation in the series
     * @return the updated exponential moving average
     */
    public double add(double value) {
        if (!initialized) {
            average = value;
            initialized = true;
        } else {
            average = (value * alpha) + (average * (1 - alpha));
        }
        return average;
    }

    /**
     * Returns the current EMA value.
     *
     * @return the current average
     * @throws IllegalStateException if no values have been added yet
     */
    public double getAverage() {
        if (!initialized) {
            throw new IllegalStateException("No values have been added yet");
        }
        return average;
    }

    /**
     * @return true if at least one value has been added
     */
    public boolean hasValue() {
        return initialized;
    }

    /**
     * Resets the average so the next added value becomes the new starting point.
     */
    public void reset() {
        initialized = false;
        average = 0.0;
    }

    public double getAlpha() {
        return alpha;
    }


}
