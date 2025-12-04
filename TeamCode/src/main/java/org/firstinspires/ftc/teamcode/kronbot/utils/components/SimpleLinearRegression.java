package org.firstinspires.ftc.teamcode.kronbot.utils.components;

public class SimpleLinearRegression {

    private double sumX = 0;
    private double sumY = 0;
    private double sumXY = 0;
    private double sumX2 = 0;
    private int n = 0;

    private double a = 0;  // slope
    private double b = 0;  // intercept
    private boolean trained = false;

    public void addData(double x, double y) {
        sumX += x;
        sumY += y;
        sumXY += x * y;
        sumX2 += x * x;
        n++;
        trained = false; // must recompute
    }

    private void compute() {
        if (trained || n < 2) return;

        a = (n * sumXY - sumX * sumY) / (n * sumX2 - (sumX * sumX));
        b = (sumY - a * sumX) / n;

        trained = true;
    }

    public double predict(double x) {
        compute();
        return a * x + b;
    }

    public double getSlope() {
        compute();
        return a;
    }

    public double getIntercept() {
        compute();
        return b;
    }
}
