package org.firstinspires.ftc.teamcode.common.command.auto.spline;

/** Odometry-derived velocity, with the original per-tick exponential filter. */
public final class VelocityEstimator {
    private double prevX;
    private double prevY;
    private double vx;
    private double vy;

    public void rebase(double x, double y) {
        prevX = x;
        prevY = y;
        vx = 0;
        vy = 0;
    }

    public void update(double x, double y, double dt, double alpha) {
        if (dt <= 1e-5 || dt > 0.25) dt = 0.02;
        double rawVx = (x - prevX) / dt;
        double rawVy = (y - prevY) / dt;
        prevX = x;
        prevY = y;
        alpha = Math.max(0, Math.min(1, alpha));
        vx = alpha * rawVx + (1 - alpha) * vx;
        vy = alpha * rawVy + (1 - alpha) * vy;
    }

    public double vx() { return vx; }
    public double vy() { return vy; }
}
