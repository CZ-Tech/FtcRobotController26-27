package org.firstinspires.ftc.teamcode.common.command.auto.spline;

/**
 * One immutable cubic Hermite segment with a sampled arc-length lookup table.
 * Coordinates and tangent parameters are in inches; u is dimensionless.
 * No Android or robot hardware dependency.
 */
public final class SplineGeometry {
    private final double[] sx;
    private final double[] sy;
    private final double[] cumulative;
    private final int n;
    private final double total;

    public SplineGeometry(double x0, double y0, double dx0, double dy0,
                          double x1, double y1, double dx1, double dy1,
                          int arcSamples) {
        sx = hermite(x0, dx0, x1, dx1);
        sy = hermite(y0, dy0, y1, dy1);
        n = Math.max(40, arcSamples);
        cumulative = new double[n + 1];
        double px = xAt(0);
        double py = yAt(0);
        for (int i = 1; i <= n; i++) {
            double u = (double) i / n;
            double cx = xAt(u);
            double cy = yAt(u);
            cumulative[i] = cumulative[i - 1] + Math.hypot(cx - px, cy - py);
            px = cx;
            py = cy;
        }
        total = cumulative[n];
    }

    public static double[] hermite(double x0, double dx0, double x1, double dx1) {
        double a = 2 * x0 + dx0 - 2 * x1 + dx1;
        double b = -3 * x0 - 2 * dx0 + 3 * x1 - dx1;
        return new double[]{a, b, dx0, x0};
    }

    public static double eval(double[] spline, double u) {
        return spline[0] * u * u * u + spline[1] * u * u + spline[2] * u + spline[3];
    }

    private static double derivative(double[] spline, double u) {
        return 3 * spline[0] * u * u + 2 * spline[1] * u + spline[2];
    }

    private static double secondDerivative(double[] spline, double u) {
        return 6 * spline[0] * u + 2 * spline[1];
    }

    public double xAt(double u) { return eval(sx, u); }
    public double yAt(double u) { return eval(sy, u); }
    public double totalLength() { return total; }

    public double sAtU(double u) {
        u = clip01(u);
        double p = u * n;
        int i = (int) Math.floor(p);
        if (i >= n) return total;
        double f = p - i;
        return cumulative[i] + f * (cumulative[i + 1] - cumulative[i]);
    }

    public double uAtS(double s) {
        if (total <= 1e-9) return 0;
        if (s <= 0) return 0;
        if (s >= total) return 1;
        int lo = 0;
        int hi = n;
        while (lo < hi - 1) {
            int mid = (lo + hi) >>> 1;
            if (cumulative[mid] <= s) lo = mid;
            else hi = mid;
        }
        double span = cumulative[hi] - cumulative[lo];
        double f = span > 1e-9 ? (s - cumulative[lo]) / span : 0;
        return (lo + f) / n;
    }

    /** Local coarse search followed by Newton projection; preserves previous window semantics. */
    public double closestU(double rx, double ry, double previousU,
                           double backtrackIn, double forwardIn,
                           int projectionSamples, int newtonIterations) {
        double previousS = sAtU(previousU);
        double lowU = uAtS(previousS - Math.max(0, backtrackIn));
        double highU = uAtS(previousS + Math.max(0, forwardIn));
        if (highU < lowU + 1e-6) {
            lowU = Math.max(0, previousU - 0.05);
            highU = Math.min(1, previousU + 0.10);
        }

        int samples = Math.max(4, projectionSamples);
        double bestU = lowU;
        double bestD2 = Double.POSITIVE_INFINITY;
        for (int i = 0; i <= samples; i++) {
            double u = lowU + (highU - lowU) * i / samples;
            double ex = xAt(u) - rx;
            double ey = yAt(u) - ry;
            double d2 = ex * ex + ey * ey;
            if (d2 < bestD2) {
                bestD2 = d2;
                bestU = u;
            }
        }

        double u = bestU;
        for (int i = 0; i < Math.max(1, newtonIterations); i++) {
            double px = xAt(u);
            double py = yAt(u);
            double dxdu = derivative(sx, u);
            double dydu = derivative(sy, u);
            double ddxdu = secondDerivative(sx, u);
            double ddydu = secondDerivative(sy, u);
            double ex = px - rx;
            double ey = py - ry;
            double f = ex * dxdu + ey * dydu;
            double df = dxdu * dxdu + dydu * dydu + ex * ddxdu + ey * ddydu;
            if (Math.abs(df) < 1e-9) break;
            u -= f / df;
            u = clip(u, lowU, highU);
        }

        double d0 = distanceSq(rx, ry, u);
        double dl = distanceSq(rx, ry, lowU);
        double dh = distanceSq(rx, ry, highU);
        if (dl < d0 && dl <= dh) return lowU;
        if (dh < d0) return highU;
        return u;
    }

    private double distanceSq(double rx, double ry, double u) {
        double ex = xAt(u) - rx;
        double ey = yAt(u) - ry;
        return ex * ex + ey * ey;
    }

    /** Unit tangent in increasing-u direction, including at zero-derivative endpoints. */
    public double[] unitTangent(double u, double tangentSampleDu) {
        double tx = derivative(sx, u);
        double ty = derivative(sy, u);
        double mag = Math.hypot(tx, ty);
        if (mag < 1e-6) {
            double du = Math.max(1e-4, tangentSampleDu);
            double u0 = Math.max(0, u - du);
            double u1 = Math.min(1, u + du);
            if (u1 - u0 < 1e-6) {
                u0 = Math.max(0, u - 2 * du);
                u1 = Math.min(1, u + 2 * du);
            }
            tx = xAt(u1) - xAt(u0);
            ty = yAt(u1) - yAt(u0);
            mag = Math.hypot(tx, ty);
        }
        if (mag < 1e-6) {
            tx = xAt(1) - xAt(0);
            ty = yAt(1) - yAt(0);
            mag = Math.hypot(tx, ty);
        }
        if (mag < 1e-6) return new double[]{1, 0};
        return new double[]{tx / mag, ty / mag};
    }

    private static double clip01(double x) { return Math.max(0, Math.min(1, x)); }
    private static double clip(double x, double low, double high) {
        return Math.max(low, Math.min(high, x));
    }
}
