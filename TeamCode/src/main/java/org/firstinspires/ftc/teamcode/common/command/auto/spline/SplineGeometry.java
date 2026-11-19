package org.firstinspires.ftc.teamcode.common.command.auto.spline;

/**
 * Immutable cubic Hermite segment with an equal-arc-length projection polyline.
 * Coordinates and tangent parameters are in inches; u is dimensionless.
 * No Android or robot hardware dependency.
 */
public final class SplineGeometry {
    private final double[] sx, sy;
    private final double[] cumulative;
    private final int n;
    private final double total;
    private final double[] sampleX, sampleY, sampleU;
    private final int segments;

    /** Nearest point on the entire sampled polyline, not just a local window. */
    public static final class Projection {
        public final double u, x, y, s, tx, ty, distanceSq;

        private Projection(double u, double x, double y, double s,
                           double tx, double ty, double distanceSq) {
            this.u = u;
            this.x = x;
            this.y = y;
            this.s = s;
            this.tx = tx;
            this.ty = ty;
            this.distanceSq = distanceSq;
        }
    }

    public SplineGeometry(double x0, double y0, double dx0, double dy0,
                          double x1, double y1, double dx1, double dy1,
                          int projectionSegments) {
        if (!Double.isFinite(x0) || !Double.isFinite(y0)
                || !Double.isFinite(dx0) || !Double.isFinite(dy0)
                || !Double.isFinite(x1) || !Double.isFinite(y1)
                || !Double.isFinite(dx1) || !Double.isFinite(dy1)) {
            throw new IllegalArgumentException("Non-finite spline geometry");
        }
        sx = hermite(x0, dx0, x1, dx1);
        sy = hermite(y0, dy0, y1, dy1);
        // Bound configuration mistakes as well as the work done in each control tick.
        segments = Math.max(40, Math.min(10000, projectionSegments));
        n = Math.max(1000, segments * 4);
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
        if (!Double.isFinite(total)) {
            throw new IllegalArgumentException("Spline geometry overflow");
        }
        sampleX = new double[segments + 1];
        sampleY = new double[segments + 1];
        sampleU = new double[segments + 1];
        for (int i = 0; i <= segments; i++) {
            double u = i == segments ? 1 : uAtS(total * i / segments);
            sampleU[i] = u;
            sampleX[i] = xAt(u);
            sampleY[i] = yAt(u);
        }
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

    public double xAt(double u) { return eval(sx, u); }
    public double yAt(double u) { return eval(sy, u); }
    public double totalLength() { return total; }
    public int projectionSegmentCount() { return segments; }

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

    /**
     * Fixed-cost global scan: project onto every chord and compare squared distances.
     * previousU only breaks equal-distance ties; it never limits the search range.
     * Exactly one result object is allocated, with no per-chord allocations or square roots.
     */
    public Projection project(double rx, double ry, double previousU) {
        if (!Double.isFinite(rx) || !Double.isFinite(ry) || !Double.isFinite(previousU)) {
            throw new IllegalArgumentException("Non-finite projection input");
        }
        double bestD2 = Double.POSITIVE_INFINITY;
        double bestT = 0;
        double bestU = 0;
        double bestX = sampleX[0], bestY = sampleY[0];
        int bestIndex = 0;
        for (int i = 0; i < segments; i++) {
            double dx = sampleX[i + 1] - sampleX[i];
            double dy = sampleY[i + 1] - sampleY[i];
            double lengthSq = dx * dx + dy * dy;
            double t = lengthSq > 0
                    ? clip01(((rx - sampleX[i]) * dx + (ry - sampleY[i]) * dy) / lengthSq)
                    : 0;
            double x = sampleX[i] + t * dx;
            double y = sampleY[i] + t * dy;
            double ex = x - rx, ey = y - ry;
            double d2 = ex * ex + ey * ey;
            double u = sampleU[i] + t * (sampleU[i + 1] - sampleU[i]);
            if (d2 < bestD2 - 1e-12 || (Math.abs(d2 - bestD2) <= 1e-12
                    && Math.abs(u - previousU) < Math.abs(bestU - previousU))) {
                bestD2 = d2;
                bestT = t;
                bestU = u;
                bestX = x;
                bestY = y;
                bestIndex = i;
            }
        }
        double tx = sampleX[bestIndex + 1] - sampleX[bestIndex];
        double ty = sampleY[bestIndex + 1] - sampleY[bestIndex];
        double length = Math.hypot(tx, ty);
        if (length > 0) {
            tx /= length;
            ty /= length;
        } else {
            double[] tangent = unitTangent(bestU, 0.01);
            tx = tangent[0];
            ty = tangent[1];
        }
        double s = total * (bestIndex + bestT) / segments;
        return new Projection(uAtS(s), bestX, bestY, s, tx, ty, bestD2);
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
}
