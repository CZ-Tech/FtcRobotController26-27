package org.firstinspires.ftc.teamcode.common.command.auto.spline;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class SplineGeometryTest {
    @Test
    public void hermiteMatchesEndpointsAndEndpointTangents() {
        double[] c = SplineGeometry.hermite(2, 5, 20, -3);
        assertEquals(2, SplineGeometry.eval(c, 0), 1e-12);
        assertEquals(20, SplineGeometry.eval(c, 1), 1e-12);
        assertEquals(5, c[2], 1e-12);
        assertEquals(-3, 3 * c[0] + 2 * c[1] + c[2], 1e-12);
    }

    @Test
    public void straightLineArcLengthAndContinuousChordProjectionAreConsistent() {
        SplineGeometry g = new SplineGeometry(0, 0, 20, 0,
                20, 0, 20, 0, 1000);
        assertEquals(1000, g.projectionSegmentCount());
        assertEquals(20, g.totalLength(), 1e-9);
        assertEquals(5, g.sAtU(0.25), 1e-9);
        assertEquals(0.75, g.uAtS(15), 1e-9);
        // Deliberately between samples: projection is continuous, not nearest vertex.
        SplineGeometry.Projection p = g.project(10.013, 4, 0.1);
        assertEquals(10.013 / 20, p.u, 1e-9);
        assertEquals(10.013, p.x, 1e-9);
        assertEquals(0, p.y, 1e-9);
        assertEquals(10.013, p.s, 1e-9);
        assertEquals(16, p.distanceSq, 1e-9);
        assertEquals(1, p.tx, 1e-9);
        assertEquals(0, p.ty, 1e-9);
    }

    @Test
    public void curvedProjectionReturnsNearbyPoint() {
        SplineGeometry g = new SplineGeometry(0, 0, 12, 10,
                20, 0, 12, -10, 1000);
        SplineGeometry.Projection p = g.project(g.xAt(0.6), g.yAt(0.6), 0);
        assertEquals(0.6, p.u, 1e-5);
        assertTrue(p.distanceSq < 1e-8);
        assertTrue(g.totalLength() > 20);
    }

    @Test
    public void degenerateSegmentAndZeroEndpointDerivativeDoNotProduceNan() {
        SplineGeometry zero = new SplineGeometry(2, 4, 0, 0,
                2, 4, 0, 0, 1000);
        assertEquals(0, zero.totalLength(), 0);
        SplineGeometry.Projection p = zero.project(5, 8, 0);
        assertEquals(2, p.x, 0);
        assertEquals(4, p.y, 0);
        assertEquals(25, p.distanceSq, 0);
        assertEquals(1, p.tx, 0);
        assertEquals(0, p.ty, 0);
        SplineGeometry curved = new SplineGeometry(0, 0, 0, 0,
                10, 10, 0, 0, 1000);
        p = curved.project(5, 5, 0);
        assertEquals(5, p.x, 1e-9);
        assertEquals(Math.sqrt(0.5), p.tx, 1e-9);
        assertEquals(Math.sqrt(0.5), p.ty, 1e-9);
    }

    @Test
    public void globalProjectionMatchesDenseCurveReferenceFarOffPath() {
        SplineGeometry g = new SplineGeometry(0, 0, 80, 120,
                100, 0, 80, -120, 1000);
        for (int x = -40; x <= 140; x += 20) {
            for (int y = -100; y <= 100; y += 20) {
                SplineGeometry.Projection p = g.project(x, y, 0);
                double denseDistance = Double.POSITIVE_INFINITY;
                for (int i = 0; i <= 20000; i++) {
                    double u = i / 20000.0;
                    denseDistance = Math.min(denseDistance,
                            Math.hypot(g.xAt(u) - x, g.yAt(u) - y));
                }
                assertEquals(denseDistance, Math.sqrt(p.distanceSq), 0.002);
                assertEquals(Math.hypot(p.x - x, p.y - y), Math.sqrt(p.distanceSq), 1e-9);
            }
        }
    }

    @Test
    public void searchIsGlobalRegardlessOfPreviousProjection() {
        SplineGeometry g = new SplineGeometry(0, 0, 100, 0,
                100, 0, 100, 0, 1000);
        assertEquals(0.95, g.project(95, 0, 0).u, 1e-9);
        assertEquals(0.05, g.project(5, 0, 1).u, 1e-9);
    }

    @Test
    public void equalArcSamplingHandlesHighlyNonuniformParameterSpeed() {
        SplineGeometry g = new SplineGeometry(0, 0, 0, 0,
                100, 0, 300, 0, 1000); // x(u) = 100u^3
        for (double x : new double[]{0.03, 0.15, 10.013, 95.127}) {
            SplineGeometry.Projection p = g.project(x, 3, 0);
            assertEquals(x, p.x, 1e-9);
            assertEquals(x, p.s, 0.0001);
            assertEquals(Math.cbrt(x / 100), p.u, 0.002);
        }
    }

    @Test
    public void equalDistanceRetracedBranchesUsePreviousUOnlyAsTieBreaker() {
        // x(u)=4u(1-u), y=0: both halves trace the same line.
        SplineGeometry g = new SplineGeometry(0, 0, 4, 0,
                0, 0, -4, 0, 1000);
        assertTrue(g.project(0.5, 0, 0).u < 0.5);
        assertTrue(g.project(0.5, 0, 1).u > 0.5);
    }

    @Test(expected = IllegalArgumentException.class)
    public void nonFiniteGeometryIsRejected() {
        new SplineGeometry(0, 0, Double.NaN, 0, 20, 0, 20, 0, 1000);
    }

    @Test(expected = IllegalArgumentException.class)
    public void nonFiniteProjectionInputIsRejected() {
        new SplineGeometry(0, 0, 20, 0, 20, 0, 20, 0, 1000)
                .project(Double.NaN, 0, 0);
    }
}
