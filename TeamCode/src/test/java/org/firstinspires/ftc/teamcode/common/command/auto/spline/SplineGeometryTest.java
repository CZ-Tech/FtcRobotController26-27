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
        // The derivative at u=0 and u=1 must be the supplied Hermite parameters.
        assertEquals(5, c[2], 1e-12);
        assertEquals(-3, 3 * c[0] + 2 * c[1] + c[2], 1e-12);
    }

    @Test
    public void straightLineArcLengthAndProjectionAreConsistent() {
        SplineGeometry g = new SplineGeometry(0, 0, 20, 0,
                20, 0, 20, 0, 240);
        assertEquals(20, g.totalLength(), 1e-9);
        assertEquals(5, g.sAtU(0.25), 1e-9);
        assertEquals(0.75, g.uAtS(15), 1e-9);
        assertEquals(0.5, g.closestU(10, 4, 0.1, 3, 18, 14, 4), 1e-7);
        assertEquals(1, g.unitTangent(0.5, 0.01)[0], 1e-9);
        assertEquals(0, g.unitTangent(0.5, 0.01)[1], 1e-9);
    }

    @Test
    public void curvedProjectionReturnsNearbyPoint() {
        SplineGeometry g = new SplineGeometry(0, 0, 12, 10,
                20, 0, 12, -10, 240);
        double u = g.closestU(g.xAt(0.6), g.yAt(0.6),
                0.5, 10, 15, 14, 4);
        assertEquals(0.6, u, 1e-5);
        assertTrue(g.totalLength() > 20);
    }

    @Test
    public void degenerateSegmentAndZeroEndpointDerivativeDoNotProduceNan() {
        SplineGeometry zero = new SplineGeometry(2, 4, 0, 0,
                2, 4, 0, 0, 40);
        assertEquals(0, zero.totalLength(), 0);
        assertEquals(0, zero.uAtS(1), 0);
        assertEquals(1, zero.unitTangent(0, 0.01)[0], 0);
        SplineGeometry curved = new SplineGeometry(0, 0, 0, 0,
                10, 10, 0, 0, 80);
        double[] tangent = curved.unitTangent(0, 0.01);
        assertEquals(Math.sqrt(0.5), tangent[0], 1e-5);
        assertEquals(Math.sqrt(0.5), tangent[1], 1e-5);
    }

    @Test
    public void localSearchWindowPreventsLargeForwardJump() {
        SplineGeometry g = new SplineGeometry(0, 0, 100, 0,
                100, 0, 100, 0, 100);
        double u = g.closestU(95, 0, 0, 3, 18, 14, 4);
        assertTrue(u <= 0.181);
    }
}
