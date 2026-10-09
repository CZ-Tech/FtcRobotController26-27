package org.firstinspires.ftc.teamcode.common.command.auto.spline;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class VelocityEstimatorTest {
    @Test
    public void velocityUsesDisplacementAndExponentialFilter() {
        VelocityEstimator estimator = new VelocityEstimator();
        estimator.rebase(5, 3);
        estimator.update(7, 2, 0.1, 0.35); // raw: 20, -10
        assertEquals(7, estimator.vx(), 1e-12);
        assertEquals(-3.5, estimator.vy(), 1e-12);
        estimator.update(9, 1, 0.1, 0.35);
        assertEquals(0.35 * 20 + 0.65 * 7, estimator.vx(), 1e-12);
        assertEquals(0.35 * -10 + 0.65 * -3.5, estimator.vy(), 1e-12);
    }

    @Test
    public void rebaseClearsVelocityAndOutOfRangeDtUsesTwentyMilliseconds() {
        VelocityEstimator estimator = new VelocityEstimator();
        estimator.rebase(10, 10);
        estimator.update(11, 10, 0.5, 1);
        assertEquals(50, estimator.vx(), 1e-12);
        estimator.rebase(100, 100);
        assertEquals(0, estimator.vx(), 0);
        estimator.update(100.2, 100.4, 0.02, 1);
        assertEquals(10, estimator.vx(), 1e-10);
        assertEquals(20, estimator.vy(), 1e-10);
    }

    @Test
    public void filterAlphaIsClamped() {
        VelocityEstimator estimator = new VelocityEstimator();
        estimator.rebase(0, 0);
        estimator.update(1, 0, 0.1, -1);
        assertEquals(0, estimator.vx(), 0);
        estimator.update(2, 0, 0.1, 2);
        assertEquals(10, estimator.vx(), 1e-12);
    }
}
