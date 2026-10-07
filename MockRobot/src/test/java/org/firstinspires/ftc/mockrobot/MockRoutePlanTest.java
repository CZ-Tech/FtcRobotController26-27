package org.firstinspires.ftc.mockrobot;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class MockRoutePlanTest {
    @Test
    public void demoSplineParsesAndInterpolatesEndpoints() {
        MockRoutePlan plan = MockRoutePlan.parse(DemoRoutes.spline());

        assertTrue(plan.totalTime() > 0);
        MockRoutePlan.Pose start = plan.startPose();
        MockRoutePlan.Pose end = plan.endPose();
        MockRoutePlan.Pose sampledStart = plan.sample(0);
        MockRoutePlan.Pose sampledEnd = plan.sample(plan.totalTime());

        assertEquals(start.x, sampledStart.x, 1e-9);
        assertEquals(start.y, sampledStart.y, 1e-9);
        assertEquals(end.x, sampledEnd.x, 1e-9);
        assertEquals(end.y, sampledEnd.y, 1e-9);
    }

    @Test
    public void polylineContainsEnoughSamplesForFieldRendering() {
        MockRoutePlan plan = MockRoutePlan.parse(DemoRoutes.figureEight());
        assertTrue(plan.polyline(12).size() > 20);
    }
}
