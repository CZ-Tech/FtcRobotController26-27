package org.firstinspires.ftc.mockrobot;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.assertThrows;
import org.json.JSONException;

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
    public void versionTwoAndLegacyArraysHaveIdenticalGeometry() {
        String legacy = DemoRoutes.spline();
        MockRoutePlan old = MockRoutePlan.parse(legacy);
        MockRoutePlan versioned = MockRoutePlan.parse("{\"version\":2,\"frames\":" + legacy + "}");
        assertEquals(old.totalTime(), versioned.totalTime(), 0);
        for (int i = 0; i <= 10; i++) {
            double time = old.totalTime() * i / 10;
            assertEquals(old.sample(time).x, versioned.sample(time).x, 0);
            assertEquals(old.sample(time).y, versioned.sample(time).y, 0);
            assertEquals(old.sample(time).heading, versioned.sample(time).heading, 0);
        }
    }

    @Test
    public void unsupportedVersionIsNotTreatedAsLegacyGeometry() {
        assertThrows(JSONException.class,
                () -> MockRoutePlan.parse("{\"version\":3,\"frames\":" + DemoRoutes.spline() + "}"));
    }

    @Test
    public void versionTwoWaitFramesHoldThePreviousPoseInsteadOfAddingOriginPoints() {
        String frames = "[{\"x\":10,\"y\":20},{\"wait\":1},"
                + "{\"x\":20,\"y\":20,\"duration\":2},{\"wait\":3}]";
        MockRoutePlan plan = MockRoutePlan.parse("{\"version\":2,\"frames\":" + frames + "}");
        assertEquals(6, plan.totalTime(), 0);
        assertEquals(10, plan.sample(0.5).x, 0);
        assertEquals(15, plan.sample(2).x, 1e-9);
        assertEquals(20, plan.sample(5).x, 0);
        assertEquals(20, plan.sample(5).y, 0);
        assertEquals(20, plan.endPose().x, 0);
    }

    @Test
    public void waitFrameRequiresAPrecedingPoint() {
        assertThrows(IllegalArgumentException.class, () -> MockRoutePlan.parse("[{\"wait\":1}]"));
        assertThrows(IllegalArgumentException.class,
                () -> MockRoutePlan.parse("[{\"x\":1,\"y\":2},{\"wait\":-1}]"));
    }

    @Test
    public void polylineContainsEnoughSamplesForFieldRendering() {
        MockRoutePlan plan = MockRoutePlan.parse(DemoRoutes.figureEight());
        assertTrue(plan.polyline(12).size() > 20);
    }
}
