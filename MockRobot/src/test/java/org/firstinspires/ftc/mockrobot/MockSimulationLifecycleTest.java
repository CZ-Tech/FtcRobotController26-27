package org.firstinspires.ftc.mockrobot;

import org.firstinspires.ftc.teamcode.common.network.ExecutionStateStore;
import org.firstinspires.ftc.teamcode.common.network.OpModeLifecycleService;
import org.firstinspires.ftc.teamcode.common.network.RobotRuntimeStore;
import org.junit.Test;

import java.nio.file.Files;
import java.nio.file.Path;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

public class MockSimulationLifecycleTest {
    @Test
    public void boundRouteStartsWhenOpModeStarts() throws Exception {
        Path dir = Files.createTempDirectory("mockrobot-lifecycle-test");
        MockRouteStore routes = new MockRouteStore(dir.resolve("routes.json"));
        routes.put("Test Route", DemoRoutes.straight(), 0L);

        ExecutionStateStore execution = new ExecutionStateStore();
        RobotRuntimeStore runtime = new RobotRuntimeStore();
        OpModeLifecycleService opModes = new OpModeLifecycleService();
        MockSimulationEngine simulation = new MockSimulationEngine(
                routes,
                new MockSettings(),
                execution,
                runtime,
                opModes);
        MockOpModeBackend backend = new MockOpModeBackend(
                opModes,
                simulation,
                execution,
                routes);
        opModes.attach(backend);

        try {
            simulation.start();
            assertTrue(backend.externalInit("Test Route"));
            assertTrue(backend.externalStart());

            Thread.sleep(80);

            MockSimulationEngine.Snapshot snapshot = simulation.snapshot();
            assertTrue(snapshot.routeRunning);
            assertEquals("Test Route", snapshot.routeName);
            assertNotEquals(0.0, snapshot.x, 0.001);
        } finally {
            simulation.close();
        }
    }
}
