package org.firstinspires.ftc.mockrobot;

import org.firstinspires.ftc.teamcode.common.network.CommandCatalog;
import org.firstinspires.ftc.teamcode.common.network.ControlGate;
import org.firstinspires.ftc.teamcode.common.network.ControlMailbox;
import org.firstinspires.ftc.teamcode.common.network.ExecutionStateStore;
import org.firstinspires.ftc.teamcode.common.network.RobotApiRouter;
import org.firstinspires.ftc.teamcode.common.network.RobotRuntimeStore;
import org.firstinspires.ftc.teamcode.common.network.SessionLease;
import org.firstinspires.ftc.teamcode.common.network.http.HttpHandler;
import org.firstinspires.ftc.teamcode.common.network.http.RobotHttpServer;
import org.firstinspires.ftc.teamcode.common.opmode.OpModeLifecycleService;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

/** Composition root for the standalone desktop virtual Robot Controller. */
public final class MockRobotRuntime implements AutoCloseable {
    public static final int PORT = 8888;

    public final MockSettings settings = new MockSettings();
    public final SessionLease session = new SessionLease();
    public final MockRouteStore routes;
    public final ControlMailbox mailbox = new ControlMailbox();
    public final ControlGate control = new ControlGate(mailbox);
    public final ExecutionStateStore execution = new ExecutionStateStore();
    public final RobotRuntimeStore runtime = new RobotRuntimeStore();
    public final CommandCatalog commands = new CommandCatalog();
    public final OpModeLifecycleService opModes = new OpModeLifecycleService();
    public final MockSimulationEngine simulation;
    public final MockOpModeBackend opModeBackend;

    private final RobotHttpServer server;

    public MockRobotRuntime() {
        Path routeFile = Path.of(
                System.getProperty("user.home"),
                ".azconductor",
                "mockrobot",
                "routes.json");
        routes = new MockRouteStore(routeFile);
        routes.seedIfEmpty();

        List<MockOpModeProfile> profiles = List.of(
                new MockOpModeProfile("Mock Idle Auto", "Mock", null, false),
                new MockOpModeProfile("Mock Straight Auto", "Mock", "Mock Straight", true),
                new MockOpModeProfile("Mock Spline Auto", "Mock", "Mock Spline", true),
                new MockOpModeProfile(
                        "Mock Figure Eight Auto",
                        "Mock",
                        "Mock Figure Eight",
                        true));

        simulation = new MockSimulationEngine(
                routes,
                settings,
                control,
                execution,
                runtime,
                opModes);
        opModeBackend = new MockOpModeBackend(
                opModes,
                simulation,
                control,
                execution,
                profiles);
        opModes.attach(opModeBackend);
        simulation.setAutoStopCallback(opModeBackend::externalStop);

        RobotApiRouter router = new RobotApiRouter(
                session,
                routes,
                control,
                execution,
                runtime,
                commands,
                opModes);
        HttpHandler loggingHandler = exchange -> {
            MockLog.debug(
                    "HTTP",
                    exchange.request.method + " " + exchange.request.path
                            + " <- " + exchange.remoteAddress());
            int latency = settings.httpLatencyMs();
            if (latency > 0 && !"/api/v2/events".equals(exchange.request.path)) {
                try {
                    Thread.sleep(latency);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
            router.handle(exchange);
        };
        server = new RobotHttpServer(PORT, loggingHandler);
    }

    public void start() throws IOException {
        server.start();
        simulation.start();
        runtime.publish(false, null, 0, 0, 0);
        MockLog.info("MockRobot", "Listening on http://127.0.0.1:" + PORT);
    }

    public boolean isRunning() {
        return server.isRunning();
    }

    public void disconnectClient() {
        session.revoke();
        MockLog.info("Session", "Current client lease revoked");
    }

    @Override
    public void close() {
        try {
            opModeBackend.externalStop();
        } catch (Exception ignored) {}
        simulation.close();
        server.stop();
        opModes.detach(opModeBackend);
    }
}
