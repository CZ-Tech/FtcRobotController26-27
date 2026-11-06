package org.firstinspires.ftc.mockrobot;

import org.firstinspires.ftc.teamcode.common.network.CommandCatalog;
import org.firstinspires.ftc.teamcode.common.network.CommandCatalog.Descriptor;
import org.firstinspires.ftc.teamcode.common.network.CommandDispatcher;
import org.firstinspires.ftc.teamcode.common.network.ExecutionStateStore;
import org.firstinspires.ftc.teamcode.common.network.RobotApiRouter;
import org.firstinspires.ftc.teamcode.common.network.RobotRuntimeStore;
import org.firstinspires.ftc.teamcode.common.network.SessionLease;
import org.firstinspires.ftc.teamcode.common.network.http.HttpHandler;
import org.firstinspires.ftc.teamcode.common.network.http.PemTls;
import org.firstinspires.ftc.teamcode.common.network.http.RobotHttpServer;
import org.firstinspires.ftc.teamcode.common.network.OpModeLifecycleService;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/** Composition root for the standalone desktop virtual Robot Controller. */
public final class MockRobotRuntime implements AutoCloseable {
    public static final int PORT = 8888;
    public static final int HTTPS_PORT = 8889;

    private boolean tlsEnabled;

    public final MockSettings settings = new MockSettings();
    public final SessionLease session = new SessionLease();
    public final MockRouteStore routes;
    public final ExecutionStateStore execution = new ExecutionStateStore();
    public final RobotRuntimeStore runtime = new RobotRuntimeStore();
    public final CommandCatalog commands = new CommandCatalog();
    public final OpModeLifecycleService opModes = new OpModeLifecycleService();
    public final CommandDispatcher commandDispatcher = new CommandDispatcher();
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

        simulation = new MockSimulationEngine(
                routes,
                settings,
                execution,
                runtime,
                opModes);
        opModeBackend = new MockOpModeBackend(
                opModes,
                simulation,
                execution,
                routes);
        opModes.attach(opModeBackend);
        simulation.setAutoStopCallback(opModeBackend::externalStop);

        commands.publish(sampleCommands());
        commandDispatcher.attach((name, args) -> {
            MockLog.info("Command", name + " " + Arrays.toString(args));
            return true;
        });

        RobotApiRouter router = new RobotApiRouter(
                session,
                routes,
                execution,
                runtime,
                commands,
                opModes,
                commandDispatcher);
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
        enableTlsIfCertificateAvailable();
    }

    /**
     * Mirrors the real robot: HTTPS is optional and never prevents HTTP from starting.
     * Certificate directory: -Dmockrobot.tls.dir (set by Gradle to .cert-cache) or
     * ~/.azconductor/mockrobot/tls, containing fullchain.pem and privkey.pem.
     */
    private void enableTlsIfCertificateAvailable() {
        String configured = System.getProperty("mockrobot.tls.dir");
        Path dir = configured != null && !configured.isEmpty()
                ? Path.of(configured)
                : Path.of(System.getProperty("user.home"), ".azconductor", "mockrobot", "tls");
        Path chain = dir.resolve("fullchain.pem");
        Path key = dir.resolve("privkey.pem");
        if (!Files.isRegularFile(chain) || !Files.isRegularFile(key)) {
            MockLog.info("MockRobot", "No TLS certificate in " + dir + "; HTTPS disabled");
            return;
        }
        try {
            PemTls.Material material = PemTls.load(
                    Files.readString(chain, StandardCharsets.UTF_8),
                    Files.readString(key, StandardCharsets.UTF_8));
            server.enableTls(HTTPS_PORT, material.context);
            tlsEnabled = true;
            MockLog.info("MockRobot", "HTTPS enabled on port " + HTTPS_PORT
                    + ", certificate valid until " + material.leaf.getNotAfter());
        } catch (Exception e) {
            MockLog.info("MockRobot", "TLS certificate unusable, HTTPS disabled: " + e);
        }
    }

    /** Catalog mirroring what the real robot publishes, including an overloaded command. */
    private static List<Descriptor> sampleCommands() {
        return Arrays.asList(
                new Descriptor(
                        "shoot",
                        Arrays.asList("count", "power"),
                        Arrays.asList("int", "double"),
                        Arrays.asList(null, "0.8"),
                        Arrays.asList(
                                Collections.singletonList("int"),
                                Arrays.asList("int", "double"))),
                new Descriptor(
                        "intaker.intakerIn",
                        Collections.singletonList("power"),
                        Collections.singletonList("double"),
                        Arrays.asList(
                                Collections.<String>emptyList(),
                                Collections.singletonList("double"))),
                new Descriptor(
                        "intaker.intakerArmStop",
                        Collections.<String>emptyList(),
                        Collections.<String>emptyList()));
    }

    public void start() throws IOException {
        server.start();
        simulation.start();
        runtime.publish(false, null, 0, 0, 0);
        MockLog.info("MockRobot", "Listening on http://127.0.0.1:" + PORT
                + (tlsEnabled ? " and https://127.0.0.1:" + HTTPS_PORT : ""));
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
