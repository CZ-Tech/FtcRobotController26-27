package org.firstinspires.ftc.teamcode.common.network;

import android.content.Context;

import org.firstinspires.ftc.teamcode.common.network.http.RobotHttpServer;

import java.io.IOException;

/**
 * Hardware-free composition root for the new network stack.
 *
 * <p>Not wired into Robot or any OpMode in this phase. The later integration layer can
 * consume {@link #mailbox}, publish {@link #runtime}, and update {@link #execution}.</p>
 */
public final class RobotNetworkV2 {
    public static final int DEFAULT_PORT = 8888;

    public final SessionLease session = new SessionLease();
    public final RouteStore routes;
    public final ControlMailbox mailbox = new ControlMailbox();
    public final ExecutionStateStore execution = new ExecutionStateStore();
    public final RobotRuntimeStore runtime = new RobotRuntimeStore();
    public final CommandCatalog commands = new CommandCatalog();

    private final RobotHttpServer server;

    public RobotNetworkV2(Context context) {
        this(context, DEFAULT_PORT);
    }

    public RobotNetworkV2(Context context, int port) {
        routes = new RouteStore(context);
        RobotApiRouter router = new RobotApiRouter(
                session, routes, mailbox, execution, runtime, commands);
        server = new RobotHttpServer(port, router);
    }

    public void start() throws IOException {
        server.start();
    }

    public void stop() {
        server.stop();
    }

    public boolean isRunning() {
        return server.isRunning();
    }
}
