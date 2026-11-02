package org.firstinspires.ftc.teamcode.common.network;

import android.content.Context;
import android.util.Log;

import org.firstinspires.ftc.teamcode.common.network.http.RobotHttpServer;

import java.io.IOException;

/**
 * Hardware-free composition root for the new network stack.
 *
 * <p>OpModes publish {@link #runtime} and {@link #execution}, and may attach a
 * {@link #commandDispatcher} sink to accept remote commands.</p>
 */
public final class RobotNetworkV2 {
    private static final String TAG = "RobotNetworkV2";
    public static final int DEFAULT_PORT = 8888;

    public final SessionLease session = new SessionLease();
    public final RouteStore routes;
    public final ExecutionStateStore execution = new ExecutionStateStore();
    public final RobotRuntimeStore runtime = new RobotRuntimeStore();
    public final CommandCatalog commands = new CommandCatalog();
    public final OpModeLifecycleService opModes = new OpModeLifecycleService();
    /** An OpMode that runs remote commands attaches a sink here while it is RUNNING. */
    public final CommandDispatcher commandDispatcher = new CommandDispatcher();

    private final RobotHttpServer server;

    public RobotNetworkV2(Context context) {
        this(context, DEFAULT_PORT);
    }

    public RobotNetworkV2(Context context, int port) {
        routes = new RouteStore(context);
        try {
            commands.publish(SubsystemCommandBinding.subsystemCommands().descriptors());
        } catch (RuntimeException | LinkageError e) {
            Log.e(TAG, "Failed to publish subsystem commands", e);
        }
        RobotApiRouter router = new RobotApiRouter(
                session, routes, execution, runtime, commands, opModes, commandDispatcher);
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
