package org.firstinspires.ftc.teamcode.common.network;

import org.firstinspires.ftc.teamcode.common.network.http.HttpExchange;

import java.io.IOException;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.locks.LockSupport;

/**
 * Single-client SSE stream. Pose is latest-value data: old frames are never queued.
 */
public final class RobotEventStream {
    public static final long FRAME_PERIOD_NANOS = 1_000_000_000L / 60L;
    private static final long HEARTBEAT_MS = 1000;

    private final SessionLease sessionLease;
    private final RobotRuntimeStore runtimeStore;
    private final ExecutionStateStore executionState;
    private final RouteStore routeStore;
    private final CommandCatalog commandCatalog;

    public RobotEventStream(SessionLease sessionLease,
                            RobotRuntimeStore runtimeStore,
                            ExecutionStateStore executionState,
                            RouteStore routeStore,
                            CommandCatalog commandCatalog) {
        this.sessionLease = sessionLease;
        this.runtimeStore = runtimeStore;
        this.executionState = executionState;
        this.routeStore = routeStore;
        this.commandCatalog = commandCatalog;
    }

    /**
     * Blocks the current network worker for the lifetime of the stream.
     * No robot/hardware object is reachable from this method.
     */
    public void serve(HttpExchange exchange, String sessionToken) throws IOException {
        if (!sessionLease.validate(sessionToken)) {
            exchange.send(org.firstinspires.ftc.teamcode.common.network.http.HttpResponse.json(
                    401, "{\"error\":\"invalid_session\"}"));
            return;
        }

        OutputStream out = exchange.beginEventStream();
        long lastPoseSeq = -1;
        long lastExecutionRevision = -1;
        long lastRouteRevision = -1;
        long lastCommandRevision = -1;
        boolean runtimeSent = false;
        boolean lastOpModeActive = false;
        String lastOpModeName = null;
        long lastHeartbeat = 0;
        long nextFrameNanos = System.nanoTime();

        writeEvent(out, "hello", 0,
                "{\"protocol\":2,\"poseHz\":60,\"sessionTimeoutMs\":"
                        + SessionLease.DEFAULT_TIMEOUT_MS + "}");

        while (!Thread.currentThread().isInterrupted()) {
            long now = android.os.SystemClock.elapsedRealtime();
            if (!sessionLease.touch(sessionToken)) return;

            RobotRuntimeStore.Snapshot runtime = runtimeStore.latest();
            if (runtime.sequence != lastPoseSeq) {
                writeEvent(out, "pose", runtime.sequence, runtime.poseJson());
                lastPoseSeq = runtime.sequence;
            }

            if (!runtimeSent
                    || runtime.opModeActive != lastOpModeActive
                    || !same(runtime.opModeName, lastOpModeName)) {
                writeEvent(out, "runtime", runtime.sequence, runtime.runtimeJson());
                runtimeSent = true;
                lastOpModeActive = runtime.opModeActive;
                lastOpModeName = runtime.opModeName;
            }

            ExecutionStateStore.Snapshot execution = executionState.snapshot();
            if (execution.revision != lastExecutionRevision) {
                writeEvent(out, "execution", execution.revision, execution.toJson());
                lastExecutionRevision = execution.revision;
            }

            long routeRevision = routeStore.changeRevision();
            if (routeRevision != lastRouteRevision) {
                writeEvent(out, "routes", routeRevision,
                        "{\"revision\":" + routeRevision + "}");
                lastRouteRevision = routeRevision;
            }

            long commandRevision = commandCatalog.revision();
            if (commandRevision != lastCommandRevision) {
                writeEvent(out, "commands", commandRevision,
                        "{\"revision\":" + commandRevision + "}");
                lastCommandRevision = commandRevision;
            }

            if (now - lastHeartbeat >= HEARTBEAT_MS) {
                writeEvent(out, "heartbeat", now, "{\"t\":" + now + "}");
                lastHeartbeat = now;
            }

            nextFrameNanos += FRAME_PERIOD_NANOS;
            long remaining = nextFrameNanos - System.nanoTime();
            if (remaining > 0) {
                LockSupport.parkNanos(remaining);
            } else if (remaining < -FRAME_PERIOD_NANOS * 4) {
                // Do not try to replay missed frames after a long network stall.
                nextFrameNanos = System.nanoTime();
            }
            if (Thread.currentThread().isInterrupted()) return;
        }
    }

    private static void writeEvent(OutputStream out,
                                   String event,
                                   long id,
                                   String json) throws IOException {
        String payload = "event: " + event + "\n"
                + "id: " + id + "\n"
                + "data: " + json + "\n\n";
        out.write(payload.getBytes(StandardCharsets.UTF_8));
        out.flush();
    }

    private static boolean same(String a, String b) {
        return a == null ? b == null : a.equals(b);
    }
}
