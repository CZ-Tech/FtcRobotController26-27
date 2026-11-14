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
    private final RouteRepository routeStore;
    private final CommandCatalog commandCatalog;
    private final OpModeLifecycleService opModes;
    private final TelemetryStore telemetryStore;
    private final AutoConfigRegistry tuning;

    public RobotEventStream(SessionLease sessionLease,
                            RobotRuntimeStore runtimeStore,
                            ExecutionStateStore executionState,
                            RouteRepository routeStore,
                            CommandCatalog commandCatalog,
                            OpModeLifecycleService opModes) {
        this(sessionLease, runtimeStore, executionState, routeStore, commandCatalog, opModes,
                new TelemetryStore());
    }

    public RobotEventStream(SessionLease sessionLease,
                            RobotRuntimeStore runtimeStore,
                            ExecutionStateStore executionState,
                            RouteRepository routeStore,
                            CommandCatalog commandCatalog,
                            OpModeLifecycleService opModes,
                            TelemetryStore telemetryStore) {
        this(sessionLease, runtimeStore, executionState, routeStore, commandCatalog, opModes,
                telemetryStore, new AutoConfigRegistry(Object.class));
    }

    public RobotEventStream(SessionLease sessionLease,
                            RobotRuntimeStore runtimeStore,
                            ExecutionStateStore executionState,
                            RouteRepository routeStore,
                            CommandCatalog commandCatalog,
                            OpModeLifecycleService opModes,
                            TelemetryStore telemetryStore,
                            AutoConfigRegistry tuning) {
        this.tuning = tuning;
        this.sessionLease = sessionLease;
        this.runtimeStore = runtimeStore;
        this.executionState = executionState;
        this.routeStore = routeStore;
        this.commandCatalog = commandCatalog;
        this.opModes = opModes;
        this.telemetryStore = telemetryStore;
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
        long lastTuningRevision = -1;
        long lastOpModeRevision = -1;
        boolean runtimeSent = false;
        boolean lastOpModeActive = false;
        String lastOpModeName = null;
        long lastHeartbeat = 0;
        long nextFrameNanos = System.nanoTime();
        // Frames are latest-value; log entries are an ordered history. A client that read
        // the history over GET /telemetry/log passes the newest seq it saw as ?logSince=N
        // so nothing is repeated or missed; without it only new entries are streamed.
        long lastTelemetrySeq = -1;
        long lastLogSeq = parseLong(exchange.request.query.get("logSince"),
                telemetryStore.latestLogSequence());

        writeEvent(out, "hello", 0,
                "{\"protocol\":2,\"poseHz\":60,\"telemetryHz\":60,\"sessionTimeoutMs\":"
                        + SessionLease.DEFAULT_TIMEOUT_MS + "}");

        while (!Thread.currentThread().isInterrupted()) {
            long now = System.nanoTime() / 1_000_000L;
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

            TelemetryStore.Frame frame = telemetryStore.latest();
            if (frame.sequence != lastTelemetrySeq && frame.sequence > 0) {
                writeEvent(out, "telemetry", frame.sequence, frame.toJson());
                lastTelemetrySeq = frame.sequence;
            }

            for (TelemetryStore.LogEntry entry : telemetryStore.logSince(lastLogSeq)) {
                writeEvent(out, "log", entry.sequence, entry.toJson());
                lastLogSeq = entry.sequence;
            }

            ExecutionStateStore.Snapshot execution = executionState.snapshot();
            if (execution.revision != lastExecutionRevision) {
                writeEvent(out, "execution", execution.revision, execution.toJson());
                lastExecutionRevision = execution.revision;
            }

            OpModeLifecycleService.Snapshot opMode = opModes.snapshot();
            if (opMode.revision != lastOpModeRevision) {
                writeEvent(out, "opmode", opMode.revision, opModeJson(opMode));
                lastOpModeRevision = opMode.revision;
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

            long tuningRevision = tuning.revision();
            if (tuningRevision != lastTuningRevision) {
                writeEvent(out, "tuning", tuningRevision,
                        "{\"revision\":" + tuningRevision + "}");
                lastTuningRevision = tuningRevision;
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

    private static long parseLong(String text, long fallback) {
        if (text == null) return fallback;
        try {
            return Long.parseLong(text.trim());
        } catch (NumberFormatException e) {
            return fallback;
        }
    }

    private static boolean same(String a, String b) {
        return a == null ? b == null : a.equals(b);
    }

    private static String opModeJson(OpModeLifecycleService.Snapshot snapshot) {
        return "{\"revision\":" + snapshot.revision
                + ",\"controllerAvailable\":" + snapshot.controllerAvailable
                + ",\"phase\":\"" + snapshot.phase.name() + "\""
                + ",\"activeName\":"
                + (snapshot.activeName == null
                    ? "null"
                    : "\"" + JsonUtil.escape(snapshot.activeName) + "\"")
                + "}";
    }
}
