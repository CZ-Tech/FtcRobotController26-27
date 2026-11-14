package org.firstinspires.ftc.mockrobot;

import org.firstinspires.ftc.teamcode.common.network.CommandCatalog;
import org.firstinspires.ftc.teamcode.common.network.CommandDispatcher;
import org.firstinspires.ftc.teamcode.common.network.ExecutionStateStore;
import org.firstinspires.ftc.teamcode.common.network.OpModeLifecycleService;
import org.firstinspires.ftc.teamcode.common.network.RobotApiRouter;
import org.firstinspires.ftc.teamcode.common.network.RobotRuntimeStore;
import org.firstinspires.ftc.teamcode.common.network.SessionLease;
import org.firstinspires.ftc.teamcode.common.network.TelemetryStore;
import org.firstinspires.ftc.teamcode.common.network.http.RobotHttpServer;
import org.json.JSONArray;
import org.json.JSONObject;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.io.BufferedReader;
import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.ServerSocket;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/** Telemetry over the real router: GET /telemetry, /telemetry/log and the SSE events. */
public class TelemetryApiTest {
    private static final class Reply {
        final int status;
        final String body;

        Reply(int status, String body) {
            this.status = status;
            this.body = body;
        }

        JSONObject json() {
            return new JSONObject(body);
        }
    }

    private static final class Event {
        final String name;
        final String data;

        Event(String name, String data) {
            this.name = name;
            this.data = data;
        }
    }

    private final TelemetryStore store = new TelemetryStore(3);
    private RobotHttpServer server;
    private int port;
    private String token;

    @Before
    public void setUp() throws Exception {
        RobotApiRouter router = new RobotApiRouter(
                new SessionLease(),
                new MockRouteStore(
                        Files.createTempDirectory("telemetry-api-test").resolve("routes.json")),
                new ExecutionStateStore(),
                new RobotRuntimeStore(),
                new CommandCatalog(),
                new OpModeLifecycleService(),
                new CommandDispatcher(),
                store);
        try (ServerSocket probe = new ServerSocket(0)) {
            port = probe.getLocalPort();
        }
        server = new RobotHttpServer(port, router);
        server.start();
        token = request("POST", "/api/v2/session", "{}", null).json().getString("token");
    }

    @After
    public void tearDown() {
        server.stop();
    }

    private static List<TelemetryStore.Item> items(String caption, double value, String unit) {
        return Collections.singletonList(new TelemetryStore.Item(
                caption, value + " " + unit, TelemetryStore.TYPE_NUMBER, value, unit));
    }

    private void publish(String caption, double value) {
        store.publishFrame(Collections.singletonList(
                new TelemetryStore.Line(null, items(caption, value, "rpm"))));
    }

    private Reply request(String method, String path, String body, String session)
            throws Exception {
        HttpURLConnection connection =
                (HttpURLConnection) new URL("http://127.0.0.1:" + port + path).openConnection();
        connection.setRequestMethod(method);
        connection.setConnectTimeout(3000);
        connection.setReadTimeout(3000);
        if (session != null) connection.setRequestProperty("X-Az-Session", session);
        if (body != null) {
            connection.setDoOutput(true);
            connection.setRequestProperty("Content-Type", "application/json");
            try (OutputStream out = connection.getOutputStream()) {
                out.write(body.getBytes(StandardCharsets.UTF_8));
            }
        }
        int status = connection.getResponseCode();
        InputStream stream = status >= 400
                ? connection.getErrorStream()
                : connection.getInputStream();
        ByteArrayOutputStream bytes = new ByteArrayOutputStream();
        if (stream != null) {
            byte[] buffer = new byte[1024];
            int n;
            while ((n = stream.read(buffer)) > 0) bytes.write(buffer, 0, n);
        }
        return new Reply(status, new String(bytes.toByteArray(), StandardCharsets.UTF_8));
    }

    /** Reads events from /events until {@code count} events named {@code wanted} arrived. */
    private List<Event> readEvents(String query, String wanted, int count) throws Exception {
        HttpURLConnection connection = (HttpURLConnection) new URL(
                "http://127.0.0.1:" + port + "/api/v2/events?" + query).openConnection();
        connection.setRequestProperty("X-Az-Session", token);
        connection.setConnectTimeout(3000);
        connection.setReadTimeout(3000);
        List<Event> found = new ArrayList<>();
        try (BufferedReader reader = new BufferedReader(new InputStreamReader(
                connection.getInputStream(), StandardCharsets.UTF_8))) {
            String name = null;
            String line;
            while (found.size() < count && (line = reader.readLine()) != null) {
                if (line.startsWith("event: ")) {
                    name = line.substring(7);
                } else if (line.startsWith("data: ") && wanted.equals(name)) {
                    found.add(new Event(name, line.substring(6)));
                }
            }
        } finally {
            connection.disconnect();
        }
        return found;
    }

    @Test
    public void telemetryEndpointReturnsTheLatestFrameWithTypedValues() throws Exception {
        publish("rpm", 3120.5);
        Reply reply = request("GET", "/api/v2/telemetry", null, token);

        assertEquals(reply.body, 200, reply.status);
        JSONObject frame = reply.json().getJSONObject("frame");
        assertEquals(1, frame.getLong("seq"));
        JSONObject item = frame.getJSONArray("lines").getJSONObject(0)
                .getJSONArray("items").getJSONObject(0);
        assertEquals("rpm", item.getString("caption"));
        assertEquals("number", item.getString("type"));
        assertEquals(3120.5, item.getDouble("value"), 0);
        assertEquals("rpm", item.getString("unit"));
    }

    @Test
    public void telemetryRequiresASessionAndOnlyAllowsGet() throws Exception {
        assertEquals(401, request("GET", "/api/v2/telemetry", null, null).status);
        assertEquals(405, request("POST", "/api/v2/telemetry", "{}", token).status);
        assertEquals(405, request("POST", "/api/v2/telemetry/log", "{}", token).status);
    }

    @Test
    public void logEndpointReturnsOrderedEntriesAfterTheGivenSequence() throws Exception {
        store.appendLog(TelemetryStore.KIND_LOG, "one");
        store.appendLog(TelemetryStore.KIND_LOG, "two");
        store.appendLog(TelemetryStore.KIND_SPEAK, "three");

        JSONObject all = request("GET", "/api/v2/telemetry/log", null, token).json();
        assertEquals(3, all.getLong("latestSeq"));
        JSONArray entries = all.getJSONArray("entries");
        assertEquals(3, entries.length());
        assertEquals("one", entries.getJSONObject(0).getString("text"));
        assertEquals("speak", entries.getJSONObject(2).getString("kind"));

        JSONArray tail = request("GET", "/api/v2/telemetry/log?since=2", null, token)
                .json().getJSONArray("entries");
        assertEquals(1, tail.length());
        assertEquals(3, tail.getJSONObject(0).getLong("seq"));

        assertEquals(400,
                request("GET", "/api/v2/telemetry/log?since=abc", null, token).status);
    }

    @Test
    public void logHistoryIsBoundedAndSequencesRevealTheGap() throws Exception {
        for (int i = 1; i <= 5; i++) store.appendLog(TelemetryStore.KIND_LOG, "m" + i);

        JSONArray entries = request("GET", "/api/v2/telemetry/log", null, token)
                .json().getJSONArray("entries");
        assertEquals(3, entries.length());
        assertEquals(3, entries.getJSONObject(0).getLong("seq"));
        assertEquals("m5", entries.getJSONObject(2).getString("text"));
    }

    @Test
    public void eventStreamPushesTelemetryFrames() throws Exception {
        publish("rpm", 100);
        List<Event> events = readEvents("", "telemetry", 1);

        assertEquals(1, events.size());
        JSONObject frame = new JSONObject(events.get(0).data);
        assertEquals(1, frame.getLong("seq"));
        assertEquals(100.0, frame.getJSONArray("lines").getJSONObject(0)
                .getJSONArray("items").getJSONObject(0).getDouble("value"), 0);
    }

    @Test
    public void eventStreamReplaysLogEntriesFromLogSince() throws Exception {
        store.appendLog(TelemetryStore.KIND_LOG, "a");
        store.appendLog(TelemetryStore.KIND_LOG, "b");
        store.appendLog(TelemetryStore.KIND_LOG, "c");

        List<Event> events = readEvents("logSince=1", "log", 2);

        assertEquals(2, events.size());
        assertEquals(2, new JSONObject(events.get(0).data).getLong("seq"));
        assertEquals("c", new JSONObject(events.get(1).data).getString("text"));
    }

    @Test
    public void eventStreamWithoutLogSinceSkipsOldLogEntries() throws Exception {
        store.appendLog(TelemetryStore.KIND_LOG, "old");
        Thread writer = new Thread(() -> {
            try {
                Thread.sleep(300);
            } catch (InterruptedException ignored) {}
            store.appendLog(TelemetryStore.KIND_LOG, "new");
        });
        writer.start();

        List<Event> events = readEvents("", "log", 1);
        writer.join();

        assertEquals(1, events.size());
        assertTrue(events.get(0).data, events.get(0).data.contains("\"text\":\"new\""));
    }
}
