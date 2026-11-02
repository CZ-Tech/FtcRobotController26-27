package org.firstinspires.ftc.mockrobot;

import org.firstinspires.ftc.teamcode.common.network.CommandCatalog;
import org.firstinspires.ftc.teamcode.common.network.CommandCatalog.Descriptor;
import org.firstinspires.ftc.teamcode.common.network.CommandDispatcher;
import org.firstinspires.ftc.teamcode.common.network.ExecutionStateStore;
import org.firstinspires.ftc.teamcode.common.network.OpModeLifecycleService;
import org.firstinspires.ftc.teamcode.common.network.RobotApiRouter;
import org.firstinspires.ftc.teamcode.common.network.RobotRuntimeStore;
import org.firstinspires.ftc.teamcode.common.network.SessionLease;
import org.firstinspires.ftc.teamcode.common.network.http.RobotHttpServer;
import org.json.JSONObject;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.ServerSocket;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/** Drives POST /api/v2/commands/run through the real router over HTTP. */
public class CommandRunApiTest {
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

    private static final class Call {
        final String name;
        final Object[] args;

        Call(String name, Object[] args) {
            this.name = name;
            this.args = args;
        }
    }

    private final ExecutionStateStore execution = new ExecutionStateStore();
    private final CommandCatalog catalog = new CommandCatalog();
    private final CommandDispatcher dispatcher = new CommandDispatcher();
    private final List<Call> calls = new ArrayList<>();
    private OpModeLifecycleService opModes;
    private RobotHttpServer server;
    private int port;
    private String token;
    private CommandDispatcher.Sink sink;
    private boolean sinkAccepts = true;

    @Before
    public void setUp() throws Exception {
        startServer(true);
    }

    @After
    public void tearDown() {
        server.stop();
    }

    private void startServer(boolean controllerAttached) throws Exception {
        if (server != null) server.stop();

        opModes = new OpModeLifecycleService();
        if (controllerAttached) {
            opModes.attach(new OpModeLifecycleService.Backend() {
                @Override
                public List<OpModeLifecycleService.Descriptor> listAutonomous() {
                    return Collections.emptyList();
                }

                @Override
                public OpModeLifecycleService.BackendResult init(
                        String name, OpModeLifecycleService.Snapshot snapshot) {
                    return OpModeLifecycleService.BackendResult.accepted();
                }

                @Override
                public OpModeLifecycleService.BackendResult start(
                        String name, OpModeLifecycleService.Snapshot snapshot) {
                    return OpModeLifecycleService.BackendResult.accepted();
                }

                @Override
                public OpModeLifecycleService.BackendResult stop(
                        String name, OpModeLifecycleService.Snapshot snapshot) {
                    return OpModeLifecycleService.BackendResult.accepted();
                }
            });
        }

        catalog.publish(Arrays.asList(
                new Descriptor("shoot",
                        Arrays.asList("count", "power"),
                        Arrays.asList("int", "double"),
                        Arrays.asList(null, "0.8"),
                        Arrays.asList(
                                Collections.singletonList("int"),
                                Arrays.asList("int", "double"))),
                new Descriptor("spin",
                        Collections.singletonList("power"),
                        Collections.singletonList("double"),
                        Arrays.asList(
                                Collections.<String>emptyList(),
                                Collections.singletonList("double")))));
        sink = (name, args) -> {
            calls.add(new Call(name, args));
            return sinkAccepts;
        };
        dispatcher.attach(sink);

        MockRouteStore routes = new MockRouteStore(
                Files.createTempDirectory("command-run-test").resolve("routes.json"));
        RobotApiRouter router = new RobotApiRouter(
                new SessionLease(),
                routes,
                execution,
                new RobotRuntimeStore(),
                catalog,
                opModes,
                dispatcher);

        try (ServerSocket probe = new ServerSocket(0)) {
            port = probe.getLocalPort();
        }
        server = new RobotHttpServer(port, router);
        server.start();

        token = new JSONObject(request("POST", "/api/v2/session", "{}", null).body)
                .getString("token");
    }

    private void running() {
        opModes.publishInit("Test");
        opModes.publishRunning("Test");
    }

    private String run(String name, String args) {
        return run(name, args, opModes.snapshot().revision);
    }

    private String run(String name, String args, long revision) {
        return "{\"name\":\"" + name + "\","
                + (args == null ? "" : "\"args\":" + args + ",")
                + "\"expectedRevision\":" + revision + "}";
    }

    private Reply post(String body) throws Exception {
        return request("POST", "/api/v2/commands/run", body, token);
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

    private static void assertError(Reply reply, int status, String code) {
        assertEquals(reply.body, status, reply.status);
        assertEquals(reply.body, code, reply.json().getString("error"));
    }

    @Test
    public void acceptsValidCommandAndPassesConvertedArguments() throws Exception {
        running();
        Reply reply = post(run("shoot", "[\"3\",\"0.8\"]"));

        assertEquals(reply.body, 202, reply.status);
        assertEquals("{\"accepted\":true,\"name\":\"shoot\"}", reply.body);
        assertEquals(1, calls.size());
        assertEquals("shoot", calls.get(0).name);
        assertArrayEquals(new Object[]{3, 0.8}, calls.get(0).args);
    }

    @Test
    public void argsDefaultToEmptyAndOverloadsAcceptEachArgumentCount() throws Exception {
        running();
        assertEquals(202, post(run("spin", null)).status);
        assertEquals(202, post(run("spin", "[]")).status);
        assertEquals(202, post(run("spin", "[\"0.5\"]")).status);
        assertEquals(3, calls.size());
        assertArrayEquals(new Object[0], calls.get(0).args);
        assertArrayEquals(new Object[]{0.5}, calls.get(2).args);

        assertError(post(run("spin", "[\"1\",\"2\"]")), 400, "invalid_command_args");
    }

    @Test
    public void defaultedTrailingArgumentsMayBeOmitted() throws Exception {
        running();
        Reply reply = post(run("shoot", "[\"3\"]"));

        assertEquals(reply.body, 202, reply.status);
        assertArrayEquals(new Object[]{3}, calls.get(0).args);
        // Omitting a required argument is still an error, and so is a bad value for
        // an argument that is present.
        assertError(post(run("shoot", "[]")), 400, "invalid_command_args");
        assertError(post(run("shoot", "[\"3\",\"fast\"]")), 400, "invalid_command_args");
    }

    @Test
    public void catalogListsParamDefaultsWithNullForRequired() throws Exception {
        JSONObject root = request("GET", "/api/v2/commands", null, token).json();
        JSONObject shoot = root.getJSONArray("commands").getJSONObject(0);

        assertEquals("shoot", shoot.getString("name"));
        assertEquals("[\"count\",\"power\"]", shoot.getJSONArray("paramNames").toString());
        assertEquals("[\"int\",\"double\"]", shoot.getJSONArray("paramTypes").toString());
        assertEquals("[null,\"0.8\"]", shoot.getJSONArray("paramDefaults").toString());

        JSONObject spin = root.getJSONArray("commands").getJSONObject(1);
        assertEquals("[null]", spin.getJSONArray("paramDefaults").toString());
    }

    @Test
    public void requiresSessionAndPost() throws Exception {
        running();
        assertError(request("POST", "/api/v2/commands/run", run("shoot", null), null),
                401, "invalid_session");
        assertError(request("GET", "/api/v2/commands/run", null, token),
                405, "method_not_allowed");
    }

    @Test
    public void missingNameIsReportedWithoutOpModeSnapshot() throws Exception {
        running();
        assertEquals("{\"error\":\"missing_command_name\"}",
                post("{\"expectedRevision\":1}").body);
        assertEquals(400, post("{\"name\":\"\",\"expectedRevision\":1}").status);
        assertEquals(400, post("{\"name\":7,\"expectedRevision\":1}").status);
        assertEquals(400, post("not json").status);
        assertEquals(400, post("").status);
    }

    @Test
    public void missingRevisionIsChecked_afterNameAndBeforeEverythingElse() throws Exception {
        Reply reply = post("{\"name\":\"nope\"}");
        assertEquals("{\"error\":\"expected_revision_required\"}", reply.body);
        assertEquals(428, reply.status);
    }

    @Test
    public void staleRevisionIsStateChangedWithSnapshot() throws Exception {
        running();
        long current = opModes.snapshot().revision;
        Reply reply = post(run("shoot", "[\"1\",\"1\"]", current - 1));

        assertError(reply, 409, "state_changed");
        JSONObject opMode = reply.json().getJSONObject("opMode");
        assertEquals(current, opMode.getLong("revision"));
        assertEquals("RUNNING", opMode.getString("phase"));
        assertEquals("Test", opMode.getString("activeName"));
        assertTrue(calls.isEmpty());
    }

    @Test
    public void unavailableControllerIs503() throws Exception {
        startServer(false);
        assertError(post(run("shoot", "[\"1\",\"1\"]")), 503, "controller_unavailable");
    }

    @Test
    public void unknownCommandIs404EvenWhenNotRunning() throws Exception {
        // Step 5 comes before the phase check in step 7.
        assertError(post(run("nope", null)), 404, "command_not_found");
    }

    @Test
    public void invalidArgumentsAre400EvenWhenNotRunning() throws Exception {
        assertError(post(run("shoot", "[]")), 400, "invalid_command_args");
        assertError(post(run("shoot", "[\"1\",\"1\",\"1\"]")), 400, "invalid_command_args");
        assertError(post(run("shoot", "[\"x\",\"1\"]")), 400, "invalid_command_args");
        assertError(post(run("shoot", "[1,2]")), 400, "invalid_command_args");
        assertError(post(run("shoot", "\"1\"")), 400, "invalid_command_args");
        assertError(post(run("shoot", "[\"1\",null]")), 400, "invalid_command_args");
        assertTrue(calls.isEmpty());
    }

    @Test
    public void onlyRunningPhaseAcceptsCommands() throws Exception {
        assertError(post(run("shoot", "[\"1\",\"1\"]")), 409, "command_not_ready");

        opModes.publishInit("Test");
        assertError(post(run("shoot", "[\"1\",\"1\"]")), 409, "command_not_ready");

        running();
        assertEquals(202, post(run("shoot", "[\"1\",\"1\"]")).status);

        opModes.publishStopped();
        assertError(post(run("shoot", "[\"1\",\"1\"]")), 409, "command_not_ready");
        assertEquals(1, calls.size());
    }

    @Test
    public void runningOpModeWithoutSinkIsNotReady() throws Exception {
        running();
        dispatcher.detach(sink);
        Reply reply = post(run("shoot", "[\"1\",\"1\"]"));
        assertError(reply, 409, "command_not_ready");
        assertTrue(calls.isEmpty());
    }

    @Test
    public void sinkRefusingTheCommandIsNotReady() throws Exception {
        running();
        sinkAccepts = false;
        assertError(post(run("shoot", "[\"1\",\"1\"]")), 409, "command_not_ready");
    }

    @Test
    public void executingRouteBlocksCommands() throws Exception {
        running();
        execution.publish(ExecutionStateStore.State.RUNNING, 1, "Auto");
        Reply reply = post(run("shoot", "[\"1\",\"1\"]"));

        assertError(reply, 409, "route_executing");
        assertEquals("A route is being executed", reply.json().getString("message"));
        assertTrue(calls.isEmpty());

        execution.publish(ExecutionStateStore.State.IDLE, 1, "Auto");
        assertEquals(202, post(run("shoot", "[\"1\",\"1\"]")).status);
    }

    @Test
    public void errorsAfterRevisionCheckCarryOpModeSnapshot() throws Exception {
        Reply reply = post(run("nope", null));
        assertEquals("STOPPED", reply.json().getJSONObject("opMode").getString("phase"));
    }
}
