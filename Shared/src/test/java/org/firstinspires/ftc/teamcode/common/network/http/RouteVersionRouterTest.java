package org.firstinspires.ftc.teamcode.common.network.http;

import org.firstinspires.ftc.teamcode.common.network.*;
import org.junit.Test;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import static org.junit.Assert.*;

public class RouteVersionRouterTest {
    @Test public void legacyAndVersionTwoBodiesRoundTripWithoutRewriting() throws Exception {
        Fixture fixture = new Fixture();
        String old = "[{\"x\":0,\"y\":0}]";
        assertTrue(fixture.request("PUT", old, "0").startsWith("HTTP/1.1 201"));
        assertTrue(fixture.request("GET", "", null).endsWith(old));
        String versioned = "{\"version\":2,\"frames\":[{\"x\":1,\"y\":2,\"commands\":[]}]}";
        assertTrue(fixture.request("PUT", versioned, "1").startsWith("HTTP/1.1 200"));
        assertTrue(fixture.request("GET", "", null).endsWith(versioned));
        assertEquals(2, fixture.routes.puts);
    }

    @Test public void invalidEnvelopeIsRejectedBeforeWritingAnything() throws Exception {
        Fixture fixture = new Fixture();
        for (String json : new String[]{"{}", "{\"version\":3,\"frames\":[]}",
                "{\"version\":\"2\",\"frames\":[]}", "{\"version\":2}", "[] []"}) {
            String response = fixture.request("PUT", json, "0");
            assertTrue(response, response.startsWith("HTTP/1.1 400"));
            assertTrue(response, response.contains("\"error\":\"invalid_route_json\""));
        }
        assertEquals(0, fixture.routes.puts);
        assertNull(fixture.routes.get("route"));
    }

    @Test public void requiredRevisionHeaderStillPrecedesBodyValidation() throws Exception {
        Fixture fixture = new Fixture();
        String response = fixture.request("PUT", "{}", null);
        assertTrue(response, response.startsWith("HTTP/1.1 428"));
        assertEquals(0, fixture.routes.puts);
    }

    @Test public void versionedRoutesStillUseRevisionPreconditions() throws Exception {
        Fixture fixture = new Fixture();
        assertTrue(fixture.request("PUT", "[]", "0").startsWith("HTTP/1.1 201"));
        String response = fixture.request("PUT", "{\"version\":2,\"frames\":[]}", "0");
        assertTrue(response, response.startsWith("HTTP/1.1 412"));
        assertEquals("[]", fixture.routes.get("route").json);
    }

    private static final class Fixture {
        final SessionLease lease = new SessionLease();
        final String token = lease.open("test").token;
        final MemoryRoutes routes = new MemoryRoutes();
        final RobotApiRouter router = new RobotApiRouter(lease, routes, new ExecutionStateStore(),
                new RobotRuntimeStore(), new CommandCatalog(), new OpModeLifecycleService(),
                new CommandDispatcher());

        String request(String method, String body, String revision) throws Exception {
            byte[] bytes = body.getBytes(StandardCharsets.UTF_8);
            String headers = method + " /api/v2/routes/route HTTP/1.1\r\n"
                    + "Host: localhost\r\nX-Az-Session: " + token + "\r\n"
                    + (revision == null ? "" : "If-Match: " + revision + "\r\n")
                    + "Content-Length: " + bytes.length + "\r\n\r\n";
            ByteArrayOutputStream wire = new ByteArrayOutputStream();
            wire.write(headers.getBytes(StandardCharsets.UTF_8));
            wire.write(bytes);
            HttpRequest request = HttpRequest.read(new ByteArrayInputStream(wire.toByteArray()));
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            try (Socket socket = new Socket()) {
                router.handle(new HttpExchange(socket, out, request));
            }
            return new String(out.toByteArray(), StandardCharsets.UTF_8);
        }
    }

    private static final class MemoryRoutes implements RouteRepository {
        final Map<String, Entry> entries = new LinkedHashMap<>();
        int puts;
        long revision;
        @Override public List<Entry> list() { return new ArrayList<>(entries.values()); }
        @Override public Entry get(String name) { return entries.get(name); }
        @Override public PutResult put(String name, String json, Long expected) {
            Entry current = get(name);
            long actual = current == null ? 0 : current.revision;
            if (expected != null && expected != actual) return new PutResult(false, true, current);
            Entry next = new Entry(name, json, ++revision);
            entries.put(name, next);
            puts++;
            return new PutResult(true, false, next);
        }
        @Override public boolean delete(String name, Long expected) { return entries.remove(name) != null; }
        @Override public long changeRevision() { return revision; }
        @Override public String manifestJson() { return "{\"routes\":[]}"; }
    }
}
