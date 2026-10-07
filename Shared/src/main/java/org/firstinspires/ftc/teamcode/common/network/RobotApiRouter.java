package org.firstinspires.ftc.teamcode.common.network;

import org.firstinspires.ftc.teamcode.common.network.http.HttpExchange;
import org.firstinspires.ftc.teamcode.common.network.http.HttpHandler;
import org.firstinspires.ftc.teamcode.common.network.http.HttpRequest;
import org.firstinspires.ftc.teamcode.common.network.http.HttpResponse;
import org.json.JSONObject;

import java.util.List;

/** Network V2 router. Contains protocol logic only; never touches robot hardware. */
public final class RobotApiRouter implements HttpHandler {
    private static final String PREFIX = "/api/v2";

    private final SessionLease sessionLease;
    private final RouteRepository routeStore;
    private final ExecutionStateStore executionState;
    private final RobotRuntimeStore runtimeStore;
    private final CommandCatalog commandCatalog;
    private final OpModeLifecycleService opModes;
    private final RobotEventStream eventStream;

    public RobotApiRouter(SessionLease sessionLease,
                          RouteRepository routeStore,
                          ExecutionStateStore executionState,
                          RobotRuntimeStore runtimeStore,
                          CommandCatalog commandCatalog,
                          OpModeLifecycleService opModes) {
        this.sessionLease = sessionLease;
        this.routeStore = routeStore;
        this.executionState = executionState;
        this.runtimeStore = runtimeStore;
        this.commandCatalog = commandCatalog;
        this.opModes = opModes;
        this.eventStream = new RobotEventStream(
                sessionLease, runtimeStore, executionState, routeStore, commandCatalog, opModes);
    }

    @Override
    public void handle(HttpExchange exchange) throws Exception {
        HttpRequest request = exchange.request;
        String path = request.path;

        if ("GET".equals(request.method) && (PREFIX + "/health").equals(path)) {
            exchange.send(HttpResponse.json(200,
                    "{\"status\":\"ok\",\"protocol\":2,"
                            + "\"sessionOwned\":" + sessionLease.isOwned() + "}"));
            return;
        }

        if ((PREFIX + "/session").equals(path)) {
            handleSession(exchange);
            return;
        }

        String token = sessionToken(request);
        if (!sessionLease.validate(token)) {
            exchange.send(HttpResponse.json(
                    401, "{\"error\":\"invalid_session\"}"));
            return;
        }
        sessionLease.touch(token);

        if ("GET".equals(request.method) && (PREFIX + "/events").equals(path)) {
            eventStream.serve(exchange, token);
            return;
        }

        if ((PREFIX + "/opmodes").equals(path)) {
            if ("GET".equals(request.method)) {
                handleOpModeList(exchange);
            } else {
                methodNotAllowed(exchange);
            }
            return;
        }

        if ((PREFIX + "/opmode").equals(path)) {
            if ("GET".equals(request.method)) {
                exchange.send(HttpResponse.json(200, opModeJson(opModes.snapshot())));
            } else {
                methodNotAllowed(exchange);
            }
            return;
        }

        if ((PREFIX + "/opmode/init").equals(path)) {
            if ("POST".equals(request.method)) {
                handleOpModeAction(exchange, "init");
            } else {
                methodNotAllowed(exchange);
            }
            return;
        }

        if ((PREFIX + "/opmode/start").equals(path)) {
            if ("POST".equals(request.method)) {
                handleOpModeAction(exchange, "start");
            } else {
                methodNotAllowed(exchange);
            }
            return;
        }

        if ((PREFIX + "/opmode/stop").equals(path)) {
            if ("POST".equals(request.method)) {
                handleOpModeAction(exchange, "stop");
            } else {
                methodNotAllowed(exchange);
            }
            return;
        }

        if ((PREFIX + "/routes").equals(path)) {
            if ("GET".equals(request.method)) {
                exchange.send(HttpResponse.json(200, routeStore.manifestJson()));
            } else {
                methodNotAllowed(exchange);
            }
            return;
        }

        String routePrefix = PREFIX + "/routes/";
        if (path.startsWith(routePrefix)) {
            String name = path.substring(routePrefix.length());
            handleRoute(exchange, name);
            return;
        }

        if ((PREFIX + "/execution").equals(path)) {
            if ("GET".equals(request.method)) {
                exchange.send(HttpResponse.json(200, executionState.snapshot().toJson()));
            } else {
                methodNotAllowed(exchange);
            }
            return;
        }

        if ((PREFIX + "/commands").equals(path)) {
            if ("GET".equals(request.method)) {
                exchange.send(HttpResponse.json(200, commandCatalog.toJson()));
            } else {
                methodNotAllowed(exchange);
            }
            return;
        }

        if ("GET".equals(request.method) && (PREFIX + "/runtime").equals(path)) {
            RobotRuntimeStore.Snapshot runtime = runtimeStore.latest();
            exchange.send(HttpResponse.json(200,
                    "{\"runtime\":" + runtime.runtimeJson()
                            + ",\"pose\":" + runtime.poseJson() + "}"));
            return;
        }

        exchange.send(HttpResponse.json(404, "{\"error\":\"not_found\"}"));
    }

    private void handleSession(HttpExchange exchange) throws Exception {
        HttpRequest request = exchange.request;
        if ("POST".equals(request.method)) {
            String owner = "AzConductor";
            if (request.body.length > 0) {
                JSONObject body = new JSONObject(request.bodyUtf8());
                owner = body.optString("client", owner);
            }
            SessionLease.OpenResult result = sessionLease.open(owner);
            if (!result.acquired) {
                exchange.send(HttpResponse.json(
                        409,
                        "{\"error\":\"already_connected\","
                                + "\"owner\":\"" + JsonUtil.escape(sessionLease.owner()) + "\","
                                + "\"retryAfterMs\":" + result.expiresInMs + "}"));
                return;
            }
            exchange.send(HttpResponse.json(
                    201,
                    "{\"token\":\"" + result.token + "\","
                            + "\"expiresInMs\":" + result.expiresInMs + "}"));
            return;
        }

        if ("DELETE".equals(request.method)) {
            String token = sessionToken(request);
            if (!sessionLease.close(token)) {
                exchange.send(HttpResponse.json(
                        401, "{\"error\":\"invalid_session\"}"));
                return;
            }
            exchange.send(HttpResponse.empty(204));
            return;
        }

        methodNotAllowed(exchange);
    }

    private void handleRoute(HttpExchange exchange, String name) throws Exception {
        HttpRequest request = exchange.request;
        if (name == null || name.isEmpty()) {
            exchange.send(HttpResponse.json(400, "{\"error\":\"empty_route_name\"}"));
            return;
        }

        if ("GET".equals(request.method)) {
            RouteRepository.Entry entry = routeStore.get(name);
            if (entry == null) {
                exchange.send(HttpResponse.json(404, "{\"error\":\"route_not_found\"}"));
                return;
            }
            exchange.send(HttpResponse
                    .text(200, "application/json; charset=utf-8", entry.json)
                    .withHeader("ETag", "\"" + entry.revision + "\"")
                    .withHeader("X-Route-Revision", Long.toString(entry.revision)));
            return;
        }

        if ("PUT".equals(request.method)) {
            Long expected = parseRequiredIfMatch(request);
            if (expected == null) {
                exchange.send(HttpResponse.json(
                        428, "{\"error\":\"if_match_required\"}"));
                return;
            }
            RouteRepository.PutResult result =
                    routeStore.put(name, request.bodyUtf8(), expected);
            if (result.preconditionFailed) {
                long actual = result.entry == null ? 0 : result.entry.revision;
                exchange.send(HttpResponse.json(
                        412,
                        "{\"error\":\"revision_mismatch\",\"actual\":" + actual + "}"));
                return;
            }
            exchange.send(HttpResponse.json(
                    expected == 0 ? 201 : 200,
                    "{\"name\":\"" + JsonUtil.escape(name) + "\","
                            + "\"revision\":" + result.entry.revision + "}"));
            return;
        }

        if ("DELETE".equals(request.method)) {
            Long expected = parseRequiredIfMatch(request);
            if (expected == null) {
                exchange.send(HttpResponse.json(
                        428, "{\"error\":\"if_match_required\"}"));
                return;
            }
            RouteRepository.Entry current = routeStore.get(name);
            if (current == null) {
                exchange.send(HttpResponse.json(404, "{\"error\":\"route_not_found\"}"));
                return;
            }
            if (current.revision != expected) {
                exchange.send(HttpResponse.json(
                        412,
                        "{\"error\":\"revision_mismatch\",\"actual\":"
                                + current.revision + "}"));
                return;
            }
            routeStore.delete(name, expected);
            exchange.send(HttpResponse.empty(204));
            return;
        }

        methodNotAllowed(exchange);
    }

    private void handleOpModeList(HttpExchange exchange) throws Exception {
        if (!opModes.controllerAvailable()) {
            exchange.send(HttpResponse.json(
                    503, "{\"error\":\"controller_unavailable\"}"));
            return;
        }

        List<OpModeLifecycleService.Descriptor> descriptors = opModes.listAutonomous();
        StringBuilder json = new StringBuilder("{\"opModes\":[");
        for (int i = 0; i < descriptors.size(); i++) {
            if (i > 0) json.append(',');
            OpModeLifecycleService.Descriptor item = descriptors.get(i);
            json.append("{\"name\":\"")
                    .append(JsonUtil.escape(item.name))
                    .append("\",\"group\":\"")
                    .append(JsonUtil.escape(item.group))
                    .append("\"}");
        }
        json.append("]}");
        exchange.send(HttpResponse.json(200, json.toString()));
    }

    private void handleOpModeAction(HttpExchange exchange, String action) throws Exception {
        JSONObject body = new JSONObject(exchange.request.bodyUtf8());
        String name = body.optString("name", "");
        if (name.isEmpty()) {
            exchange.send(HttpResponse.json(
                    400, "{\"error\":\"missing_opmode_name\"}"));
            return;
        }
        if (!body.has("expectedRevision")) {
            exchange.send(HttpResponse.json(
                    428, "{\"error\":\"expected_revision_required\"}"));
            return;
        }

        long expectedRevision = body.optLong("expectedRevision", Long.MIN_VALUE);
        OpModeLifecycleService.ActionResult result;
        switch (action) {
            case "init":
                result = opModes.init(name, expectedRevision);
                break;
            case "start":
                result = opModes.start(name, expectedRevision);
                break;
            case "stop":
                result = opModes.stop(name, expectedRevision);
                break;
            default:
                throw new AssertionError(action);
        }

        if (!result.accepted) {
            int status;
            if ("controller_unavailable".equals(result.code)) {
                status = 503;
            } else if ("opmode_not_found".equals(result.code)) {
                status = 404;
            } else {
                status = 409;
            }
            exchange.send(HttpResponse.json(
                    status,
                    "{\"error\":\"" + JsonUtil.escape(result.code) + "\","
                            + "\"message\":"
                            + (result.message == null
                                ? "null"
                                : "\"" + JsonUtil.escape(result.message) + "\"")
                            + ",\"opMode\":" + opModeJson(result.snapshot) + "}"));
            return;
        }

        exchange.send(HttpResponse.json(
                202,
                "{\"accepted\":true,\"action\":\"" + action + "\","
                        + "\"name\":\"" + JsonUtil.escape(name) + "\"}"));
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

    private static String sessionToken(HttpRequest request) {
        String header = request.header("x-az-session");
        if (header != null && !header.isEmpty()) return header;
        return request.query.get("session");
    }

    private static Long parseRequiredIfMatch(HttpRequest request) {
        String raw = request.header("if-match");
        if (raw == null || raw.trim().isEmpty()) return null;
        raw = raw.trim();
        if (raw.startsWith("\"") && raw.endsWith("\"") && raw.length() >= 2) {
            raw = raw.substring(1, raw.length() - 1);
        }
        try {
            return Long.parseLong(raw);
        } catch (NumberFormatException e) {
            return null;
        }
    }

    private static void methodNotAllowed(HttpExchange exchange) throws Exception {
        exchange.send(HttpResponse.json(
                405, "{\"error\":\"method_not_allowed\"}"));
    }
}
