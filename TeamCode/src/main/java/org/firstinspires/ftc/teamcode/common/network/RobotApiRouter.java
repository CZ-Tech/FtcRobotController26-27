package org.firstinspires.ftc.teamcode.common.network;

import org.firstinspires.ftc.teamcode.common.network.http.HttpExchange;
import org.firstinspires.ftc.teamcode.common.network.http.HttpHandler;
import org.firstinspires.ftc.teamcode.common.network.http.HttpRequest;
import org.firstinspires.ftc.teamcode.common.network.http.HttpResponse;
import org.json.JSONArray;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.List;

/** Network V2 router. Contains protocol logic only; never touches robot hardware. */
public final class RobotApiRouter implements HttpHandler {
    private static final String PREFIX = "/api/v2";

    private final SessionLease sessionLease;
    private final RouteStore routeStore;
    private final ControlMailbox mailbox;
    private final ExecutionStateStore executionState;
    private final RobotRuntimeStore runtimeStore;
    private final CommandCatalog commandCatalog;
    private final RobotEventStream eventStream;

    public RobotApiRouter(SessionLease sessionLease,
                          RouteStore routeStore,
                          ControlMailbox mailbox,
                          ExecutionStateStore executionState,
                          RobotRuntimeStore runtimeStore,
                          CommandCatalog commandCatalog) {
        this.sessionLease = sessionLease;
        this.routeStore = routeStore;
        this.mailbox = mailbox;
        this.executionState = executionState;
        this.runtimeStore = runtimeStore;
        this.commandCatalog = commandCatalog;
        this.eventStream = new RobotEventStream(
                sessionLease, runtimeStore, executionState, routeStore, commandCatalog);
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

        if ((PREFIX + "/executions").equals(path)) {
            if ("POST".equals(request.method)) {
                handleExecutionRequest(exchange);
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

        String commandPrefix = PREFIX + "/commands/";
        if (path.startsWith(commandPrefix)) {
            if ("POST".equals(request.method)) {
                handleCommand(exchange, path.substring(commandPrefix.length()));
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
            RouteStore.Entry entry = routeStore.get(name);
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
            RouteStore.PutResult result = routeStore.put(name, request.bodyUtf8(), expected);
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
            RouteStore.Entry current = routeStore.get(name);
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

    private void handleExecutionRequest(HttpExchange exchange) throws Exception {
        if (executionState.snapshot().state == ExecutionStateStore.State.NOT_READY) {
            exchange.send(HttpResponse.json(
                    503, "{\"error\":\"control_thread_not_ready\"}"));
            return;
        }
        if (mailbox.hasPending()
                || executionState.snapshot().state == ExecutionStateStore.State.RUNNING
                || executionState.snapshot().state == ExecutionStateStore.State.QUEUED) {
            exchange.send(HttpResponse.json(409, "{\"error\":\"robot_busy\"}"));
            return;
        }

        JSONObject body = new JSONObject(exchange.request.bodyUtf8());
        String type = body.optString("type", "");
        ControlRequest queued;
        String subject;
        if ("saved".equals(type)) {
            String path = body.optString("path", "");
            if (path.isEmpty() || routeStore.get(path) == null) {
                exchange.send(HttpResponse.json(
                        404, "{\"error\":\"route_not_found\"}"));
                return;
            }
            queued = mailbox.offerSavedPath(path);
            subject = path;
        } else if ("inline".equals(type)) {
            Object trajectory = body.opt("trajectory");
            if (trajectory == null || trajectory == JSONObject.NULL) {
                exchange.send(HttpResponse.json(
                        400, "{\"error\":\"missing_trajectory\"}"));
                return;
            }
            queued = mailbox.offerInlinePath(
                    trajectory instanceof String ? (String) trajectory : trajectory.toString());
            subject = "inline";
        } else {
            exchange.send(HttpResponse.json(
                    400, "{\"error\":\"invalid_execution_type\"}"));
            return;
        }

        if (queued == null) {
            exchange.send(HttpResponse.json(409, "{\"error\":\"mailbox_full\"}"));
            return;
        }
        executionState.publish(ExecutionStateStore.State.QUEUED, queued.id, subject);
        exchange.send(HttpResponse.json(
                202, "{\"requestId\":" + queued.id + ",\"state\":\"QUEUED\"}"));
    }

    private void handleCommand(HttpExchange exchange, String name) throws Exception {
        if (executionState.snapshot().state == ExecutionStateStore.State.NOT_READY) {
            exchange.send(HttpResponse.json(
                    503, "{\"error\":\"control_thread_not_ready\"}"));
            return;
        }
        if (mailbox.hasPending()
                || executionState.snapshot().state == ExecutionStateStore.State.QUEUED
                || executionState.snapshot().state == ExecutionStateStore.State.RUNNING) {
            exchange.send(HttpResponse.json(409, "{\"error\":\"robot_busy\"}"));
            return;
        }
        boolean known = false;
        for (CommandCatalog.Descriptor descriptor : commandCatalog.list()) {
            if (descriptor.name.equals(name)) {
                known = true;
                break;
            }
        }
        if (!known) {
            exchange.send(HttpResponse.json(
                    404, "{\"error\":\"command_not_found\"}"));
            return;
        }

        List<Object> args = new ArrayList<>();
        if (exchange.request.body.length > 0) {
            JSONObject body = new JSONObject(exchange.request.bodyUtf8());
            JSONArray array = body.optJSONArray("args");
            if (array != null) {
                for (int i = 0; i < array.length(); i++) args.add(array.get(i));
            }
        }

        ControlRequest queued = mailbox.offerCommand(name, args);
        if (queued == null) {
            exchange.send(HttpResponse.json(409, "{\"error\":\"mailbox_full\"}"));
            return;
        }
        exchange.send(HttpResponse.json(
                202, "{\"requestId\":" + queued.id + ",\"state\":\"QUEUED\"}"));
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
