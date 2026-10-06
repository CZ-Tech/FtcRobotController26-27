package org.firstinspires.ftc.teamcode.common.network.http;

import java.io.IOException;
import java.io.OutputStream;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.Locale;
import java.util.Map;

/**
 * One accepted HTTP connection. A handler must either call {@link #send(HttpResponse)}
 * or {@link #beginEventStream()} exactly once.
 */
public final class HttpExchange {
    public static final String BROWSER_ACCESS_HEADERS =
            "Access-Control-Allow-Origin: *\r\n"
            + "Access-Control-Allow-Methods: GET, PUT, POST, DELETE, OPTIONS\r\n"
            + "Access-Control-Allow-Headers: Content-Type, X-Az-Session, If-Match, Last-Event-ID\r\n"
            + "Access-Control-Expose-Headers: ETag, X-Route-Revision\r\n"
            + "Access-Control-Allow-Private-Network: true\r\n"
            + "Access-Control-Max-Age: 600\r\n"
            + "Cross-Origin-Resource-Policy: cross-origin\r\n"
            + "Vary: Origin, Access-Control-Request-Method, Access-Control-Request-Headers, "
            + "Access-Control-Request-Private-Network\r\n";

    private final Socket socket;
    private final OutputStream out;
    private boolean committed;
    public final HttpRequest request;

    HttpExchange(Socket socket, OutputStream out, HttpRequest request) {
        this.socket = socket;
        this.out = out;
        this.request = request;
    }

    public synchronized boolean isCommitted() {
        return committed;
    }

    public synchronized void send(HttpResponse response) throws IOException {
        if (committed) throw new IllegalStateException("response already committed");
        committed = true;

        StringBuilder header = new StringBuilder();
        header.append("HTTP/1.1 ")
                .append(response.status)
                .append(' ')
                .append(reason(response.status))
                .append("\r\n");
        if (response.contentType != null) {
            header.append("Content-Type: ").append(response.contentType).append("\r\n");
        }
        for (Map.Entry<String, String> entry : response.headers.entrySet()) {
            header.append(entry.getKey()).append(": ").append(entry.getValue()).append("\r\n");
        }
        header.append(BROWSER_ACCESS_HEADERS)
                .append("Cache-Control: no-store\r\n")
                .append("Content-Length: ").append(response.body.length).append("\r\n")
                .append("Connection: close\r\n\r\n");
        out.write(header.toString().getBytes(StandardCharsets.UTF_8));
        out.write(response.body);
        out.flush();
    }

    /**
     * Commit an SSE response and return the raw stream. The handler owns the connection
     * until it returns; {@link RobotHttpServer} closes the socket afterwards.
     */
    public synchronized OutputStream beginEventStream() throws IOException {
        if (committed) throw new IllegalStateException("response already committed");
        committed = true;
        String header =
                "HTTP/1.1 200 OK\r\n"
                + "Content-Type: text/event-stream; charset=utf-8\r\n"
                + "Cache-Control: no-cache, no-transform\r\n"
                + "Connection: keep-alive\r\n"
                + BROWSER_ACCESS_HEADERS
                + "X-Accel-Buffering: no\r\n"
                + "\r\n";
        out.write(header.getBytes(StandardCharsets.UTF_8));
        out.flush();
        return out;
    }

    public String remoteAddress() {
        return socket.getRemoteSocketAddress() == null
                ? "unknown"
                : socket.getRemoteSocketAddress().toString();
    }

    private static String reason(int status) {
        switch (status) {
            case 200: return "OK";
            case 201: return "Created";
            case 202: return "Accepted";
            case 204: return "No Content";
            case 400: return "Bad Request";
            case 401: return "Unauthorized";
            case 404: return "Not Found";
            case 409: return "Conflict";
            case 412: return "Precondition Failed";
            case 428: return "Precondition Required";
            case 429: return "Too Many Requests";
            case 500: return "Internal Server Error";
            case 503: return "Service Unavailable";
            default: return String.format(Locale.US, "Status %d", status);
        }
    }
}
