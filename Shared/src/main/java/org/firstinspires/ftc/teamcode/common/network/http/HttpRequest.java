package org.firstinspires.ftc.teamcode.common.network.http;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.URLDecoder;
import java.nio.charset.StandardCharsets;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

/** Immutable HTTP/1.1 request parsed by {@link RobotHttpServer}. */
public final class HttpRequest {
    private static final int MAX_REQUEST_LINE_BYTES = 8 * 1024;
    private static final int MAX_HEADER_LINE_BYTES = 16 * 1024;
    private static final int MAX_BODY_BYTES = 2 * 1024 * 1024;

    public final String method;
    public final String rawTarget;
    public final String path;
    public final Map<String, String> query;
    public final Map<String, String> headers;
    public final byte[] body;

    private HttpRequest(String method,
                        String rawTarget,
                        String path,
                        Map<String, String> query,
                        Map<String, String> headers,
                        byte[] body) {
        this.method = method;
        this.rawTarget = rawTarget;
        this.path = path;
        this.query = Collections.unmodifiableMap(query);
        this.headers = Collections.unmodifiableMap(headers);
        this.body = body;
    }

    public String bodyUtf8() {
        return new String(body, StandardCharsets.UTF_8);
    }

    public String header(String name) {
        if (name == null) return null;
        return headers.get(name.toLowerCase());
    }

    static HttpRequest read(InputStream in) throws IOException {
        String requestLine = readLine(in, MAX_REQUEST_LINE_BYTES);
        if (requestLine == null || requestLine.trim().isEmpty()) return null;

        String[] requestParts = requestLine.split(" ", 3);
        if (requestParts.length < 2) {
            throw new BadRequestException("Malformed request line");
        }

        String method = requestParts[0].trim().toUpperCase();
        String rawTarget = requestParts[1].trim();

        Map<String, String> headers = new LinkedHashMap<>();
        while (true) {
            String line = readLine(in, MAX_HEADER_LINE_BYTES);
            if (line == null) throw new BadRequestException("Unexpected EOF in headers");
            if (line.isEmpty()) break;
            int colon = line.indexOf(':');
            if (colon <= 0) throw new BadRequestException("Malformed header");
            String name = line.substring(0, colon).trim().toLowerCase();
            String value = line.substring(colon + 1).trim();
            headers.put(name, value);
        }

        int contentLength = 0;
        String contentLengthHeader = headers.get("content-length");
        if (contentLengthHeader != null && !contentLengthHeader.isEmpty()) {
            try {
                contentLength = Integer.parseInt(contentLengthHeader);
            } catch (NumberFormatException e) {
                throw new BadRequestException("Invalid Content-Length");
            }
            if (contentLength < 0 || contentLength > MAX_BODY_BYTES) {
                throw new BadRequestException("Request body too large");
            }
        }

        byte[] body = new byte[contentLength];
        int offset = 0;
        while (offset < contentLength) {
            int n = in.read(body, offset, contentLength - offset);
            if (n < 0) throw new BadRequestException("Unexpected EOF in body");
            offset += n;
        }

        int q = rawTarget.indexOf('?');
        String rawPath = q >= 0 ? rawTarget.substring(0, q) : rawTarget;
        String rawQuery = q >= 0 ? rawTarget.substring(q + 1) : "";
        String path = decode(rawPath);
        Map<String, String> query = parseQuery(rawQuery);

        return new HttpRequest(method, rawTarget, path, query, headers, body);
    }

    private static Map<String, String> parseQuery(String rawQuery) throws BadRequestException {
        Map<String, String> result = new LinkedHashMap<>();
        if (rawQuery == null || rawQuery.isEmpty()) return result;
        String[] pairs = rawQuery.split("&");
        for (String pair : pairs) {
            if (pair.isEmpty()) continue;
            int eq = pair.indexOf('=');
            String key = eq >= 0 ? pair.substring(0, eq) : pair;
            String value = eq >= 0 ? pair.substring(eq + 1) : "";
            result.put(decode(key), decode(value));
        }
        return result;
    }

    private static String decode(String value) throws BadRequestException {
        try {
            return URLDecoder.decode(value, "UTF-8");
        } catch (Exception e) {
            throw new BadRequestException("Invalid URL encoding");
        }
    }

    private static String readLine(InputStream in, int maxBytes) throws IOException {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        int previous = -1;
        while (out.size() <= maxBytes) {
            int current = in.read();
            if (current < 0) {
                if (out.size() == 0) return null;
                break;
            }
            if (previous == '\r' && current == '\n') {
                byte[] bytes = out.toByteArray();
                int len = bytes.length;
                if (len > 0 && bytes[len - 1] == '\r') len--;
                return new String(bytes, 0, len, StandardCharsets.UTF_8);
            }
            out.write(current);
            previous = current;
        }
        if (out.size() > maxBytes) throw new BadRequestException("HTTP line too long");
        return new String(out.toByteArray(), StandardCharsets.UTF_8);
    }

    static final class BadRequestException extends IOException {
        BadRequestException(String message) {
            super(message);
        }
    }
}
