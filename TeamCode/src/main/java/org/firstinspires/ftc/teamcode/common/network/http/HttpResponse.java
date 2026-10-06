package org.firstinspires.ftc.teamcode.common.network.http;

import java.nio.charset.StandardCharsets;
import java.util.LinkedHashMap;
import java.util.Map;

/** Immutable response description for ordinary non-streaming HTTP requests. */
public final class HttpResponse {
    public final int status;
    public final String contentType;
    public final byte[] body;
    public final Map<String, String> headers;

    private HttpResponse(int status,
                         String contentType,
                         byte[] body,
                         Map<String, String> headers) {
        this.status = status;
        this.contentType = contentType;
        this.body = body;
        this.headers = headers;
    }

    public static HttpResponse json(int status, String json) {
        return text(status, "application/json; charset=utf-8", json);
    }

    public static HttpResponse text(int status, String contentType, String text) {
        return new HttpResponse(
                status,
                contentType,
                text == null ? new byte[0] : text.getBytes(StandardCharsets.UTF_8),
                new LinkedHashMap<>());
    }

    public static HttpResponse empty(int status) {
        return new HttpResponse(status, null, new byte[0], new LinkedHashMap<>());
    }

    public HttpResponse withHeader(String name, String value) {
        headers.put(name, value);
        return this;
    }
}
