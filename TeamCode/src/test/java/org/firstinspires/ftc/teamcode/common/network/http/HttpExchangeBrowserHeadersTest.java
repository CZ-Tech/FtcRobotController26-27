package org.firstinspires.ftc.teamcode.common.network.http;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.io.ByteArrayOutputStream;
import java.net.Socket;
import java.nio.charset.StandardCharsets;

public class HttpExchangeBrowserHeadersTest {

    @Test
    public void normalResponsesExposeCorsRevisionAndPrivateNetworkHeaders() throws Exception {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        HttpExchange exchange = new HttpExchange(new Socket(), out, null);

        exchange.send(
                HttpResponse.json(200, "{}")
                        .withHeader("ETag", "\"7\"")
                        .withHeader("X-Route-Revision", "7"));

        String response = out.toString(StandardCharsets.UTF_8.name());
        assertBrowserHeaders(response);
        assertTrue(response.contains("ETag: \"7\"\r\n"));
        assertTrue(response.contains("X-Route-Revision: 7\r\n"));
    }

    @Test
    public void eventStreamCarriesSameBrowserAccessPolicy() throws Exception {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        HttpExchange exchange = new HttpExchange(new Socket(), out, null);

        exchange.beginEventStream();

        String response = out.toString(StandardCharsets.UTF_8.name());
        assertTrue(response.contains("Content-Type: text/event-stream; charset=utf-8\r\n"));
        assertBrowserHeaders(response);
    }

    private static void assertBrowserHeaders(String response) {
        assertTrue(response.contains("Access-Control-Allow-Origin: *\r\n"));
        assertTrue(response.contains(
                "Access-Control-Allow-Methods: GET, PUT, POST, DELETE, OPTIONS\r\n"));
        assertTrue(response.contains(
                "Access-Control-Allow-Headers: Content-Type, X-Az-Session, If-Match, Last-Event-ID\r\n"));
        assertTrue(response.contains(
                "Access-Control-Expose-Headers: ETag, X-Route-Revision\r\n"));
        assertTrue(response.contains("Access-Control-Allow-Private-Network: true\r\n"));
        assertTrue(response.contains("Access-Control-Max-Age: 600\r\n"));
        assertTrue(response.contains("Cross-Origin-Resource-Policy: cross-origin\r\n"));
        assertTrue(response.contains(
                "Vary: Origin, Access-Control-Request-Method, Access-Control-Request-Headers, "
                        + "Access-Control-Request-Private-Network\r\n"));
    }
}
