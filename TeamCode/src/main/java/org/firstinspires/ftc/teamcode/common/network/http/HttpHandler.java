package org.firstinspires.ftc.teamcode.common.network.http;

/** Request handler invoked on a worker thread. It must never touch robot hardware. */
public interface HttpHandler {
    void handle(HttpExchange exchange) throws Exception;
}
