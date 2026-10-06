package org.firstinspires.ftc.teamcode.common.network.http;

import android.util.Log;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Small concurrent HTTP/1.1 server for the robot controller.
 *
 * <p>The acceptor thread does no request work. Every connection is delegated to a
 * worker. Handlers are explicitly forbidden from touching hardware; the network
 * package communicates with the robot control thread through data-only stores/mailboxes.</p>
 */
public final class RobotHttpServer {
    private static final String TAG = "RobotHttpServer";

    private final int port;
    private final HttpHandler handler;
    private final ExecutorService workers;
    private final AtomicBoolean running = new AtomicBoolean(false);

    private volatile ServerSocket serverSocket;
    private volatile Thread acceptThread;

    public RobotHttpServer(int port, HttpHandler handler) {
        this(port, handler, 8);
    }

    public RobotHttpServer(int port, HttpHandler handler, int maxWorkers) {
        if (handler == null) throw new IllegalArgumentException("handler == null");
        if (maxWorkers < 2) throw new IllegalArgumentException("maxWorkers < 2");
        this.port = port;
        this.handler = handler;
        this.workers = Executors.newFixedThreadPool(maxWorkers, new NetworkThreadFactory());
    }

    public synchronized void start() throws IOException {
        if (running.get()) return;
        ServerSocket socket = new ServerSocket(port);
        socket.setReuseAddress(true);
        serverSocket = socket;
        running.set(true);

        acceptThread = new Thread(this::acceptLoop, "AzHttp-Acceptor");
        acceptThread.setDaemon(true);
        acceptThread.setPriority(Thread.MIN_PRIORITY);
        acceptThread.start();
        Log.i(TAG, "Listening on port " + port);
    }

    public synchronized void stop() {
        if (!running.getAndSet(false)) return;
        ServerSocket socket = serverSocket;
        serverSocket = null;
        if (socket != null) {
            try {
                socket.close();
            } catch (IOException ignored) {}
        }
        workers.shutdownNow();
    }

    public boolean isRunning() {
        return running.get();
    }

    private void acceptLoop() {
        while (running.get()) {
            try {
                Socket socket = serverSocket.accept();
                socket.setTcpNoDelay(true);
                socket.setSoTimeout(5000);
                workers.execute(() -> handleConnection(socket));
            } catch (SocketException e) {
                if (running.get()) Log.e(TAG, "accept failed", e);
            } catch (Exception e) {
                if (running.get()) Log.e(TAG, "accept loop error", e);
            }
        }
    }

    private void handleConnection(Socket socket) {
        try (Socket ignored = socket;
             InputStream in = socket.getInputStream();
             OutputStream out = socket.getOutputStream()) {
            HttpRequest request;
            try {
                request = HttpRequest.read(in);
            } catch (HttpRequest.BadRequestException e) {
                writeFallback(out, HttpResponse.json(
                        400, "{\"error\":\"bad_request\",\"message\":\""
                                + escape(e.getMessage()) + "\"}"));
                return;
            }
            if (request == null) return;

            HttpExchange exchange = new HttpExchange(socket, out, request);
            if ("OPTIONS".equals(request.method)) {
                exchange.send(HttpResponse.empty(204));
                return;
            }

            try {
                handler.handle(exchange);
                if (!exchange.isCommitted()) {
                    exchange.send(HttpResponse.json(
                            500, "{\"error\":\"handler_did_not_respond\"}"));
                }
            } catch (Exception e) {
                Log.e(TAG, "request handler failed: " + request.method + " " + request.path, e);
                if (!exchange.isCommitted()) {
                    exchange.send(HttpResponse.json(
                            500, "{\"error\":\"internal_error\"}"));
                }
            }
        } catch (Exception e) {
            Log.d(TAG, "connection closed: " + e.getMessage());
        }
    }

    private static void writeFallback(OutputStream out, HttpResponse response) throws IOException {
        byte[] body = response.body;
        String header = "HTTP/1.1 " + response.status + " Bad Request\r\n"
                + "Content-Type: application/json; charset=utf-8\r\n"
                + "Content-Length: " + body.length + "\r\n"
                + "Connection: close\r\n\r\n";
        out.write(header.getBytes(java.nio.charset.StandardCharsets.UTF_8));
        out.write(body);
        out.flush();
    }

    private static String escape(String value) {
        if (value == null) return "";
        return value.replace("\\", "\\\\")
                .replace("\"", "\\\"")
                .replace("\n", "\\n")
                .replace("\r", "\\r");
    }

    private static final class NetworkThreadFactory implements ThreadFactory {
        private final AtomicInteger nextId = new AtomicInteger(1);

        @Override
        public Thread newThread(Runnable runnable) {
            Thread thread = new Thread(runnable, "AzHttp-Worker-" + nextId.getAndIncrement());
            thread.setDaemon(true);
            thread.setPriority(Thread.MIN_PRIORITY);
            return thread;
        }
    }
}
