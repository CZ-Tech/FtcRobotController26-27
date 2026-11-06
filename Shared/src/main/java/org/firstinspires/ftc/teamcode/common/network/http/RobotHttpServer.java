package org.firstinspires.ftc.teamcode.common.network.http;

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
import java.util.logging.Level;
import java.util.logging.Logger;

import javax.net.ssl.SSLContext;

/**
 * Small concurrent HTTP/1.1 server for the robot controller.
 *
 * <p>The acceptor thread does no request work. Every connection is delegated to a
 * worker. Handlers are explicitly forbidden from touching hardware; the network
 * package communicates with the robot control thread through data-only stores/mailboxes.</p>
 */
public final class RobotHttpServer {
    private static final Logger LOGGER = Logger.getLogger(RobotHttpServer.class.getName());

    private final int port;
    private final HttpHandler handler;
    private final ExecutorService workers;
    private final AtomicBoolean running = new AtomicBoolean(false);

    private volatile ServerSocket serverSocket;
    private volatile Thread acceptThread;

    private SSLContext tlsContext;
    private int tlsPort;
    private volatile ServerSocket tlsSocket;
    private volatile Thread tlsAcceptThread;

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

    /**
     * Additionally serves the same handler over TLS on {@code httpsPort}. Must be called before
     * {@link #start()}. The plain HTTP listener is unaffected; if the TLS port cannot be
     * bound, {@link #start()} logs the failure and keeps serving HTTP.
     */
    public synchronized void enableTls(int httpsPort, SSLContext context) {
        if (running.get()) throw new IllegalStateException("server already started");
        if (context == null) throw new IllegalArgumentException("context == null");
        this.tlsPort = httpsPort;
        this.tlsContext = context;
    }

    public synchronized void start() throws IOException {
        if (running.get()) return;
        ServerSocket socket = new ServerSocket(port);
        socket.setReuseAddress(true);
        serverSocket = socket;
        running.set(true);

        acceptThread = startAcceptor(socket, "AzHttp-Acceptor");
        LOGGER.info("Listening on port " + port);

        if (tlsContext != null) {
            try {
                ServerSocket secure = tlsContext.getServerSocketFactory().createServerSocket(tlsPort);
                secure.setReuseAddress(true);
                tlsSocket = secure;
                tlsAcceptThread = startAcceptor(secure, "AzHttps-Acceptor");
                LOGGER.info("Listening (TLS) on port " + secure.getLocalPort());
            } catch (IOException | RuntimeException e) {
                LOGGER.log(Level.SEVERE, "HTTPS listener failed to start on port " + tlsPort
                        + "; continuing with HTTP only", e);
            }
        }
    }

    public synchronized void stop() {
        if (!running.getAndSet(false)) return;
        closeQuietly(serverSocket);
        serverSocket = null;
        closeQuietly(tlsSocket);
        tlsSocket = null;
        workers.shutdownNow();
    }

    public boolean isRunning() {
        return running.get();
    }

    /** Actual HTTPS port once started (useful when 0 was requested), or -1 when TLS is off. */
    public int tlsPort() {
        ServerSocket secure = tlsSocket;
        return secure == null ? -1 : secure.getLocalPort();
    }

    private Thread startAcceptor(ServerSocket socket, String name) {
        Thread thread = new Thread(() -> acceptLoop(socket), name);
        thread.setDaemon(true);
        thread.setPriority(Thread.MIN_PRIORITY);
        thread.start();
        return thread;
    }

    private static void closeQuietly(ServerSocket socket) {
        if (socket == null) return;
        try {
            socket.close();
        } catch (IOException ignored) {}
    }

    private void acceptLoop(ServerSocket listener) {
        while (running.get()) {
            try {
                Socket socket = listener.accept();
                socket.setTcpNoDelay(true);
                socket.setSoTimeout(5000);
                workers.execute(() -> handleConnection(socket));
            } catch (SocketException e) {
                if (running.get()) LOGGER.log(Level.SEVERE, "accept failed", e);
            } catch (Exception e) {
                if (running.get()) LOGGER.log(Level.SEVERE, "accept loop error", e);
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
                LOGGER.log(Level.SEVERE, "request handler failed: " + request.method + " " + request.path, e);
                if (!exchange.isCommitted()) {
                    exchange.send(HttpResponse.json(
                            500, "{\"error\":\"internal_error\"}"));
                }
            }
        } catch (Exception e) {
            LOGGER.fine("connection closed: " + e.getMessage());
        }
    }

    private static void writeFallback(OutputStream out, HttpResponse response) throws IOException {
        byte[] body = response.body;
        String header = "HTTP/1.1 " + response.status + " Bad Request\r\n"
                + "Content-Type: application/json; charset=utf-8\r\n"
                + HttpExchange.BROWSER_ACCESS_HEADERS
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
