package org.firstinspires.ftc.teamcode.common.network;

import java.security.SecureRandom;

/**
 * Enforces a single controlling computer per robot.
 *
 * <p>A client opens a lease, receives a random token, and supplies that token on all
 * later API requests. A second client receives a conflict until the first lease is
 * explicitly closed or expires. The event-stream worker continuously touches the lease,
 * so a healthy long connection keeps ownership alive without polling.</p>
 */
public final class SessionLease {
    public static final long DEFAULT_TIMEOUT_MS = 10_000;

    public static final class OpenResult {
        public final boolean acquired;
        public final String token;
        public final long expiresInMs;

        private OpenResult(boolean acquired, String token, long expiresInMs) {
            this.acquired = acquired;
            this.token = token;
            this.expiresInMs = expiresInMs;
        }
    }

    private final long timeoutMs;
    private final SecureRandom random = new SecureRandom();

    private String token;
    private String owner;
    private long lastTouchMs;

    public SessionLease() {
        this(DEFAULT_TIMEOUT_MS);
    }

    public SessionLease(long timeoutMs) {
        if (timeoutMs < 1000) throw new IllegalArgumentException("timeoutMs < 1000");
        this.timeoutMs = timeoutMs;
    }

    public synchronized OpenResult open(String requestedOwner) {
        long now = now();
        expireIfNeeded(now);
        if (token != null) {
            return new OpenResult(false, null, remaining(now));
        }
        token = newToken();
        owner = requestedOwner == null ? "unknown" : requestedOwner;
        lastTouchMs = now;
        return new OpenResult(true, token, timeoutMs);
    }

    public synchronized boolean validate(String candidate) {
        long now = now();
        expireIfNeeded(now);
        return token != null && token.equals(candidate);
    }

    public synchronized boolean touch(String candidate) {
        long now = now();
        expireIfNeeded(now);
        if (token == null || !token.equals(candidate)) return false;
        lastTouchMs = now;
        return true;
    }

    public synchronized boolean close(String candidate) {
        if (token == null || !token.equals(candidate)) return false;
        clear();
        return true;
    }

    /** Local administrative revoke. Not exposed as a network endpoint. */
    public synchronized void revoke() {
        clear();
    }

    public synchronized boolean isOwned() {
        expireIfNeeded(now());
        return token != null;
    }

    public synchronized String owner() {
        expireIfNeeded(now());
        return owner;
    }

    public synchronized long expiresInMs() {
        long now = now();
        expireIfNeeded(now);
        return remaining(now);
    }

    private long remaining(long now) {
        if (token == null) return 0;
        return Math.max(0, timeoutMs - (now - lastTouchMs));
    }

    private void expireIfNeeded(long now) {
        if (token != null && now - lastTouchMs > timeoutMs) clear();
    }

    private void clear() {
        token = null;
        owner = null;
        lastTouchMs = 0;
    }

    private String newToken() {
        byte[] bytes = new byte[24];
        random.nextBytes(bytes);
        StringBuilder sb = new StringBuilder(bytes.length * 2);
        for (byte b : bytes) sb.append(String.format("%02x", b & 0xff));
        return sb.toString();
    }

    private static long now() {
        return System.nanoTime() / 1_000_000L;
    }
}
