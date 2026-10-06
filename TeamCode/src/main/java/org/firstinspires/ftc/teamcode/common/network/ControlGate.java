package org.firstinspires.ftc.teamcode.common.network;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

/** OpMode lifecycle gate for hardware-affecting network requests. */
public final class ControlGate {
    private final AtomicBoolean active = new AtomicBoolean(false);
    private final AtomicLong generation = new AtomicLong(0);
    private final ControlMailbox mailbox;

    public ControlGate(ControlMailbox mailbox) {
        this.mailbox = mailbox;
    }

    public synchronized long activate() {
        mailbox.clear();
        long next = generation.incrementAndGet();
        active.set(true);
        return next;
    }

    public synchronized void deactivate() {
        active.set(false);
        generation.incrementAndGet();
        mailbox.clear();
    }

    public boolean isActive() {
        return active.get();
    }

    public long generation() {
        return generation.get();
    }

    public synchronized ControlRequest submitSavedPath(String pathName) {
        if (!active.get()) return null;
        return mailbox.offerSavedPath(pathName);
    }

    public synchronized ControlRequest submitInlinePath(String json) {
        if (!active.get()) return null;
        return mailbox.offerInlinePath(json);
    }

    /** Main-thread consumer. Returns only the latest unconsumed operation. */
    public synchronized ControlRequest pollLatest() {
        if (!active.get()) {
            mailbox.clear();
            return null;
        }
        return mailbox.poll();
    }
}
