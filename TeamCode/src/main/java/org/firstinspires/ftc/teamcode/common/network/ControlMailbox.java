package org.firstinspires.ftc.teamcode.common.network;

import java.util.List;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Single-slot handoff from network workers to the future OpMode/main-thread consumer.
 *
 * <p>There is intentionally no background execution facility here. Producers can only
 * enqueue immutable data. The control thread must explicitly {@link #poll()} it.</p>
 */
public final class ControlMailbox {
    private final AtomicReference<ControlRequest> pending = new AtomicReference<>();
    private final AtomicLong nextId = new AtomicLong(1);

    public ControlRequest offerSavedPath(String pathName) {
        return offer(ControlRequest.saved(nextId.getAndIncrement(), pathName));
    }

    public ControlRequest offerInlinePath(String json) {
        return offer(ControlRequest.inline(nextId.getAndIncrement(), json));
    }

    public ControlRequest offerCommand(String name, List<Object> args) {
        return offer(ControlRequest.command(nextId.getAndIncrement(), name, args));
    }

    /** Returns request on success, null when another request is already waiting. */
    private ControlRequest offer(ControlRequest request) {
        return pending.compareAndSet(null, request) ? request : null;
    }

    /** Intended to be called by the robot control thread only. */
    public ControlRequest poll() {
        return pending.getAndSet(null);
    }

    public ControlRequest peek() {
        return pending.get();
    }

    public boolean hasPending() {
        return pending.get() != null;
    }
}
