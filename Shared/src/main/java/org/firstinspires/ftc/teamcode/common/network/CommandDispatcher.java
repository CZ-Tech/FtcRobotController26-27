package org.firstinspires.ftc.teamcode.common.network;

import java.util.concurrent.atomic.AtomicReference;

/**
 * Hand-off point between the HTTP threads and whatever currently owns the robot.
 *
 * <p>An OpMode attaches a {@link Sink} while it is able to run commands and detaches it
 * before shutting down. The sink is called on an HTTP worker thread, so it must only
 * publish data to the control thread (for example a subsystem schedule mailbox) and
 * never touch hardware.</p>
 */
public final class CommandDispatcher {
    public interface Sink {
        /**
         * @param args already validated and converted per the catalog's parameter types
         * @return false when the command can no longer be accepted
         */
        boolean submit(String name, Object[] args);
    }

    private final AtomicReference<Sink> sink = new AtomicReference<>();

    public void attach(Sink value) {
        if (value == null) throw new IllegalArgumentException("sink == null");
        sink.set(value);
    }

    /** Detaches {@code value} only if it is still the attached sink. */
    public void detach(Sink value) {
        sink.compareAndSet(value, null);
    }

    public boolean ready() {
        return sink.get() != null;
    }

    public boolean submit(String name, Object[] args) {
        Sink value = sink.get();
        return value != null && value.submit(name, args);
    }
}
