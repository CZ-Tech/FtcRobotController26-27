package org.firstinspires.ftc.teamcode.common.subsystem;

import java.util.function.BooleanSupplier;

/**
 * Common fluent operations available on every subsystem schedule proxy.
 *
 * <p>{@link #build()} is optional. Without it, the current draft is atomically
 * snapshotted by the next {@code ScheduledSubsystem.update()} call. Calling
 * {@code build()} publishes the complete draft immediately, reducing submission
 * latency by up to one control-loop tick.</p>
 */
public interface ScheduleApi<S> {
    S waitMillis(long millis);

    S waitUntil(BooleanSupplier condition);

    /**
     * Immediately seals and publishes this schedule.
     *
     * <p>Hardware is still only touched when the subsystem is updated on the
     * OpMode/control thread.</p>
     */
    S build();

    /** Cancels this schedule if it is still the newest schedule for its subsystem. */
    S cancel();

    boolean isCancelled();

    boolean isBuilt();
}
