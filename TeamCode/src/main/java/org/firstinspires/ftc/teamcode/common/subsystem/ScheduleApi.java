package org.firstinspires.ftc.teamcode.common.subsystem;

import java.util.function.BooleanSupplier;

/**
 * Common fluent operations available on every subsystem schedule proxy.
 *
 * <p>A draft is completely independent until {@link #execute()} seals and publishes it.
 * Drafts may be stored, passed around and executed in any order.</p>
 */
public interface ScheduleApi<S> {
    S waitMillis(long millis);

    S waitUntil(BooleanSupplier condition);

    /** Seals and publishes this schedule to the capacity-1 latest-execute-wins mailbox. */
    S execute();

    /** Cancels this schedule if it is still the newest schedule for its subsystem. */
    S cancel();

    boolean isCancelled();

    boolean isExecuted();
}
