package org.firstinspires.ftc.teamcode.common.subsystem;

import java.util.function.BooleanSupplier;

/**
 * Common fluent operations available on every subsystem schedule proxy.
 *
 * <p>A schedule is only eligible for execution after {@link #build()} seals and
 * publishes it. Forgetting to build leaves the draft inert.</p>
 */
public interface ScheduleApi<S> {
    S waitMillis(long millis);

    S waitUntil(BooleanSupplier condition);

    /** Seals and publishes this schedule to the capacity-1 latest-wins mailbox. */
    S build();

    /** Cancels this schedule if it is still the newest schedule for its subsystem. */
    S cancel();

    boolean isCancelled();

    boolean isBuilt();
}
