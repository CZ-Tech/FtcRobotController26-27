package org.firstinspires.ftc.teamcode.common.subsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * Optional, single-use non-blocking flow. Ordinary subsystem calls need no flow.
 * Build on one thread, then submit with {@link Subsystem#start(ActionSequence)}.
 * Waits delay subsequent steps, never the Java method body of a run callback.
 */
public final class ActionSequence {
    private static final int MAX_STEPS_PER_UPDATE = 32;
    private final List<Step> steps = new ArrayList<>();
    private boolean submitted;
    private int index;
    private boolean stopped;

    private interface Step {
        boolean update(long nowNanos);
    }

    private ActionSequence() {}

    public static ActionSequence create() {
        return new ActionSequence();
    }

    public ActionSequence run(Runnable action) {
        Objects.requireNonNull(action, "action");
        return add(now -> { action.run(); return true; });
    }

    public ActionSequence waitMillis(long millis) {
        if (millis < 0 || millis > Long.MAX_VALUE / 1_000_000L) {
            throw new IllegalArgumentException("millis out of range");
        }
        return add(new Step() {
            private boolean started;
            private long start;
            @Override public boolean update(long now) {
                if (!started) { started = true; start = now; }
                return now - start >= millis * 1_000_000L;
            }
        });
    }

    public ActionSequence waitUntil(BooleanSupplier condition) {
        Objects.requireNonNull(condition, "condition");
        return add(now -> condition.getAsBoolean());
    }

    private ActionSequence add(Step step) {
        if (submitted) throw new IllegalStateException("Sequence already submitted");
        steps.add(step);
        return this;
    }

    void submit() {
        if (submitted) throw new IllegalStateException("Sequence is single-use");
        submitted = true;
    }

    void stop() {
        stopped = true;
    }

    /** Called only by the control loop. True means completed or explicitly stopped. */
    boolean update(long nowNanos, BooleanSupplier keepRunning) {
        int budget = MAX_STEPS_PER_UPDATE;
        try {
            while (!stopped && index < steps.size() && budget-- > 0 && keepRunning.getAsBoolean()) {
                if (!steps.get(index).update(nowNanos)) return stopped;
                index++;
            }
            return stopped || index == steps.size();
        } catch (RuntimeException | Error e) {
            stopped = true; // A failed hardware callback must never be retried implicitly.
            throw e;
        }
    }
}
