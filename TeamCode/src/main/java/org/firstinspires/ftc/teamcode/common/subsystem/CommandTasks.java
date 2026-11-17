package org.firstinspires.ftc.teamcode.common.subsystem;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.CancellationException;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.BooleanSupplier;

/**
 * OpMode-owned command threads. One submission is one sequential Java task; separate
 * submissions may overlap. This class does not arbitrate ownership of hardware.
 * Commands must cooperate with interruption (including polling loops).
 */
public final class CommandTasks {
    private static final AtomicLong NEXT_ID = new AtomicLong();
    private static final ThreadLocal<Task> CURRENT = new ThreadLocal<>();
    private final Set<Task> active = new LinkedHashSet<>();
    private final BooleanSupplier keepRunning;
    private final long shutdownTimeoutMillis;
    private boolean closed;
    private Throwable failure;

    public CommandTasks(BooleanSupplier keepRunning) {
        this(keepRunning, 1000);
    }

    public CommandTasks(BooleanSupplier keepRunning, long shutdownTimeoutMillis) {
        this.keepRunning = Objects.requireNonNull(keepRunning, "keepRunning");
        if (shutdownTimeoutMillis < 0 || shutdownTimeoutMillis > Long.MAX_VALUE / 1_000_000L) {
            throw new IllegalArgumentException("shutdown timeout out of range");
        }
        this.shutdownTimeoutMillis = shutdownTimeoutMillis;
    }

    /** Starts a managed thread, not a control-loop callback. Rejected after stop/seal/failure. */
    public synchronized void execute(Runnable action) {
        Objects.requireNonNull(action, "action");
        if (closed || !keepRunning.getAsBoolean()) {
            throw new IllegalStateException("Command tasks are no longer accepting work");
        }
        Task task = new Task(action);
        active.add(task);
        try {
            task.thread.start();
        } catch (RuntimeException | Error e) {
            active.remove(task);
            throw e;
        }
    }

    /** Prevent new external submissions while already started commands finish normally. */
    public synchronized void seal() { closed = true; }

    public synchronized boolean isIdle() { return active.isEmpty(); }

    /** Rethrow a worker failure on the OpMode thread so its normal cleanup runs. */
    public synchronized void throwIfFailed() {
        if (failure != null) throw new IllegalStateException("Command task failed", failure);
    }

    /** Cancel sibling tasks, retaining the caller so stopAll can finish its own writes. */
    public synchronized void cancelOthers() {
        checkCancelled();
        Task caller = CURRENT.get();
        for (Task task : active) {
            if (task != caller) task.cancel();
        }
    }

    /** True for an interrupted/cancelled managed task, even if it swallowed interruption. */
    public static boolean isCancelled() {
        Task task = CURRENT.get();
        return task != null && (task.cancelled || Thread.currentThread().isInterrupted()
                || !task.owner.keepRunning.getAsBoolean());
    }

    /** Guard built-in hardware entry points against writes from an obsolete task. */
    public static void checkCancelled() {
        if (isCancelled()) throw new CancellationException("Command task cancelled");
    }

    /**
     * Close ingress, interrupt every task, then join outside the monitor before hardware
     * cleanup. Never use Thread.stop(). A non-cooperating task is reported, not hidden.
     */
    public void shutdown() {
        List<Task> stopping;
        synchronized (this) {
            closed = true;
            stopping = new ArrayList<>(active);
            for (Task task : stopping) task.cancel();
        }
        boolean interrupted = Thread.interrupted();
        long started = System.nanoTime();
        long timeoutNanos = shutdownTimeoutMillis * 1_000_000L;
        try {
            for (Task task : stopping) {
                if (task.thread == Thread.currentThread()) continue;
                while (task.thread.isAlive()) {
                    long remaining = timeoutNanos - (System.nanoTime() - started);
                    if (remaining <= 0) break;
                    try {
                        task.thread.join(remaining / 1_000_000L,
                                (int) (remaining % 1_000_000L));
                    } catch (InterruptedException e) {
                        interrupted = true; // Still attempt bounded cleanup of the other tasks.
                    }
                }
            }
            List<String> survivors = new ArrayList<>();
            for (Task task : stopping) {
                if (task.thread != Thread.currentThread() && task.thread.isAlive()) {
                    survivors.add(task.thread.getName());
                }
            }
            if (!survivors.isEmpty()) {
                throw new IllegalStateException("Command tasks ignored cancellation: " + survivors);
            }
        } finally {
            if (interrupted) Thread.currentThread().interrupt();
        }
    }

    private void run(Task task) {
        CURRENT.set(task);
        try {
            checkCancelled();
            task.action.run();
        } catch (CancellationException ignored) {
            // Explicit cancellation or an interrupted command has no executable tail.
        } catch (RuntimeException | Error e) {
            synchronized (this) {
                if (!task.cancelled) {
                    if (failure == null) failure = e;
                    closed = true;
                    for (Task other : active) if (other != task) other.cancel();
                }
            }
        } finally {
            CURRENT.remove();
            synchronized (this) { active.remove(task); }
        }
    }

    private final class Task {
        final CommandTasks owner = CommandTasks.this;
        final Runnable action;
        final Thread thread;
        volatile boolean cancelled;

        Task(Runnable action) {
            this.action = action;
            thread = new Thread(() -> run(this), "RobotCommand-" + NEXT_ID.incrementAndGet());
            thread.setDaemon(true);
        }

        void cancel() {
            cancelled = true;
            thread.interrupt();
        }
    }
}
