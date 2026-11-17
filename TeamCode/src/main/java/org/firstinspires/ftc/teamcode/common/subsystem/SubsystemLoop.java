package org.firstinspires.ftc.teamcode.common.subsystem;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Queue;
import java.util.function.LongSupplier;
import java.util.function.BooleanSupplier;

/** Hardware-independent control loop: FIFO ingress, optional flows, lifecycle hooks. */
final class SubsystemLoop {
    private static final int MAX_COMMANDS_PER_UPDATE = 64;
    private final Thread owner = Thread.currentThread();
    private final Queue<Runnable> incoming = new ArrayDeque<>();
    private final List<ActionSequence> flows = new ArrayList<>();
    private final List<SubsystemLifecycle> subsystems = new ArrayList<>();
    private final LongSupplier nanoTime;
    private final BooleanSupplier keepRunning;
    private boolean closed; // guarded by incoming; update/shutdown run on owner

    SubsystemLoop() { this(System::nanoTime); }
    SubsystemLoop(LongSupplier nanoTime) { this(nanoTime, () -> true); }
    SubsystemLoop(LongSupplier nanoTime, BooleanSupplier keepRunning) {
        this.nanoTime = Objects.requireNonNull(nanoTime, "nanoTime");
        this.keepRunning = Objects.requireNonNull(keepRunning, "keepRunning");
    }

    <T extends SubsystemLifecycle> T add(T subsystem) {
        checkThread();
        subsystems.add(Objects.requireNonNull(subsystem, "subsystem"));
        return subsystem;
    }

    /** Thread-safe submission; no hardware or user code runs here. */
    void enqueue(Runnable action) {
        Objects.requireNonNull(action, "action");
        synchronized (incoming) {
            checkOpen();
            incoming.add(action);
        }
    }

    /** Builder is single-threaded; queue publication transfers it to the control thread. */
    void start(ActionSequence sequence) {
        Objects.requireNonNull(sequence, "sequence");
        synchronized (incoming) {
            checkOpen();
            sequence.submit();
            incoming.add(new StartFlow(sequence));
        }
    }

    void update() {
        checkThread();
        for (int i = 0; i < MAX_COMMANDS_PER_UPDATE && keepRunning.getAsBoolean(); i++) {
            Runnable action;
            synchronized (incoming) {
                if (closed) return;
                action = incoming.poll();
            }
            if (action == null) break;
            action.run();
        }
        if (closed || !keepRunning.getAsBoolean()) return;
        long now = nanoTime.getAsLong();
        int index = 0;
        while (index < flows.size() && keepRunning.getAsBoolean()) {
            ActionSequence flow = flows.get(index);
            boolean done = flow.update(now, keepRunning);
            // A callback may explicitly clearFlows()/shutdown(). New flows enter via FIFO.
            if (index < flows.size() && flows.get(index) == flow) {
                if (done) flows.remove(index);
                else index++;
            }
        }
        for (SubsystemLifecycle subsystem : subsystems) {
            if (closed || !keepRunning.getAsBoolean()) break;
            subsystem.periodic();
        }
    }

    /** Explicitly stop active and queued flows, not hardware or ordinary queued calls. */
    void clearFlows() {
        checkThread();
        synchronized (incoming) {
            incoming.removeIf(action -> action instanceof StartFlow);
        }
        for (ActionSequence flow : flows) flow.stop();
        flows.clear();
    }

    /** Identifies queued flow submissions so an explicit clear can discard them. */
    private final class StartFlow implements Runnable {
        private final ActionSequence sequence;
        StartFlow(ActionSequence sequence) { this.sequence = sequence; }
        @Override public void run() { flows.add(sequence); }
    }

    /** Close ingress before stopping all devices, even if one shutdown hook fails. */
    void shutdown() {
        checkThread();
        synchronized (incoming) {
            if (closed) return;
            closed = true;
            incoming.clear();
        }
        clearFlows();
        Throwable failure = null;
        for (SubsystemLifecycle subsystem : subsystems) {
            try {
                subsystem.onShutdown();
            } catch (RuntimeException | Error e) {
                if (failure == null) failure = e;
                else if (failure != e) failure.addSuppressed(e);
            }
        }
        if (failure instanceof RuntimeException) throw (RuntimeException) failure;
        if (failure instanceof Error) throw (Error) failure;
    }

    private void checkOpen() {
        if (closed) throw new IllegalStateException("Subsystems are shut down");
    }

    private void checkThread() {
        if (Thread.currentThread() != owner) {
            throw new IllegalStateException("Must run on the OpMode/control thread");
        }
    }
}
