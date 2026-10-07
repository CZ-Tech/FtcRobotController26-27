package org.firstinspires.ftc.teamcode.common.opmode;

import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Thread-safe, hardware-free facade for FTC OpMode lifecycle control.
 *
 * <p>The network stack talks only to this class. The FTC SDK bridge is attached
 * separately and is the sole source of lifecycle state.</p>
 */
public final class OpModeLifecycleService {
    public enum Phase {
        STOPPED,
        INIT,
        RUNNING
    }

    public static final class Snapshot {
        public final long revision;
        public final boolean controllerAvailable;
        public final Phase phase;
        public final String activeName;

        Snapshot(long revision,
                 boolean controllerAvailable,
                 Phase phase,
                 String activeName) {
            this.revision = revision;
            this.controllerAvailable = controllerAvailable;
            this.phase = phase;
            this.activeName = activeName;
        }
    }

    public static final class Descriptor {
        public final String name;
        public final String group;

        public Descriptor(String name, String group) {
            this.name = name;
            this.group = group;
        }
    }

    public static final class BackendResult {
        public final boolean accepted;
        public final String code;
        public final String message;

        private BackendResult(boolean accepted, String code, String message) {
            this.accepted = accepted;
            this.code = code;
            this.message = message;
        }

        public static BackendResult accepted() {
            return new BackendResult(true, null, null);
        }

        public static BackendResult rejected(String code, String message) {
            return new BackendResult(false, code, message);
        }
    }

    public static final class ActionResult {
        public final boolean accepted;
        public final String code;
        public final String message;
        public final Snapshot snapshot;

        ActionResult(boolean accepted, String code, String message, Snapshot snapshot) {
            this.accepted = accepted;
            this.code = code;
            this.message = message;
            this.snapshot = snapshot;
        }
    }

    public interface Backend {
        List<Descriptor> listAutonomous();

        BackendResult init(String name, Snapshot snapshot);

        BackendResult start(String name, Snapshot snapshot);

        BackendResult stop(String name, Snapshot snapshot);
    }

    private final AtomicReference<Backend> backend = new AtomicReference<>();
    private final AtomicReference<Snapshot> current =
            new AtomicReference<>(new Snapshot(1, false, Phase.STOPPED, null));

    public Snapshot snapshot() {
        return current.get();
    }

    public synchronized void attach(Backend value) {
        if (value == null) throw new IllegalArgumentException("backend == null");
        backend.set(value);
        publish(true, current.get().phase, current.get().activeName);
    }

    public synchronized void detach(Backend value) {
        backend.compareAndSet(value, null);
        if (backend.get() == null) {
            publish(false, Phase.STOPPED, null);
        }
    }

    public synchronized void publishInit(String name) {
        publish(true, Phase.INIT, name);
    }

    public synchronized void publishRunning(String name) {
        publish(true, Phase.RUNNING, name);
    }

    public synchronized void publishStopped() {
        publish(true, Phase.STOPPED, null);
    }

    public List<Descriptor> listAutonomous() {
        Backend value = backend.get();
        return value == null
                ? Collections.emptyList()
                : value.listAutonomous();
    }

    public boolean controllerAvailable() {
        return backend.get() != null;
    }

    public ActionResult init(String name, long expectedRevision) {
        return invoke(name, expectedRevision, Action.INIT);
    }

    public ActionResult start(String name, long expectedRevision) {
        return invoke(name, expectedRevision, Action.START);
    }

    public ActionResult stop(String name, long expectedRevision) {
        return invoke(name, expectedRevision, Action.STOP);
    }

    private enum Action { INIT, START, STOP }

    private synchronized ActionResult invoke(String name, long expectedRevision, Action action) {
        Snapshot observed = current.get();
        if (expectedRevision != observed.revision) {
            return new ActionResult(
                    false,
                    "state_changed",
                    "Robot OpMode state changed before the request was applied",
                    observed);
        }

        Backend value = backend.get();
        if (value == null) {
            return new ActionResult(
                    false,
                    "controller_unavailable",
                    "FTC OpMode controller is unavailable",
                    observed);
        }

        BackendResult result;
        switch (action) {
            case INIT:
                result = value.init(name, observed);
                break;
            case START:
                result = value.start(name, observed);
                break;
            case STOP:
                result = value.stop(name, observed);
                break;
            default:
                throw new AssertionError(action);
        }
        return new ActionResult(
                result.accepted,
                result.code,
                result.message,
                current.get());
    }

    private void publish(boolean available, Phase phase, String name) {
        Snapshot old = current.get();
        if (old.controllerAvailable == available
                && old.phase == phase
                && same(old.activeName, name)) {
            return;
        }
        current.set(new Snapshot(
                old.revision + 1,
                available,
                phase,
                name));
    }

    private static boolean same(String a, String b) {
        return a == null ? b == null : a.equals(b);
    }
}
