package org.firstinspires.ftc.teamcode.common.network;

import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

/** Pure state holder; control code will update it in the later wiring phase. */
public final class ExecutionStateStore {
    public enum State {
        NOT_READY,
        IDLE,
        RUNNING
    }

    public static final class Snapshot {
        public final State state;
        public final long requestId;
        public final String subject;
        public final long revision;

        Snapshot(State state, long requestId, String subject, long revision) {
            this.state = state;
            this.requestId = requestId;
            this.subject = subject;
            this.revision = revision;
        }

        public String toJson() {
            return "{\"state\":\"" + state.name() + "\","
                    + "\"requestId\":" + requestId + ","
                    + "\"subject\":"
                    + (subject == null ? "null" : "\"" + JsonUtil.escape(subject) + "\"") + ","
                    + "\"revision\":" + revision + "}";
        }
    }

    private final AtomicLong revision = new AtomicLong(1);
    private final AtomicReference<Snapshot> current =
            new AtomicReference<>(new Snapshot(State.NOT_READY, 0, null, 1));

    public Snapshot snapshot() {
        return current.get();
    }

    public void publish(State state, long requestId, String subject) {
        long rev = revision.incrementAndGet();
        current.set(new Snapshot(state, requestId, subject, rev));
    }
}
