package org.firstinspires.ftc.teamcode.common.network;

import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Hardware-free publication point for robot runtime data.
 *
 * <p>Future main-thread code publishes snapshots here. Network workers only read the
 * immutable value and therefore never need a Robot/odometry reference.</p>
 */
public final class RobotRuntimeStore {
    public static final class Snapshot {
        public final long sequence;
        public final long timestampNanos;
        public final boolean opModeActive;
        public final String opModeName;
        public final double x;
        public final double y;
        public final double heading;

        Snapshot(long sequence,
                 long timestampNanos,
                 boolean opModeActive,
                 String opModeName,
                 double x,
                 double y,
                 double heading) {
            this.sequence = sequence;
            this.timestampNanos = timestampNanos;
            this.opModeActive = opModeActive;
            this.opModeName = opModeName;
            this.x = x;
            this.y = y;
            this.heading = heading;
        }

        public String poseJson() {
            return "{\"seq\":" + sequence
                    + ",\"tNanos\":" + timestampNanos
                    + ",\"x\":" + JsonUtil.number(x)
                    + ",\"y\":" + JsonUtil.number(y)
                    + ",\"heading\":" + JsonUtil.number(heading)
                    + "}";
        }

        public String runtimeJson() {
            return "{\"seq\":" + sequence
                    + ",\"opModeActive\":" + opModeActive
                    + ",\"opModeName\":"
                    + (opModeName == null ? "null" : "\"" + JsonUtil.escape(opModeName) + "\"")
                    + "}";
        }
    }

    private final AtomicLong sequence = new AtomicLong();
    private final AtomicReference<Snapshot> latest =
            new AtomicReference<>(new Snapshot(0, 0, false, null, 0, 0, 0));

    /**
     * Future control-thread integration point. Safe to call at the robot loop rate.
     */
    public void publish(boolean opModeActive,
                        String opModeName,
                        double x,
                        double y,
                        double heading) {
        long seq = sequence.incrementAndGet();
        latest.set(new Snapshot(
                seq,
                System.nanoTime(),
                opModeActive,
                opModeName,
                x,
                y,
                heading));
    }

    public Snapshot latest() {
        return latest.get();
    }
}
