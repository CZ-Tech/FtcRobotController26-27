package org.firstinspires.ftc.teamcode.common.network;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Hardware-free publication point for telemetry, in the same style as
 * {@link RobotRuntimeStore}.
 *
 * <p>The OpMode thread publishes one immutable {@link Frame} per {@code telemetry.update()}
 * and appends {@link LogEntry} messages. Network workers only read immutable values, so no
 * robot or hardware object is reachable from here. Frames are latest-value data: a slow
 * reader skips old frames. Log entries are an ordered, bounded history identified by a
 * strictly increasing {@code seq}, so a reader can detect and resume after a gap.</p>
 */
public final class TelemetryStore {
    public static final int DEFAULT_LOG_CAPACITY = 200;

    public static final String TYPE_NUMBER = "number";
    public static final String TYPE_BOOLEAN = "boolean";
    public static final String TYPE_TEXT = "text";

    public static final String KIND_LOG = "log";
    public static final String KIND_SPEAK = "speak";

    /** One caption/value pair. {@code value} is a Double, Boolean or null (text). */
    public static final class Item {
        public final String caption;
        /** The text the Driver Station shows. */
        public final String text;
        public final String type;
        /** Double for numbers (null when NaN/infinite), Boolean for booleans, else null. */
        public final Object value;
        /** Unit parsed from a formatted number such as "%.1f in/s"; null when unknown. */
        public final String unit;

        public Item(String caption, String text, String type, Object value, String unit) {
            this.caption = caption == null ? "" : caption;
            this.text = text == null ? "" : text;
            this.type = type == null ? TYPE_TEXT : type;
            this.value = value;
            this.unit = unit;
        }

        void appendJson(StringBuilder out) {
            out.append("{\"caption\":\"").append(JsonUtil.escape(caption))
                    .append("\",\"text\":\"").append(JsonUtil.escape(text))
                    .append("\",\"type\":\"").append(type).append("\",\"value\":");
            if (value instanceof Double) {
                out.append(jsonNumber((Double) value));
            } else if (value instanceof Boolean) {
                out.append(((Boolean) value) ? "true" : "false");
            } else {
                out.append("null");
            }
            if (unit != null) {
                out.append(",\"unit\":\"").append(JsonUtil.escape(unit)).append('"');
            }
            out.append('}');
        }
    }

    /** A telemetry line: an optional caption and the items shown on it. */
    public static final class Line {
        public final String caption;
        public final List<Item> items;

        public Line(String caption, List<Item> items) {
            this.caption = caption;
            this.items = Collections.unmodifiableList(new ArrayList<>(items));
        }

        void appendJson(StringBuilder out) {
            out.append("{\"caption\":");
            if (caption == null) {
                out.append("null");
            } else {
                out.append('"').append(JsonUtil.escape(caption)).append('"');
            }
            out.append(",\"items\":[");
            for (int i = 0; i < items.size(); i++) {
                if (i > 0) out.append(',');
                items.get(i).appendJson(out);
            }
            out.append("]}");
        }
    }

    /** Everything shown by one {@code telemetry.update()}. */
    public static final class Frame {
        public final long sequence;
        public final long timestampNanos;
        public final List<Line> lines;

        Frame(long sequence, long timestampNanos, List<Line> lines) {
            this.sequence = sequence;
            this.timestampNanos = timestampNanos;
            this.lines = lines;
        }

        public String toJson() {
            StringBuilder out = new StringBuilder("{\"seq\":").append(sequence)
                    .append(",\"tNanos\":").append(timestampNanos)
                    .append(",\"lines\":[");
            for (int i = 0; i < lines.size(); i++) {
                if (i > 0) out.append(',');
                lines.get(i).appendJson(out);
            }
            return out.append("]}").toString();
        }
    }

    /** One message from {@code telemetry.log()} or {@code telemetry.speak()}. */
    public static final class LogEntry {
        public final long sequence;
        public final long timestampNanos;
        public final String kind;
        public final String text;

        LogEntry(long sequence, long timestampNanos, String kind, String text) {
            this.sequence = sequence;
            this.timestampNanos = timestampNanos;
            this.kind = kind;
            this.text = text;
        }

        public String toJson() {
            return "{\"seq\":" + sequence
                    + ",\"tNanos\":" + timestampNanos
                    + ",\"kind\":\"" + JsonUtil.escape(kind) + "\""
                    + ",\"text\":\"" + JsonUtil.escape(text) + "\"}";
        }
    }

    private final AtomicLong frameSequence = new AtomicLong();
    private final AtomicReference<Frame> latest =
            new AtomicReference<>(new Frame(0, 0, Collections.<Line>emptyList()));

    private final int logCapacity;
    private final Object logLock = new Object();
    private final ArrayList<LogEntry> log = new ArrayList<>();
    private long logSequence;

    public TelemetryStore() {
        this(DEFAULT_LOG_CAPACITY);
    }

    public TelemetryStore(int logCapacity) {
        if (logCapacity < 1) throw new IllegalArgumentException("logCapacity < 1");
        this.logCapacity = logCapacity;
    }

    /** Control-thread entry point; safe to call at the robot loop rate. */
    public void publishFrame(List<Line> lines) {
        latest.set(new Frame(
                frameSequence.incrementAndGet(),
                System.nanoTime(),
                Collections.unmodifiableList(new ArrayList<>(lines))));
    }

    public Frame latest() {
        return latest.get();
    }

    public void appendLog(String kind, String text) {
        synchronized (logLock) {
            log.add(new LogEntry(
                    ++logSequence, System.nanoTime(), kind, text == null ? "" : text));
            if (log.size() > logCapacity) log.remove(0);
        }
    }

    /** Sequence of the newest log entry, 0 when none was ever written. */
    public long latestLogSequence() {
        synchronized (logLock) {
            return logSequence;
        }
    }

    /**
     * Entries with {@code seq > afterSequence}, oldest first. If the history already dropped
     * some of them the first returned {@code seq} is larger than {@code afterSequence + 1}.
     */
    public List<LogEntry> logSince(long afterSequence) {
        synchronized (logLock) {
            List<LogEntry> out = new ArrayList<>();
            for (LogEntry entry : log) {
                if (entry.sequence > afterSequence) out.add(entry);
            }
            return out;
        }
    }

    public String logJson(long afterSequence) {
        List<LogEntry> entries;
        long newest;
        synchronized (logLock) {
            entries = logSince(afterSequence);
            newest = logSequence;
        }
        StringBuilder out = new StringBuilder("{\"latestSeq\":").append(newest)
                .append(",\"entries\":[");
        for (int i = 0; i < entries.size(); i++) {
            if (i > 0) out.append(',');
            out.append(entries.get(i).toJson());
        }
        return out.append("]}").toString();
    }

    /** Full-precision JSON number; NaN and infinities become {@code null}. */
    static String jsonNumber(double value) {
        if (Double.isNaN(value) || Double.isInfinite(value)) return "null";
        if (value == Math.rint(value) && Math.abs(value) < 1e15) {
            return Long.toString((long) value);
        }
        return Double.toString(value);
    }
}
