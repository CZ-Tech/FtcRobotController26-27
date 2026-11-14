package org.firstinspires.ftc.teamcode.common.telemetry;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.network.TelemetryStore;

import java.util.ArrayList;
import java.util.IllegalFormatException;
import java.util.List;
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * A {@link Telemetry} whose "display" is the network API: {@link #update()} freezes what was
 * added into an immutable {@link TelemetryStore.Frame}, and {@code log()} / {@code speak()}
 * messages go to the store's ordered log. Use it as one target of a
 * {@link MultipleTelemetry} next to the official telemetry.
 *
 * <p>It follows the official semantics so the same OpMode code behaves the same on both
 * outputs: one line per top-level {@code addData}, {@code addLine} groups, auto clear after
 * each update, items with a {@link Func} producer retained by default, producers evaluated
 * only at {@link #update()}.</p>
 *
 * <p>Differences from the Driver Station: every {@link #update()} is published (no
 * {@link #setMsTransmissionInterval(int) transmission interval}; the HTTP side rate-limits
 * itself), display-only settings are stored but unused, and {@code Log.clear()} does not
 * erase the history that API clients already hold.</p>
 *
 * <p>Numeric values keep their type so clients can plot them: a {@link Number} passed to
 * {@code addData}, a single {@link Number} argument of a format, or a numeric producer
 * result. For {@code addData("rpm", "%.0f rpm", x)} the frame carries both the text
 * {@code "3120 rpm"} and the number {@code x} with unit {@code "rpm"}.</p>
 *
 * <p>Thread-safe; normally used from the OpMode thread only.</p>
 */
public final class ApiTelemetry implements Telemetry {
    private static final Pattern NUMBER_PREFIX =
            Pattern.compile("^\\s*[-+]?(?:\\d+\\.?\\d*|\\.\\d+)(?:[eE][-+]?\\d+)?\\s*(.*)$",
                    Pattern.DOTALL);
    private static final int MAX_UNIT_LENGTH = 12;

    private final TelemetryStore store;
    private final Object lock = new Object();
    private final List<LineImpl> lines = new ArrayList<>();
    private final List<ActionToken> actions = new ArrayList<>();

    private boolean autoClear = true;
    private int msTransmissionInterval = 250;
    private String itemSeparator = " | ";
    private String captionValueSeparator = " : ";
    private DisplayFormat displayFormat = DisplayFormat.CLASSIC;
    private int minDecimalPlaces;
    private int maxDecimalPlaces;
    private final LogImpl log = new LogImpl();

    public ApiTelemetry(TelemetryStore store) {
        if (store == null) throw new IllegalArgumentException("store == null");
        this.store = store;
    }

    // ------------------------------------------------------------------------------------
    // Items and lines
    // ------------------------------------------------------------------------------------

    private static final class ActionToken {
        final Runnable action;

        ActionToken(Runnable action) {
            this.action = action;
        }
    }

    private final class LineImpl implements Line {
        final String caption;
        final List<ItemImpl> items = new ArrayList<>();

        LineImpl(String caption) {
            this.caption = caption;
        }

        ItemImpl add(String itemCaption) {
            ItemImpl item = new ItemImpl(this, itemCaption);
            synchronized (lock) {
                items.add(item);
            }
            return item;
        }

        @Override
        public Item addData(String itemCaption, String format, Object... args) {
            return add(itemCaption).setValue(format, args);
        }

        @Override
        public Item addData(String itemCaption, Object value) {
            return add(itemCaption).setValue(value);
        }

        @Override
        public <T> Item addData(String itemCaption, Func<T> valueProducer) {
            return add(itemCaption).setValue(valueProducer);
        }

        @Override
        public <T> Item addData(String itemCaption, String format, Func<T> valueProducer) {
            return add(itemCaption).setValue(format, valueProducer);
        }
    }

    private final class ItemImpl implements Item {
        private final LineImpl line;
        private String caption;
        private Object value;
        private String format;
        private Object[] args;
        private Func<?> producer;
        private Boolean retained;

        ItemImpl(LineImpl line, String caption) {
            this.line = line;
            this.caption = caption;
        }

        @Override
        public String getCaption() {
            synchronized (lock) {
                return caption;
            }
        }

        @Override
        public Item setCaption(String caption) {
            synchronized (lock) {
                this.caption = caption;
            }
            return this;
        }

        @Override
        public ItemImpl setValue(String format, Object... args) {
            synchronized (lock) {
                this.value = null;
                this.format = format;
                this.args = args == null ? new Object[0] : args.clone();
                this.producer = null;
            }
            return this;
        }

        @Override
        public ItemImpl setValue(Object value) {
            synchronized (lock) {
                this.value = value;
                this.format = null;
                this.args = null;
                this.producer = null;
            }
            return this;
        }

        @Override
        public <T> ItemImpl setValue(Func<T> valueProducer) {
            synchronized (lock) {
                this.value = null;
                this.format = null;
                this.args = null;
                this.producer = valueProducer;
            }
            return this;
        }

        @Override
        public <T> ItemImpl setValue(String format, Func<T> valueProducer) {
            synchronized (lock) {
                this.value = null;
                this.format = format;
                this.args = null;
                this.producer = valueProducer;
            }
            return this;
        }

        @Override
        public Item setRetained(Boolean retained) {
            synchronized (lock) {
                this.retained = retained;
            }
            return this;
        }

        @Override
        public boolean isRetained() {
            synchronized (lock) {
                return retained != null ? retained : producer != null;
            }
        }

        @Override
        public Item addData(String caption, String format, Object... args) {
            return line.addData(caption, format, args);
        }

        @Override
        public Item addData(String caption, Object value) {
            return line.addData(caption, value);
        }

        @Override
        public <T> Item addData(String caption, Func<T> valueProducer) {
            return line.addData(caption, valueProducer);
        }

        @Override
        public <T> Item addData(String caption, String format, Func<T> valueProducer) {
            return line.addData(caption, format, valueProducer);
        }

        /** Evaluates the value now. Never throws: a failing producer shows as text. */
        TelemetryStore.Item snapshot() {
            Object current;
            Object[] formatArgs;
            try {
                if (format != null) {
                    current = null;
                    formatArgs = producer != null ? new Object[]{producer.value()} : args;
                } else {
                    current = producer != null ? producer.value() : value;
                    formatArgs = null;
                }
            } catch (RuntimeException e) {
                return new TelemetryStore.Item(
                        caption, "<error: " + e + ">", TelemetryStore.TYPE_TEXT, null, null);
            }

            if (formatArgs != null) {
                String text;
                try {
                    text = String.format(Locale.getDefault(), format, formatArgs);
                } catch (IllegalFormatException e) {
                    text = format;
                }
                if (formatArgs.length == 1 && formatArgs[0] instanceof Number) {
                    return numberItem(caption, text, (Number) formatArgs[0], unitOf(text));
                }
                if (formatArgs.length == 1 && formatArgs[0] instanceof Boolean) {
                    return new TelemetryStore.Item(
                            caption, text, TelemetryStore.TYPE_BOOLEAN, formatArgs[0], null);
                }
                return new TelemetryStore.Item(
                        caption, text, TelemetryStore.TYPE_TEXT, null, null);
            }

            String text = String.valueOf(current);
            if (current instanceof Number) {
                return numberItem(caption, text, (Number) current, null);
            }
            if (current instanceof Boolean) {
                return new TelemetryStore.Item(
                        caption, text, TelemetryStore.TYPE_BOOLEAN, current, null);
            }
            return new TelemetryStore.Item(caption, text, TelemetryStore.TYPE_TEXT, null, null);
        }
    }

    private static TelemetryStore.Item numberItem(
            String caption, String text, Number number, String unit) {
        double d = number.doubleValue();
        Object value = Double.isNaN(d) || Double.isInfinite(d) ? null : (Object) d;
        return new TelemetryStore.Item(caption, text, TelemetryStore.TYPE_NUMBER, value, unit);
    }

    /** The short text after the leading number of a formatted value, or null. */
    static String unitOf(String text) {
        Matcher matcher = NUMBER_PREFIX.matcher(text);
        if (!matcher.matches()) return null;
        String rest = matcher.group(1).trim();
        if (rest.isEmpty() || rest.length() > MAX_UNIT_LENGTH) return null;
        for (int i = 0; i < rest.length(); i++) {
            if (Character.isDigit(rest.charAt(i))) return null;
        }
        return rest;
    }

    private LineImpl newLine(String caption) {
        LineImpl line = new LineImpl(caption);
        synchronized (lock) {
            lines.add(line);
        }
        return line;
    }

    // ------------------------------------------------------------------------------------
    // Telemetry
    // ------------------------------------------------------------------------------------

    @Override
    public Item addData(String caption, String format, Object... args) {
        return newLine(null).addData(caption, format, args);
    }

    @Override
    public Item addData(String caption, Object value) {
        return newLine(null).addData(caption, value);
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        return newLine(null).addData(caption, valueProducer);
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        return newLine(null).addData(caption, format, valueProducer);
    }

    @Override
    public boolean removeItem(Item item) {
        synchronized (lock) {
            for (int i = 0; i < lines.size(); i++) {
                LineImpl line = lines.get(i);
                if (line.items.remove(item)) {
                    if (line.items.isEmpty()) lines.remove(i);
                    return true;
                }
            }
            return false;
        }
    }

    @Override
    public void clear() {
        synchronized (lock) {
            for (int i = lines.size() - 1; i >= 0; i--) {
                LineImpl line = lines.get(i);
                for (int j = line.items.size() - 1; j >= 0; j--) {
                    if (!line.items.get(j).isRetained()) line.items.remove(j);
                }
                if (line.items.isEmpty()) lines.remove(i);
            }
        }
    }

    @Override
    public void clearAll() {
        synchronized (lock) {
            lines.clear();
            actions.clear();
        }
    }

    @Override
    public Object addAction(Runnable action) {
        if (action == null) throw new IllegalArgumentException("action == null");
        ActionToken token = new ActionToken(action);
        synchronized (lock) {
            actions.add(token);
        }
        return token;
    }

    @Override
    public boolean removeAction(Object token) {
        synchronized (lock) {
            return actions.remove(token);
        }
    }

    @Override
    public void speak(String text) {
        store.appendLog(TelemetryStore.KIND_SPEAK, text);
    }

    @Override
    public void speak(String text, String languageCode, String countryCode) {
        store.appendLog(TelemetryStore.KIND_SPEAK, text);
    }

    /** Runs the actions, publishes one frame and, with auto clear, clears it. Always true. */
    @Override
    public boolean update() {
        List<ActionToken> pending;
        synchronized (lock) {
            pending = new ArrayList<>(actions);
        }
        for (ActionToken token : pending) token.action.run();

        synchronized (lock) {
            // Iterate over copies: a value producer must not be able to break the walk by
            // touching this telemetry (the lock is reentrant on the OpMode thread).
            List<TelemetryStore.Line> frame = new ArrayList<>();
            for (LineImpl line : new ArrayList<>(lines)) {
                List<TelemetryStore.Item> items = new ArrayList<>();
                for (ItemImpl item : new ArrayList<>(line.items)) items.add(item.snapshot());
                frame.add(new TelemetryStore.Line(line.caption, items));
            }
            store.publishFrame(frame);
            if (autoClear) clear();
        }
        return true;
    }

    @Override
    public Line addLine() {
        return newLine(null);
    }

    @Override
    public Line addLine(String lineCaption) {
        return newLine(lineCaption);
    }

    @Override
    public boolean removeLine(Line line) {
        synchronized (lock) {
            return lines.remove(line);
        }
    }

    @Override
    public boolean isAutoClear() {
        synchronized (lock) {
            return autoClear;
        }
    }

    @Override
    public void setAutoClear(boolean autoClear) {
        synchronized (lock) {
            this.autoClear = autoClear;
        }
    }

    @Override
    public int getMsTransmissionInterval() {
        synchronized (lock) {
            return msTransmissionInterval;
        }
    }

    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) {
        synchronized (lock) {
            this.msTransmissionInterval = msTransmissionInterval;
        }
    }

    @Override
    public String getItemSeparator() {
        synchronized (lock) {
            return itemSeparator;
        }
    }

    @Override
    public void setItemSeparator(String itemSeparator) {
        synchronized (lock) {
            this.itemSeparator = itemSeparator;
        }
    }

    @Override
    public String getCaptionValueSeparator() {
        synchronized (lock) {
            return captionValueSeparator;
        }
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {
        synchronized (lock) {
            this.captionValueSeparator = captionValueSeparator;
        }
    }

    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {
        synchronized (lock) {
            this.displayFormat = displayFormat;
        }
    }

    @Override
    public void setNumDecimalPlaces(int minDecimalPlaces, int maxDecimalPlaces) {
        synchronized (lock) {
            this.minDecimalPlaces = minDecimalPlaces;
            this.maxDecimalPlaces = maxDecimalPlaces;
        }
    }

    @Override
    public Log log() {
        return log;
    }

    private final class LogImpl implements Log {
        private int capacity = 9;
        private DisplayOrder displayOrder = DisplayOrder.NEWEST_FIRST;

        @Override
        public synchronized int getCapacity() {
            return capacity;
        }

        @Override
        public synchronized void setCapacity(int capacity) {
            this.capacity = capacity;
        }

        @Override
        public synchronized DisplayOrder getDisplayOrder() {
            return displayOrder;
        }

        @Override
        public synchronized void setDisplayOrder(DisplayOrder displayOrder) {
            this.displayOrder = displayOrder;
        }

        @Override
        public void add(String entry) {
            store.appendLog(TelemetryStore.KIND_LOG, entry);
        }

        @Override
        public void add(String format, Object... args) {
            String text;
            try {
                text = String.format(Locale.getDefault(), format, args);
            } catch (IllegalFormatException e) {
                text = format;
            }
            store.appendLog(TelemetryStore.KIND_LOG, text);
        }

        @Override
        public void clear() {
            // API clients keep their own history; nothing to erase.
        }
    }
}
