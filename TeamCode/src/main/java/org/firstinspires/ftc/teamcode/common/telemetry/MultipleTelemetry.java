package org.firstinspires.ftc.teamcode.common.telemetry;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Forwards every {@link Telemetry} call to several targets, the same idea as FTC
 * Dashboard's {@code MultipleTelemetry}. It contains no policy of its own: each target
 * keeps its own buffer and decides how and when to transmit.
 *
 * <ul>
 *   <li>Getters answer from the <b>first</b> target (the primary, normally the official
 *       Driver Station telemetry); setters apply to all targets.</li>
 *   <li>{@link #update()} updates the targets in order and returns the primary's result.</li>
 *   <li>{@link Func} value producers are evaluated <b>once per {@link #update()}</b> even
 *       when several targets transmit, because evaluating one usually reads hardware.</li>
 *   <li>{@link #addAction(Runnable)} registers on the primary only. The action writes
 *       back through this object, so every target receives what it adds; registering on
 *       every target would run it once per target and duplicate that data.</li>
 * </ul>
 *
 * <p>Like the official telemetry it must be used from the OpMode thread.</p>
 */
public final class MultipleTelemetry implements Telemetry {
    private final List<Telemetry> targets;
    /** Bumped after every update(); lets CachedFunc evaluate once per update cycle. */
    private volatile long generation;

    public MultipleTelemetry(Telemetry primary, Telemetry... others) {
        List<Telemetry> list = new ArrayList<>();
        list.add(requireTarget(primary));
        for (Telemetry other : others) list.add(requireTarget(other));
        targets = Collections.unmodifiableList(list);
    }

    public MultipleTelemetry(List<? extends Telemetry> targets) {
        if (targets == null || targets.isEmpty()) {
            throw new IllegalArgumentException("at least one telemetry target is required");
        }
        List<Telemetry> list = new ArrayList<>();
        for (Telemetry target : targets) list.add(requireTarget(target));
        this.targets = Collections.unmodifiableList(list);
    }

    private static Telemetry requireTarget(Telemetry target) {
        if (target == null) throw new IllegalArgumentException("telemetry target == null");
        return target;
    }

    private Telemetry primary() {
        return targets.get(0);
    }

    // ------------------------------------------------------------------------------------
    // Value producers: evaluated at most once per update cycle
    // ------------------------------------------------------------------------------------

    private final class CachedFunc<T> implements Func<T> {
        private final Func<T> inner;
        private long cachedGeneration = -1;
        private T cached;

        CachedFunc(Func<T> inner) {
            this.inner = inner;
        }

        @Override
        public synchronized T value() {
            long current = generation;
            if (cachedGeneration != current) {
                cached = inner.value();
                cachedGeneration = current;
            }
            return cached;
        }
    }

    private <T> Func<T> cache(Func<T> producer) {
        return producer == null ? null : new CachedFunc<>(producer);
    }

    // ------------------------------------------------------------------------------------
    // Items, lines and log: one handle over the per-target handles
    // ------------------------------------------------------------------------------------

    private final class MultipleItem implements Item {
        private final List<Item> items;

        MultipleItem(List<Item> items) {
            this.items = items;
        }

        @Override
        public String getCaption() {
            return items.get(0).getCaption();
        }

        @Override
        public Item setCaption(String caption) {
            for (Item item : items) item.setCaption(caption);
            return this;
        }

        @Override
        public Item setValue(String format, Object... args) {
            for (Item item : items) item.setValue(format, args);
            return this;
        }

        @Override
        public Item setValue(Object value) {
            for (Item item : items) item.setValue(value);
            return this;
        }

        @Override
        public <T> Item setValue(Func<T> valueProducer) {
            Func<T> shared = cache(valueProducer);
            for (Item item : items) item.setValue(shared);
            return this;
        }

        @Override
        public <T> Item setValue(String format, Func<T> valueProducer) {
            Func<T> shared = cache(valueProducer);
            for (Item item : items) item.setValue(format, shared);
            return this;
        }

        @Override
        public Item setRetained(Boolean retained) {
            for (Item item : items) item.setRetained(retained);
            return this;
        }

        @Override
        public boolean isRetained() {
            return items.get(0).isRetained();
        }

        @Override
        public Item addData(String caption, String format, Object... args) {
            List<Item> added = new ArrayList<>();
            for (Item item : items) added.add(item.addData(caption, format, args));
            return new MultipleItem(added);
        }

        @Override
        public Item addData(String caption, Object value) {
            List<Item> added = new ArrayList<>();
            for (Item item : items) added.add(item.addData(caption, value));
            return new MultipleItem(added);
        }

        @Override
        public <T> Item addData(String caption, Func<T> valueProducer) {
            Func<T> shared = cache(valueProducer);
            List<Item> added = new ArrayList<>();
            for (Item item : items) added.add(item.addData(caption, shared));
            return new MultipleItem(added);
        }

        @Override
        public <T> Item addData(String caption, String format, Func<T> valueProducer) {
            Func<T> shared = cache(valueProducer);
            List<Item> added = new ArrayList<>();
            for (Item item : items) added.add(item.addData(caption, format, shared));
            return new MultipleItem(added);
        }
    }

    private final class MultipleLine implements Line {
        private final List<Line> lines;

        MultipleLine(List<Line> lines) {
            this.lines = lines;
        }

        @Override
        public Item addData(String caption, String format, Object... args) {
            List<Item> added = new ArrayList<>();
            for (Line line : lines) added.add(line.addData(caption, format, args));
            return new MultipleItem(added);
        }

        @Override
        public Item addData(String caption, Object value) {
            List<Item> added = new ArrayList<>();
            for (Line line : lines) added.add(line.addData(caption, value));
            return new MultipleItem(added);
        }

        @Override
        public <T> Item addData(String caption, Func<T> valueProducer) {
            Func<T> shared = cache(valueProducer);
            List<Item> added = new ArrayList<>();
            for (Line line : lines) added.add(line.addData(caption, shared));
            return new MultipleItem(added);
        }

        @Override
        public <T> Item addData(String caption, String format, Func<T> valueProducer) {
            Func<T> shared = cache(valueProducer);
            List<Item> added = new ArrayList<>();
            for (Line line : lines) added.add(line.addData(caption, format, shared));
            return new MultipleItem(added);
        }
    }

    private static final class MultipleLog implements Log {
        private final List<Log> logs;

        MultipleLog(List<Log> logs) {
            this.logs = logs;
        }

        @Override
        public int getCapacity() {
            return logs.get(0).getCapacity();
        }

        @Override
        public void setCapacity(int capacity) {
            for (Log log : logs) log.setCapacity(capacity);
        }

        @Override
        public DisplayOrder getDisplayOrder() {
            return logs.get(0).getDisplayOrder();
        }

        @Override
        public void setDisplayOrder(DisplayOrder displayOrder) {
            for (Log log : logs) log.setDisplayOrder(displayOrder);
        }

        @Override
        public void add(String entry) {
            for (Log log : logs) log.add(entry);
        }

        @Override
        public void add(String format, Object... args) {
            for (Log log : logs) log.add(format, args);
        }

        @Override
        public void clear() {
            for (Log log : logs) log.clear();
        }
    }

    // ------------------------------------------------------------------------------------
    // Telemetry
    // ------------------------------------------------------------------------------------

    @Override
    public Item addData(String caption, String format, Object... args) {
        List<Item> added = new ArrayList<>();
        for (Telemetry target : targets) added.add(target.addData(caption, format, args));
        return new MultipleItem(added);
    }

    @Override
    public Item addData(String caption, Object value) {
        List<Item> added = new ArrayList<>();
        for (Telemetry target : targets) added.add(target.addData(caption, value));
        return new MultipleItem(added);
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        Func<T> shared = cache(valueProducer);
        List<Item> added = new ArrayList<>();
        for (Telemetry target : targets) added.add(target.addData(caption, shared));
        return new MultipleItem(added);
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        Func<T> shared = cache(valueProducer);
        List<Item> added = new ArrayList<>();
        for (Telemetry target : targets) added.add(target.addData(caption, format, shared));
        return new MultipleItem(added);
    }

    @Override
    public boolean removeItem(Item item) {
        if (!(item instanceof MultipleItem)) return false;
        List<Item> items = ((MultipleItem) item).items;
        if (items.size() != targets.size()) return false;
        boolean first = false;
        for (int i = 0; i < targets.size(); i++) {
            boolean removed = targets.get(i).removeItem(items.get(i));
            if (i == 0) first = removed;
        }
        return first;
    }

    @Override
    public void clear() {
        for (Telemetry target : targets) target.clear();
    }

    @Override
    public void clearAll() {
        for (Telemetry target : targets) target.clearAll();
    }

    @Override
    public Object addAction(Runnable action) {
        return primary().addAction(action);
    }

    @Override
    public boolean removeAction(Object token) {
        return primary().removeAction(token);
    }

    @Override
    public void speak(String text) {
        for (Telemetry target : targets) target.speak(text);
    }

    @Override
    public void speak(String text, String languageCode, String countryCode) {
        for (Telemetry target : targets) target.speak(text, languageCode, countryCode);
    }

    @Override
    public boolean update() {
        boolean result = false;
        try {
            for (int i = 0; i < targets.size(); i++) {
                boolean updated = targets.get(i).update();
                if (i == 0) result = updated;
            }
        } finally {
            generation++;
        }
        return result;
    }

    @Override
    public Line addLine() {
        List<Line> added = new ArrayList<>();
        for (Telemetry target : targets) added.add(target.addLine());
        return new MultipleLine(added);
    }

    @Override
    public Line addLine(String lineCaption) {
        List<Line> added = new ArrayList<>();
        for (Telemetry target : targets) added.add(target.addLine(lineCaption));
        return new MultipleLine(added);
    }

    @Override
    public boolean removeLine(Line line) {
        if (!(line instanceof MultipleLine)) return false;
        List<Line> lines = ((MultipleLine) line).lines;
        if (lines.size() != targets.size()) return false;
        boolean first = false;
        for (int i = 0; i < targets.size(); i++) {
            boolean removed = targets.get(i).removeLine(lines.get(i));
            if (i == 0) first = removed;
        }
        return first;
    }

    @Override
    public boolean isAutoClear() {
        return primary().isAutoClear();
    }

    @Override
    public void setAutoClear(boolean autoClear) {
        for (Telemetry target : targets) target.setAutoClear(autoClear);
    }

    @Override
    public int getMsTransmissionInterval() {
        return primary().getMsTransmissionInterval();
    }

    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) {
        for (Telemetry target : targets) target.setMsTransmissionInterval(msTransmissionInterval);
    }

    @Override
    public String getItemSeparator() {
        return primary().getItemSeparator();
    }

    @Override
    public void setItemSeparator(String itemSeparator) {
        for (Telemetry target : targets) target.setItemSeparator(itemSeparator);
    }

    @Override
    public String getCaptionValueSeparator() {
        return primary().getCaptionValueSeparator();
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {
        for (Telemetry target : targets) target.setCaptionValueSeparator(captionValueSeparator);
    }

    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {
        for (Telemetry target : targets) target.setDisplayFormat(displayFormat);
    }

    @Override
    public void setNumDecimalPlaces(int minDecimalPlaces, int maxDecimalPlaces) {
        for (Telemetry target : targets) {
            target.setNumDecimalPlaces(minDecimalPlaces, maxDecimalPlaces);
        }
    }

    @Override
    public Log log() {
        List<Log> logs = new ArrayList<>();
        for (Telemetry target : targets) logs.add(target.log());
        return new MultipleLog(logs);
    }

    @Override
    public String toString() {
        return "MultipleTelemetry" + Arrays.toString(targets.toArray());
    }
}
