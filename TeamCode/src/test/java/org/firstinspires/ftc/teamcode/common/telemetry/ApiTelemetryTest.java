package org.firstinspires.ftc.teamcode.common.telemetry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.network.TelemetryStore;
import org.json.JSONArray;
import org.json.JSONObject;
import org.junit.Before;
import org.junit.Test;

import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

public class ApiTelemetryTest {
    private TelemetryStore store;
    private ApiTelemetry telemetry;

    @Before
    public void setUp() {
        store = new TelemetryStore();
        telemetry = new ApiTelemetry(store);
    }

    private TelemetryStore.Item onlyItem(int line) {
        return store.latest().lines.get(line).items.get(0);
    }

    @Test
    public void updatePublishesOneFramePerCallWithIncreasingSequence() {
        assertEquals(0, store.latest().sequence);
        telemetry.addData("a", 1);
        telemetry.update();
        long first = store.latest().sequence;
        telemetry.update();

        assertEquals(1, first);
        assertEquals(2, store.latest().sequence);
    }

    @Test
    public void plainValuesKeepTheirType() {
        telemetry.addData("n", 3);
        telemetry.addData("d", 2.5);
        telemetry.addData("b", true);
        telemetry.addData("s", "hello");
        telemetry.update();

        assertEquals(4, store.latest().lines.size());
        assertEquals(TelemetryStore.TYPE_NUMBER, onlyItem(0).type);
        assertEquals(3.0, (Double) onlyItem(0).value, 0);
        assertEquals("3", onlyItem(0).text);
        assertEquals(2.5, (Double) onlyItem(1).value, 0);
        assertEquals(TelemetryStore.TYPE_BOOLEAN, onlyItem(2).type);
        assertEquals(Boolean.TRUE, onlyItem(2).value);
        assertEquals(TelemetryStore.TYPE_TEXT, onlyItem(3).type);
        assertNull(onlyItem(3).value);
        assertEquals("hello", onlyItem(3).text);
    }

    @Test
    public void formatStringKeepsTheRawNumberAndParsesTheUnit() {
        telemetry.addData("rpm", "%.0f rpm", 3120.4);
        telemetry.addData("heading", "%.1f°", 90.04);
        telemetry.addData("speed", "%.1f in/s", 12.34);
        telemetry.addData("plain", "%.2f", 0.5);
        telemetry.addData("pair", "%.1f / %.1f", 1.0, 2.0);
        telemetry.update();

        TelemetryStore.Item rpm = onlyItem(0);
        assertEquals("3120 rpm", rpm.text);
        assertEquals(TelemetryStore.TYPE_NUMBER, rpm.type);
        assertEquals(3120.4, (Double) rpm.value, 0);
        assertEquals("rpm", rpm.unit);

        assertEquals("°", onlyItem(1).unit);
        assertEquals(90.04, (Double) onlyItem(1).value, 0);
        assertEquals("in/s", onlyItem(2).unit);

        assertNull(onlyItem(3).unit);
        assertEquals(0.5, (Double) onlyItem(3).value, 0);

        // More than one argument cannot be plotted as one number.
        assertEquals(TelemetryStore.TYPE_TEXT, onlyItem(4).type);
        assertNull(onlyItem(4).value);
        assertEquals("1.0 / 2.0", onlyItem(4).text);
    }

    @Test
    public void nonFiniteNumbersAreNumbersWithoutAValue() {
        telemetry.addData("nan", Double.NaN);
        telemetry.update();

        assertEquals(TelemetryStore.TYPE_NUMBER, onlyItem(0).type);
        assertNull(onlyItem(0).value);
        assertTrue(store.latest().toJson().contains("\"value\":null"));
    }

    @Test
    public void autoClearEmptiesTheNextFrameButRetainedItemsStay() {
        telemetry.addData("gone", 1);
        telemetry.addData("kept", 2).setRetained(true);
        telemetry.update();
        assertEquals(2, store.latest().lines.size());

        telemetry.update();
        assertEquals(1, store.latest().lines.size());
        assertEquals("kept", onlyItem(0).caption);
    }

    @Test
    public void disablingAutoClearKeepsEverything() {
        telemetry.setAutoClear(false);
        telemetry.addData("a", 1);
        telemetry.update();
        telemetry.update();

        assertEquals(1, store.latest().lines.size());
        assertFalse(telemetry.isAutoClear());
    }

    @Test
    public void producersAreEvaluatedOnlyAtUpdateAndAreRetained() {
        AtomicInteger calls = new AtomicInteger();
        Func<Integer> producer = calls::incrementAndGet;
        telemetry.addData("count", producer);
        assertEquals(0, calls.get());

        telemetry.update();
        assertEquals(1, calls.get());
        assertEquals(1.0, (Double) onlyItem(0).value, 0);

        // Retained by default, so it is re-evaluated by the next frame without re-adding.
        telemetry.update();
        assertEquals(2, calls.get());
        assertEquals(2.0, (Double) onlyItem(0).value, 0);
    }

    @Test
    public void producerWithFormatKeepsNumberAndUnit() {
        telemetry.addData("v", "%.1f V", (Func<Double>) () -> 12.34);
        telemetry.update();

        assertEquals("12.3 V", onlyItem(0).text);
        assertEquals(12.34, (Double) onlyItem(0).value, 0);
        assertEquals("V", onlyItem(0).unit);
    }

    @Test
    public void failingProducerDoesNotBreakTheFrame() {
        telemetry.addData("bad", (Func<Integer>) () -> {
            throw new IllegalStateException("boom");
        });
        telemetry.addData("ok", 1);
        telemetry.update();

        assertEquals(2, store.latest().lines.size());
        assertTrue(onlyItem(0).text.contains("boom"));
    }

    @Test
    public void itemSetValueChangesTheValueBeforeUpdate() {
        Telemetry.Item item = telemetry.addData("x", 1);
        item.setValue("%.1f m", 2.0).setCaption("y");
        telemetry.update();

        assertEquals("y", onlyItem(0).caption);
        assertEquals("2.0 m", onlyItem(0).text);
        assertEquals("m", onlyItem(0).unit);
    }

    @Test
    public void linesGroupItemsAndKeepCaptions() {
        Telemetry.Line line = telemetry.addLine("Drive");
        line.addData("x", 1).addData("y", 2);
        telemetry.addLine();
        telemetry.update();

        TelemetryStore.Line first = store.latest().lines.get(0);
        assertEquals("Drive", first.caption);
        assertEquals(2, first.items.size());
        assertEquals("y", first.items.get(1).caption);
        assertNull(store.latest().lines.get(1).caption);
        assertEquals(0, store.latest().lines.get(1).items.size());
    }

    @Test
    public void removeItemRemovesItsLine() {
        Telemetry.Item item = telemetry.addData("x", 1);
        telemetry.addData("y", 2);

        assertTrue(telemetry.removeItem(item));
        assertFalse(telemetry.removeItem(item));
        telemetry.update();

        assertEquals(1, store.latest().lines.size());
        assertEquals("y", onlyItem(0).caption);
    }

    @Test
    public void actionsRunBeforeEachFrameAndCanBeRemoved() {
        AtomicInteger runs = new AtomicInteger();
        Object token = telemetry.addAction(() -> {
            runs.incrementAndGet();
            telemetry.addData("fromAction", runs.get());
        });
        telemetry.update();
        assertEquals(1, runs.get());
        assertEquals("fromAction", onlyItem(0).caption);

        assertTrue(telemetry.removeAction(token));
        telemetry.update();
        assertEquals(1, runs.get());
    }

    @Test
    public void logAndSpeakGoToTheOrderedHistory() {
        telemetry.log().add("started");
        telemetry.log().add("pose %d", 3);
        telemetry.speak("hello");

        List<TelemetryStore.LogEntry> entries = store.logSince(0);
        assertEquals(3, entries.size());
        assertEquals("started", entries.get(0).text);
        assertEquals("pose 3", entries.get(1).text);
        assertEquals(TelemetryStore.KIND_SPEAK, entries.get(2).kind);

        // Clearing the DS log does not erase what API clients already hold.
        telemetry.log().clear();
        assertEquals(3, store.logSince(0).size());
    }

    @Test
    public void frameJsonIsValidAndComplete() throws Exception {
        telemetry.addData("rpm", "%.0f rpm", 100.0);
        telemetry.addLine("Header");
        telemetry.update();

        JSONObject frame = new JSONObject(store.latest().toJson());
        assertEquals(1, frame.getLong("seq"));
        JSONArray lines = frame.getJSONArray("lines");
        assertEquals(2, lines.length());
        JSONObject item = lines.getJSONObject(0).getJSONArray("items").getJSONObject(0);
        assertEquals("rpm", item.getString("caption"));
        assertEquals("number", item.getString("type"));
        assertEquals(100.0, item.getDouble("value"), 0);
        assertEquals("rpm", item.getString("unit"));
        assertEquals("Header", lines.getJSONObject(1).getString("caption"));
    }
}
