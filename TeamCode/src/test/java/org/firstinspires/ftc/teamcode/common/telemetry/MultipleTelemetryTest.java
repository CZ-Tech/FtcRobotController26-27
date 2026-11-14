package org.firstinspires.ftc.teamcode.common.telemetry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.network.TelemetryStore;
import org.junit.Before;
import org.junit.Test;

import java.util.concurrent.atomic.AtomicInteger;

/** Uses two {@link ApiTelemetry} as the targets, each with its own store to observe. */
public class MultipleTelemetryTest {
    private TelemetryStore storeA;
    private TelemetryStore storeB;
    private MultipleTelemetry telemetry;

    @Before
    public void setUp() {
        storeA = new TelemetryStore();
        storeB = new TelemetryStore();
        telemetry = new MultipleTelemetry(new ApiTelemetry(storeA), new ApiTelemetry(storeB));
    }

    @Test
    public void everyTargetReceivesDataAndUpdates() {
        telemetry.addData("rpm", "%.0f rpm", 3000.0);
        telemetry.addLine("Header").addData("x", 1);
        telemetry.update();

        for (TelemetryStore store : new TelemetryStore[]{storeA, storeB}) {
            assertEquals(1, store.latest().sequence);
            assertEquals(2, store.latest().lines.size());
            assertEquals("rpm", store.latest().lines.get(0).items.get(0).unit);
            assertEquals("Header", store.latest().lines.get(1).caption);
        }
    }

    @Test
    public void itemHandleChangesEveryTarget() {
        Telemetry.Item item = telemetry.addData("x", 1);
        item.setCaption("y").setValue("%.1f m", 2.0);
        item.addData("z", 5);
        telemetry.update();

        for (TelemetryStore store : new TelemetryStore[]{storeA, storeB}) {
            TelemetryStore.Line line = store.latest().lines.get(0);
            assertEquals("y", line.items.get(0).caption);
            assertEquals("2.0 m", line.items.get(0).text);
            assertEquals("z", line.items.get(1).caption);
        }
    }

    @Test
    public void producerIsEvaluatedOncePerUpdateEvenWithSeveralTargets() {
        AtomicInteger calls = new AtomicInteger();
        telemetry.addData("count", (Func<Integer>) calls::incrementAndGet);
        telemetry.update();

        assertEquals(1, calls.get());
        assertEquals(1.0,
                (Double) storeA.latest().lines.get(0).items.get(0).value, 0);
        assertEquals(1.0,
                (Double) storeB.latest().lines.get(0).items.get(0).value, 0);

        // The retained producer is evaluated again by the next cycle, again only once.
        telemetry.update();
        assertEquals(2, calls.get());
        assertEquals(2.0,
                (Double) storeB.latest().lines.get(0).items.get(0).value, 0);
    }

    @Test
    public void removeItemRemovesFromEveryTarget() {
        Telemetry.Item item = telemetry.addData("x", 1);
        telemetry.addData("y", 2);

        assertTrue(telemetry.removeItem(item));
        telemetry.update();

        assertEquals(1, storeA.latest().lines.size());
        assertEquals(1, storeB.latest().lines.size());
        assertFalse(telemetry.removeItem(item));
    }

    @Test
    public void foreignItemsAreNotRemoved() {
        Telemetry.Item foreign = new ApiTelemetry(new TelemetryStore()).addData("x", 1);
        assertFalse(telemetry.removeItem(foreign));
    }

    @Test
    public void actionsRunOncePerUpdateAndFeedEveryTarget() {
        AtomicInteger runs = new AtomicInteger();
        telemetry.addAction(() -> telemetry.addData("tick", runs.incrementAndGet()));
        telemetry.update();

        assertEquals(1, runs.get());
        assertEquals("tick", storeA.latest().lines.get(0).items.get(0).caption);
        assertEquals("tick", storeB.latest().lines.get(0).items.get(0).caption);
    }

    @Test
    public void logAndSpeakReachEveryTarget() {
        telemetry.log().add("hello");
        telemetry.speak("hi");

        assertEquals(2, storeA.logSince(0).size());
        assertEquals(2, storeB.logSince(0).size());
    }

    @Test
    public void settersApplyToAllAndGettersAnswerFromThePrimary() {
        telemetry.setAutoClear(false);
        telemetry.addData("a", 1);
        telemetry.update();
        telemetry.update();

        assertFalse(telemetry.isAutoClear());
        assertEquals(1, storeA.latest().lines.size());
        assertEquals(1, storeB.latest().lines.size());
    }

    @Test(expected = IllegalArgumentException.class)
    public void requiresAtLeastOneTarget() {
        new MultipleTelemetry(java.util.Collections.<Telemetry>emptyList());
    }
}
