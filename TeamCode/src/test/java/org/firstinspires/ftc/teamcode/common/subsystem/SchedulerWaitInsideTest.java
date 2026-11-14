package org.firstinspires.ftc.teamcode.common.subsystem;

import static org.junit.Assert.*;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** waitMillis/waitUntil called from inside a subsystem method. */
public class SchedulerWaitInsideTest {
    interface Actions extends ScheduleApi<Actions> {
        Actions a();
        Actions b();
        Actions twoWaits();
        Actions gate();
        Actions bad();
    }

    public static final class Sub implements ScheduleLifecycle {
        final List<String> log = new ArrayList<>();
        boolean ready;
        Runnable onPeriodic;

        public void a() { log.add("a"); waitMillis(100); }
        public void b() { log.add("b"); }
        public void twoWaits() { log.add("two"); waitMillis(100); waitMillis(200); }
        public void gate() { log.add("gate"); waitUntil(() -> ready); }
        public void bad() { waitMillis(10); }
        @Override public void periodic() { if (onPeriodic != null) onPeriodic.run(); }
    }

    private long now;
    private final Sub sub = new Sub();
    private final Scheduler<Actions> scheduler =
            new Scheduler<>(Actions.class, sub, () -> now);

    private void advanceMs(long ms) { now += ms * 1_000_000L; }

    @Test public void waitInsideMethodDelaysNextStep() {
        scheduler.schedule().a().b().execute();
        scheduler.update();
        assertEquals(Arrays.asList("a"), sub.log);
        advanceMs(99);
        scheduler.update();
        assertEquals(Arrays.asList("a"), sub.log);
        advanceMs(1);
        scheduler.update();
        assertEquals(Arrays.asList("a", "b"), sub.log);
        assertFalse(scheduler.hasActiveSchedule());
    }

    @Test public void equivalentToOutsideWait() {
        Sub outsideSub = new Sub();
        Scheduler<Actions> outside = new Scheduler<>(Actions.class, outsideSub, () -> now);
        outside.schedule().b().waitMillis(100).b().execute();
        outside.update();
        assertEquals(Arrays.asList("b"), outsideSub.log);
        advanceMs(100);
        outside.update();
        assertEquals(Arrays.asList("b", "b"), outsideSub.log);
    }

    @Test public void multipleWaitsAccumulateInOrder() {
        scheduler.schedule().twoWaits().b().execute();
        scheduler.update();
        advanceMs(100);
        scheduler.update();
        assertEquals(Arrays.asList("two"), sub.log);
        advanceMs(199);
        scheduler.update();
        assertEquals(Arrays.asList("two"), sub.log);
        advanceMs(1);
        scheduler.update();
        assertEquals(Arrays.asList("two", "b"), sub.log);
    }

    @Test public void waitUntilInsideMethod() {
        scheduler.schedule().gate().b().execute();
        scheduler.update();
        scheduler.update();
        assertEquals(Arrays.asList("gate"), sub.log);
        sub.ready = true;
        scheduler.update();
        assertEquals(Arrays.asList("gate", "b"), sub.log);
    }

    @Test public void newExecuteCancelsPendingInsertedWait() {
        scheduler.schedule().a().b().execute();
        scheduler.update();
        scheduler.schedule().b().execute();
        scheduler.update();
        assertEquals(Arrays.asList("a", "b"), sub.log);
        advanceMs(1000);
        scheduler.update();
        assertEquals(Arrays.asList("a", "b"), sub.log); // old schedule's b never runs
    }

    @Test public void rescheduleSameDraftWaitIsPerExecution() {
        scheduler.schedule().a().b().execute();
        scheduler.update();
        scheduler.cancelSchedule();
        scheduler.update();
        scheduler.schedule().a().b().execute();
        scheduler.update();
        advanceMs(100);
        scheduler.update();
        assertEquals(Arrays.asList("a", "a", "b"), sub.log);
    }

    @Test public void outsideAScheduledStepThrows() {
        try {
            sub.waitMillis(10);
            fail("expected IllegalStateException");
        } catch (IllegalStateException expected) {
            // ok
        }
    }

    @Test public void fromPeriodicThrows() {
        sub.onPeriodic = () -> sub.waitMillis(10);
        try {
            scheduler.update();
            fail("expected IllegalStateException");
        } catch (IllegalStateException expected) {
            // ok
        }
    }

    @Test public void negativeMillisRejected() {
        scheduler.schedule().bad().execute();
        // bad() passes a valid value, so ensure negative is rejected directly:
        try {
            Scheduler.insertWaitMillis(sub, -1);
            fail("expected IllegalArgumentException");
        } catch (IllegalArgumentException expected) {
            // ok (rejected before the context check)
        }
    }

    @Test public void contextIsClearedAfterStep() {
        scheduler.schedule().a().execute();
        scheduler.update();
        try {
            sub.waitMillis(10);
            fail("expected IllegalStateException");
        } catch (IllegalStateException expected) {
            // ok: CURRENT was restored
        }
    }
}
