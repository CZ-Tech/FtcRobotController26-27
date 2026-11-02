package org.firstinspires.ftc.teamcode.common.subsystem;

import org.junit.Test;
import static org.junit.Assert.*;

public class SchedulerTest {
    interface Actions extends ScheduleApi<Actions> {
        Actions add(int number);
        Actions reset();
    }

    public static final class Counter implements ScheduleLifecycle {
        int total;
        int cancellations;
        public void add(int number) { total += number; }
        public void reset() { total = 0; }
        @Override public void onScheduleCancelled() { cancellations++; }
    }

    @Test public void draftDoesNotExecuteUntilTick() {
        Counter target = new Counter();
        Scheduler<Actions> scheduler = Scheduler.of(Actions.class, target);
        Actions draft = scheduler.schedule().add(2).add(3);
        assertEquals(0, target.total);
        assertFalse(draft.isExecuted());
        draft.execute();
        assertTrue(draft.isExecuted());
        assertEquals(0, target.total);
        scheduler.update();
        assertEquals(5, target.total);
    }

    @Test public void latestSubmissionWins() {
        Counter target = new Counter();
        Scheduler<Actions> scheduler = Scheduler.of(Actions.class, target);
        Actions first = scheduler.schedule().add(100).execute();
        scheduler.schedule().add(7).execute();
        assertTrue(first.isCancelled());
        scheduler.update();
        assertEquals(7, target.total);
    }

    @Test public void cancelTriggersHookOnUpdate() {
        Counter target = new Counter();
        Scheduler<Actions> scheduler = Scheduler.of(Actions.class, target);
        scheduler.schedule().add(12).execute();
        scheduler.update();
        scheduler.schedule().waitUntil(() -> false).execute();
        scheduler.update();
        scheduler.cancelSchedule();
        assertEquals(0, target.cancellations);
        scheduler.update();
        assertEquals(1, target.cancellations);
    }
}
