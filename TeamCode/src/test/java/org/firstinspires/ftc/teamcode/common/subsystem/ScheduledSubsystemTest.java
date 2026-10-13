package org.firstinspires.ftc.teamcode.common.subsystem;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.ArrayList;
import java.lang.reflect.Method;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.atomic.AtomicLong;

public class ScheduledSubsystemTest {

    @Test
    public void generatedScheduleMethodsMatchPublicBusinessActions() {
        Class<?>[] types = {Intaker.class, Belt.class, Door.class, Classifier.class, Shooter.class};
        for (Class<?> type : types) {
            try {
                Class<?> api = Class.forName(type.getName() + "Schedule");
                for (Method scheduledMethod : api.getDeclaredMethods()) {
                    Method hardwareMethod = type.getDeclaredMethod(
                            scheduledMethod.getName(), scheduledMethod.getParameterTypes());
                    assertTrue(hardwareMethod.getName(),
                            java.lang.reflect.Modifier.isPublic(hardwareMethod.getModifiers()));
                }
            } catch (ReflectiveOperationException e) {
                throw new AssertionError("Generated schedule does not match " + type, e);
            }
        }
    }

    interface TestSchedule extends ScheduleApi<TestSchedule> {
        TestSchedule add(int value);
        TestSchedule set(int value);
    }

    interface BrokenSchedule extends ScheduleApi<BrokenSchedule> {
        BrokenSchedule missing();
    }

    private static final class TestSubsystem implements ScheduleLifecycle {
        int value;
        int cancelledCount;
        final Scheduler<TestSchedule> scheduler;

        TestSubsystem(AtomicLong clock) {
            scheduler = new Scheduler<>(TestSchedule.class, this, clock::get);
        }

        TestSchedule schedule() { return scheduler.schedule(); }
        void update() { scheduler.update(); }
        boolean hasActiveSchedule() { return scheduler.hasActiveSchedule(); }

        public void add(int amount) {
            value += amount;
        }

        public void set(int value) {
            this.value = value;
        }

        @Override
        public void onScheduleCancelled() {
            cancelledCount++;
        }
    }

    private static final class BrokenSubsystem {
        BrokenSubsystem(AtomicLong clock) {
            new Scheduler<>(BrokenSchedule.class, this, clock::get);
        }
    }

    @Test
    public void executePublishesButHardwareWaitsForUpdate() {
        AtomicLong clock = new AtomicLong();
        TestSubsystem subsystem = new TestSubsystem(clock);

        TestSchedule schedule = subsystem.schedule().add(2).set(7).execute();

        assertEquals(0, subsystem.value);
        assertTrue(schedule.isExecuted());

        subsystem.update();

        assertEquals(7, subsystem.value);
        assertFalse(subsystem.hasActiveSchedule());
    }

    @Test
    public void missingExecuteNeverPublishesDraft() {
        AtomicLong clock = new AtomicLong();
        TestSubsystem subsystem = new TestSubsystem(clock);

        subsystem.schedule().add(2).add(3).set(10);
        assertEquals(0, subsystem.value);

        subsystem.update();
        subsystem.update();

        assertEquals(0, subsystem.value);
        assertFalse(subsystem.hasActiveSchedule());
    }

    @Test
    public void latestExecutedScheduleWinsCapacityOneMailbox() {
        AtomicLong clock = new AtomicLong();
        TestSubsystem subsystem = new TestSubsystem(clock);

        subsystem.schedule().set(1).execute();
        subsystem.schedule().set(2).execute();
        subsystem.schedule().set(3).execute();

        subsystem.update();

        assertEquals(3, subsystem.value);
    }

    @Test
    public void draftsAreIndependentUntilExecute() {
        AtomicLong clock = new AtomicLong();
        TestSubsystem subsystem = new TestSubsystem(clock);

        TestSchedule first = subsystem.schedule().set(1);
        TestSchedule second = subsystem.schedule().set(2);

        assertFalse(first.isCancelled());
        assertFalse(second.isCancelled());

        second.execute();
        first.add(10).execute();
        subsystem.update();

        assertEquals(11, subsystem.value);
        assertTrue(second.isCancelled());
    }

    @Test
    public void waitMillisDoesNotBlockAndResumesAfterDeadline() {
        AtomicLong clock = new AtomicLong();
        TestSubsystem subsystem = new TestSubsystem(clock);

        subsystem.schedule()
                .set(1)
                .waitMillis(100)
                .set(2)
                .execute();

        subsystem.update();
        assertEquals(1, subsystem.value);
        assertTrue(subsystem.hasActiveSchedule());

        clock.set(99_000_000L);
        subsystem.update();
        assertEquals(1, subsystem.value);

        clock.set(100_000_000L);
        subsystem.update();
        assertEquals(2, subsystem.value);
        assertFalse(subsystem.hasActiveSchedule());
    }

    @Test
    public void waitUntilDoesNotAdvanceUntilConditionBecomesTrue() {
        AtomicLong clock = new AtomicLong();
        AtomicLong ready = new AtomicLong();
        TestSubsystem subsystem = new TestSubsystem(clock);

        subsystem.schedule()
                .set(1)
                .waitUntil(() -> ready.get() == 1)
                .set(2)
                .execute();

        subsystem.update();
        assertEquals(1, subsystem.value);

        subsystem.update();
        assertEquals(1, subsystem.value);

        ready.set(1);
        subsystem.update();
        assertEquals(2, subsystem.value);
    }

    @Test
    public void preemptingActiveScheduleHappensOnNextControlTick() {
        AtomicLong clock = new AtomicLong();
        TestSubsystem subsystem = new TestSubsystem(clock);

        subsystem.schedule()
                .set(1)
                .waitMillis(1000)
                .set(9)
                .execute();
        subsystem.update();
        assertEquals(1, subsystem.value);

        TestSchedule draft = subsystem.schedule().set(5);

        // Merely creating a draft must not preempt the running schedule.
        subsystem.update();
        assertEquals(1, subsystem.value);
        assertEquals(0, subsystem.cancelledCount);

        draft.execute();
        subsystem.update();

        assertEquals(5, subsystem.value);
        assertEquals(1, subsystem.cancelledCount);
    }

    @Test
    public void voidConcreteMethodCanStillBeFluentInScheduleInterface() {
        AtomicLong clock = new AtomicLong();
        TestSubsystem subsystem = new TestSubsystem(clock);

        subsystem.schedule().set(42).execute();
        subsystem.update();

        assertEquals(42, subsystem.value);
    }

    @Test(expected = IllegalStateException.class)
    public void scheduleApiMismatchFailsAtSubsystemConstruction() {
        new BrokenSubsystem(new AtomicLong());
    }

    @Test
    public void concurrentScheduleCreationLeavesAllDraftsIndependent() throws Exception {
        AtomicLong clock = new AtomicLong();
        TestSubsystem subsystem = new TestSubsystem(clock);
        CountDownLatch start = new CountDownLatch(1);
        List<TestSchedule> handles = new ArrayList<>();
        List<Thread> threads = new ArrayList<>();

        for (int i = 0; i < 16; i++) {
            final int value = i + 1;
            Thread thread = new Thread(() -> {
                try {
                    start.await();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
                TestSchedule handle = subsystem.schedule().set(value);
                synchronized (handles) {
                    handles.add(handle);
                }
            });
            threads.add(thread);
            thread.start();
        }

        start.countDown();
        for (Thread thread : threads) thread.join();

        int live = 0;
        synchronized (handles) {
            for (TestSchedule handle : handles) {
                if (!handle.isCancelled()) {
                    live++;
                }
            }
        }
        assertEquals(16, live);

        TestSchedule chosen;
        synchronized (handles) {
            chosen = handles.get(7);
        }
        chosen.execute();
        subsystem.update();
        assertTrue(subsystem.value >= 1 && subsystem.value <= 16);
    }

    @Test
    public void executedScheduleCanBeCancelledThroughItsDraftHandle() {
        AtomicLong clock = new AtomicLong();
        TestSubsystem subsystem = new TestSubsystem(clock);

        TestSchedule schedule = subsystem.schedule()
                .set(1)
                .waitMillis(1000)
                .set(2)
                .execute();

        subsystem.update();
        assertEquals(1, subsystem.value);

        schedule.cancel();
        subsystem.update();

        assertEquals(1, subsystem.value);
        assertTrue(schedule.isCancelled());
        assertEquals(1, subsystem.cancelledCount);
    }
}
