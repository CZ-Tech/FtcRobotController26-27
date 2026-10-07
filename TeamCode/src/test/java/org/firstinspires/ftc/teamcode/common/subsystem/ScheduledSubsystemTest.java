package org.firstinspires.ftc.teamcode.common.subsystem;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.atomic.AtomicLong;

public class ScheduledSubsystemTest {

    interface TestSchedule extends ScheduleApi<TestSchedule> {
        TestSchedule add(int value);
        TestSchedule set(int value);
    }

    interface BrokenSchedule extends ScheduleApi<BrokenSchedule> {
        BrokenSchedule missing();
    }

    private static final class TestSubsystem extends ScheduledSubsystem<TestSchedule> {
        int value;
        int cancelledCount;

        TestSubsystem(AtomicLong clock) {
            super(TestSchedule.class, clock::get);
        }

        public TestSubsystem add(int amount) {
            value += amount;
            return this;
        }

        public void set(int value) {
            this.value = value;
        }

        @Override
        protected void onScheduleCancelled() {
            cancelledCount++;
        }
    }

    private static final class BrokenSubsystem extends ScheduledSubsystem<BrokenSchedule> {
        BrokenSubsystem(AtomicLong clock) {
            super(BrokenSchedule.class, clock::get);
        }
    }

    @Test
    public void buildPublishesImmediatelyButHardwareWaitsForUpdate() {
        AtomicLong clock = new AtomicLong();
        TestSubsystem subsystem = new TestSubsystem(clock);

        TestSchedule schedule = subsystem.schedule().add(2).set(7).build();

        assertEquals(0, subsystem.value);
        assertTrue(schedule.isBuilt());

        subsystem.update();

        assertEquals(7, subsystem.value);
        assertFalse(subsystem.hasActiveSchedule());
    }

    @Test
    public void missingBuildNeverPublishesDraft() {
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
    public void latestBuiltScheduleWinsCapacityOneMailbox() {
        AtomicLong clock = new AtomicLong();
        TestSubsystem subsystem = new TestSubsystem(clock);

        subsystem.schedule().set(1).build();
        subsystem.schedule().set(2).build();
        subsystem.schedule().set(3).build();

        subsystem.update();

        assertEquals(3, subsystem.value);
    }

    @Test
    public void newScheduleCancelsOldProxyAndOldProxyBecomesNoOp() {
        AtomicLong clock = new AtomicLong();
        TestSubsystem subsystem = new TestSubsystem(clock);

        TestSchedule old = subsystem.schedule().set(1);
        TestSchedule newer = subsystem.schedule().set(2);

        TestSchedule cancelled = old.add(100).waitMillis(100).set(100);
        assertTrue(cancelled.isCancelled());

        newer.build();
        subsystem.update();

        assertEquals(2, subsystem.value);
    }

    @Test
    public void waitMillisDoesNotBlockAndResumesAfterDeadline() {
        AtomicLong clock = new AtomicLong();
        TestSubsystem subsystem = new TestSubsystem(clock);

        subsystem.schedule()
                .set(1)
                .waitMillis(100)
                .set(2)
                .build();

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
                .build();

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
                .build();
        subsystem.update();
        assertEquals(1, subsystem.value);

        subsystem.schedule().set(5).build();
        subsystem.update();

        assertEquals(5, subsystem.value);
        assertEquals(1, subsystem.cancelledCount);
    }

    @Test
    public void voidConcreteMethodCanStillBeFluentInScheduleInterface() {
        AtomicLong clock = new AtomicLong();
        TestSubsystem subsystem = new TestSubsystem(clock);

        subsystem.schedule().set(42).build();
        subsystem.update();

        assertEquals(42, subsystem.value);
    }

    @Test(expected = IllegalStateException.class)
    public void scheduleApiMismatchFailsAtSubsystemConstruction() {
        new BrokenSubsystem(new AtomicLong());
    }

    @Test
    public void concurrentScheduleCreationLeavesExactlyOneLiveNewestDraft() throws Exception {
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
        TestSchedule newest = null;
        synchronized (handles) {
            for (TestSchedule handle : handles) {
                if (!handle.isCancelled()) {
                    live++;
                    newest = handle;
                }
            }
        }
        assertEquals(1, live);

        newest.build();
        subsystem.update();
        assertTrue(subsystem.value >= 1 && subsystem.value <= 16);
    }
}
