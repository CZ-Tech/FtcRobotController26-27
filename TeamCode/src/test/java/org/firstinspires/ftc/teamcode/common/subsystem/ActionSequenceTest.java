package org.firstinspires.ftc.teamcode.common.subsystem;

import org.junit.Test;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import static org.junit.Assert.*;

public class ActionSequenceTest {
    @Test public void buildingDoesNothingUntilSubmittedAndUpdated() {
        List<Integer> log = new ArrayList<>();
        SubsystemLoop loop = new SubsystemLoop();
        ActionSequence flow = ActionSequence.create().run(() -> log.add(1)).run(() -> log.add(2));
        loop.update();
        assertTrue(log.isEmpty());
        loop.start(flow);
        assertTrue(log.isEmpty());
        loop.update();
        assertEquals(Arrays.asList(1, 2), log);
        loop.update();
        assertEquals(Arrays.asList(1, 2), log);
    }

    @Test public void timedOutputStopsOnlyAfterTheWait() {
        AtomicLong clock = new AtomicLong();
        AtomicLong power = new AtomicLong();
        SubsystemLoop loop = new SubsystemLoop(clock::get);
        loop.start(ActionSequence.create().run(() -> power.set(1))
                .waitMillis(100).run(() -> power.set(0)));
        loop.update();
        assertEquals(1, power.get());
        clock.set(99_999_999L);
        loop.update();
        assertEquals(1, power.get());
        clock.set(100_000_000L);
        loop.update();
        assertEquals(0, power.get());
    }

    @Test public void waitStartsWhenReachedNotWhenBuiltOrSubmitted() {
        AtomicLong clock = new AtomicLong();
        AtomicBoolean ready = new AtomicBoolean();
        List<String> log = new ArrayList<>();
        SubsystemLoop loop = new SubsystemLoop(clock::get);
        loop.start(ActionSequence.create().waitUntil(ready::get).waitMillis(100)
                .run(() -> log.add("done")));
        clock.set(1_000_000_000L);
        loop.update();
        ready.set(true);
        clock.set(2_000_000_000L);
        loop.update();
        assertTrue(log.isEmpty());
        clock.set(2_100_000_000L);
        loop.update();
        assertEquals(Arrays.asList("done"), log);
    }

    @Test public void conditionIsOnlyEvaluatedOnControlThreadDuringUpdate() {
        List<Thread> evaluations = new ArrayList<>();
        AtomicBoolean ready = new AtomicBoolean();
        List<String> log = new ArrayList<>();
        SubsystemLoop loop = new SubsystemLoop();
        loop.start(ActionSequence.create().waitUntil(() -> {
            evaluations.add(Thread.currentThread());
            return ready.get();
        }).run(() -> log.add("done")));
        assertTrue(evaluations.isEmpty());
        loop.update();
        assertTrue(log.isEmpty());
        ready.set(true);
        loop.update();
        assertEquals(Arrays.asList(Thread.currentThread(), Thread.currentThread()), evaluations);
        assertEquals(Arrays.asList("done"), log);
    }

    @Test public void consecutiveWaitsEachStartWhenReached() {
        AtomicLong clock = new AtomicLong();
        AtomicBoolean done = new AtomicBoolean();
        SubsystemLoop loop = new SubsystemLoop(clock::get);
        loop.start(ActionSequence.create().waitMillis(50).waitMillis(50).run(() -> done.set(true)));
        loop.update();
        clock.set(50_000_000L);
        loop.update();
        assertFalse(done.get());
        clock.set(99_000_000L);
        loop.update();
        assertFalse(done.get());
        clock.set(100_000_000L);
        loop.update();
        assertTrue(done.get());
    }

    @Test public void zeroWaitsAndEmptyFlowCompleteWithoutExtraTick() {
        List<Integer> log = new ArrayList<>();
        SubsystemLoop loop = new SubsystemLoop();
        loop.start(ActionSequence.create());
        loop.start(ActionSequence.create().waitMillis(0).waitUntil(() -> true).run(() -> log.add(1)));
        loop.update();
        assertEquals(Arrays.asList(1), log);
    }

    @Test public void waitHandlesNanoTimeWrapping() {
        AtomicLong clock = new AtomicLong(Long.MAX_VALUE - 5_000_000L);
        AtomicBoolean done = new AtomicBoolean();
        SubsystemLoop loop = new SubsystemLoop(clock::get);
        loop.start(ActionSequence.create().waitMillis(10).run(() -> done.set(true)));
        loop.update();
        clock.addAndGet(9_000_000L);
        loop.update();
        assertFalse(done.get());
        clock.addAndGet(1_000_000L);
        loop.update();
        assertTrue(done.get());
    }

    @Test public void sequenceIsSealedAndSingleUseAcrossLoops() {
        SubsystemLoop loop = new SubsystemLoop();
        ActionSequence flow = ActionSequence.create();
        loop.start(flow);
        assertThrows(IllegalStateException.class, () -> flow.run(() -> {}));
        assertThrows(IllegalStateException.class, () -> flow.waitMillis(1));
        assertThrows(IllegalStateException.class, () -> loop.start(flow));
        assertThrows(IllegalStateException.class, () -> new SubsystemLoop().start(flow));
        loop.update();
        assertThrows(IllegalStateException.class, () -> loop.start(flow));
    }

    @Test public void invalidStepsFailBeforeSubmission() {
        ActionSequence flow = ActionSequence.create();
        assertThrows(NullPointerException.class, () -> flow.run(null));
        assertThrows(NullPointerException.class, () -> flow.waitUntil(null));
        assertThrows(IllegalArgumentException.class, () -> flow.waitMillis(-1));
        assertThrows(IllegalArgumentException.class, () -> flow.waitMillis(Long.MAX_VALUE));
    }

    @Test public void callbacksFailFastWithoutRunningFollowingSteps() {
        RuntimeException error = new IllegalArgumentException("hardware failed");
        List<Integer> log = new ArrayList<>();
        SubsystemLoop loop = new SubsystemLoop();
        loop.start(ActionSequence.create().run(() -> { log.add(0); throw error; })
                .run(() -> log.add(1)));
        assertSame(error, assertThrows(RuntimeException.class, loop::update));
        loop.update(); // A caller catching the failure must not implicitly retry hardware writes.
        loop.shutdown();
        assertEquals(Arrays.asList(0), log);
    }

    @Test public void conditionFailuresAlsoPropagateToOpModeCleanup() {
        AssertionError error = new AssertionError("sensor failed");
        SubsystemLoop loop = new SubsystemLoop();
        loop.start(ActionSequence.create().waitUntil(() -> { throw error; }));
        assertSame(error, assertThrows(AssertionError.class, loop::update));
        loop.shutdown();
    }
}
