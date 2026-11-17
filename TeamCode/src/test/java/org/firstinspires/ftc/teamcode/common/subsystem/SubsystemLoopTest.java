package org.firstinspires.ftc.teamcode.common.subsystem;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

import static org.junit.Assert.*;

public class SubsystemLoopTest {
    @Test
    public void everySubmissionRunsInFifoOrderIncludingRepeatedRunnable() {
        SubsystemLoop loop = new SubsystemLoop();
        List<Integer> calls = new ArrayList<>();
        Runnable repeated = () -> calls.add(2);
        loop.enqueue(() -> calls.add(1));
        loop.enqueue(repeated);
        loop.enqueue(repeated);
        loop.enqueue(() -> calls.add(3));
        assertTrue(calls.isEmpty());
        loop.update();
        assertEquals(Arrays.asList(1, 2, 2, 3), calls);
        loop.update();
        assertEquals(4, calls.size());
    }

    @Test
    public void producerSubmissionsExecuteOnlyOnOwnerThread() throws Exception {
        SubsystemLoop loop = new SubsystemLoop();
        Thread owner = Thread.currentThread();
        List<Thread> executions = new ArrayList<>();
        AtomicReference<Throwable> failure = new AtomicReference<>();
        Thread producer = new Thread(() -> {
            try {
                loop.enqueue(() -> executions.add(Thread.currentThread()));
                loop.start(ActionSequence.create().run(
                        () -> executions.add(Thread.currentThread())));
            } catch (Throwable t) {
                failure.set(t);
            }
        }, "subsystem-test-producer");
        producer.start();
        producer.join(5000);
        assertFalse("producer must finish without owner update", producer.isAlive());
        assertNull(failure.get());
        assertTrue(executions.isEmpty());
        loop.update();
        assertEquals(Arrays.asList(owner, owner), executions);
    }

    @Test
    public void periodicContinuesWhileTimedFlowWaits() {
        AtomicLong now = new AtomicLong();
        SubsystemLoop loop = new SubsystemLoop(now::get);
        Hooks hooks = loop.add(new Hooks());
        AtomicInteger calls = new AtomicInteger();
        loop.start(ActionSequence.create().waitMillis(10).run(calls::incrementAndGet));
        loop.update();
        now.set(9_999_999L);
        loop.update();
        assertEquals(0, calls.get());
        assertEquals(2, hooks.periodic);
        now.set(10_000_000L);
        loop.update();
        assertEquals(1, calls.get());
        assertEquals(3, hooks.periodic);
    }

    @Test
    public void waitingFlowDoesNotBlockOrReplaceOtherFlows() {
        SubsystemLoop loop = new SubsystemLoop();
        AtomicReference<Boolean> ready = new AtomicReference<>(false);
        List<String> calls = new ArrayList<>();
        loop.start(ActionSequence.create().run(() -> calls.add("first-start"))
                .waitUntil(ready::get).run(() -> calls.add("first-end")));
        loop.start(ActionSequence.create().run(() -> calls.add("second")));
        loop.update();
        assertEquals(Arrays.asList("first-start", "second"), calls);
        loop.start(ActionSequence.create().run(() -> calls.add("third")));
        loop.update();
        ready.set(true);
        loop.update();
        assertEquals(Arrays.asList("first-start", "second", "third", "first-end"), calls);
    }

    @Test
    public void eachUpdateDrainsAtMost64CommandsAndStillRunsPeriodic() {
        SubsystemLoop loop = new SubsystemLoop();
        Hooks hooks = loop.add(new Hooks());
        List<Integer> calls = new ArrayList<>();
        for (int i = 0; i < 130; i++) {
            final int value = i;
            loop.enqueue(() -> calls.add(value));
        }
        loop.update();
        assertEquals(64, calls.size());
        assertEquals(1, hooks.periodic);
        loop.update();
        assertEquals(128, calls.size());
        loop.update();
        assertEquals(130, calls.size());
        for (int i = 0; i < 130; i++) assertEquals(i, calls.get(i).intValue());
        assertEquals(3, hooks.periodic);
    }

    @Test
    public void stepBudgetIs32ForEachFlowRatherThanShared() {
        SubsystemLoop loop = new SubsystemLoop();
        AtomicInteger first = new AtomicInteger();
        AtomicInteger second = new AtomicInteger();
        ActionSequence a = ActionSequence.create();
        ActionSequence b = ActionSequence.create();
        for (int i = 0; i < 65; i++) {
            a.run(first::incrementAndGet);
            b.run(second::incrementAndGet);
        }
        loop.start(a);
        loop.start(b);
        loop.update();
        assertEquals(32, first.get());
        assertEquals(32, second.get());
        loop.update();
        assertEquals(64, first.get());
        assertEquals(64, second.get());
        loop.update();
        assertEquals(65, first.get());
        assertEquals(65, second.get());
    }

    @Test
    public void shutdownDiscardsQueuedCommandsQueuedFlowsAndActiveFlows() {
        SubsystemLoop loop = new SubsystemLoop();
        Hooks hooks = loop.add(new Hooks());
        AtomicInteger calls = new AtomicInteger();
        loop.start(ActionSequence.create().waitUntil(() -> false)
                .run(calls::incrementAndGet));
        loop.update();
        loop.enqueue(calls::incrementAndGet);
        loop.start(ActionSequence.create().run(calls::incrementAndGet));
        loop.shutdown();
        loop.update();
        assertEquals(0, calls.get());
        assertEquals(1, hooks.periodic);
        assertEquals(1, hooks.shutdown);
    }

    @Test
    public void shutdownRejectsCommandsAndFlowsAndIsIdempotent() {
        SubsystemLoop loop = new SubsystemLoop();
        Hooks hooks = loop.add(new Hooks());
        loop.shutdown();
        loop.shutdown();
        assertThrows(IllegalStateException.class, () -> loop.enqueue(() -> {}));
        assertThrows(IllegalStateException.class,
                () -> loop.start(ActionSequence.create()));
        assertEquals(1, hooks.shutdown);
    }

    @Test
    public void failedShutdownHookDoesNotPreventOtherHooksOrClose() {
        SubsystemLoop loop = new SubsystemLoop();
        RuntimeException failure = new RuntimeException("shutdown failure");
        AtomicInteger failingCalls = new AtomicInteger();
        loop.add(new SubsystemLifecycle() {
            @Override public void onShutdown() {
                failingCalls.incrementAndGet();
                throw failure;
            }
        });
        Hooks other = loop.add(new Hooks());
        assertSame(failure, assertThrows(RuntimeException.class, loop::shutdown));
        assertEquals(1, other.shutdown);
        assertThrows(IllegalStateException.class, () -> loop.enqueue(() -> {}));
        loop.shutdown();
        assertEquals(1, failingCalls.get());
        assertEquals(1, other.shutdown);
    }

    @Test
    public void updateOnWrongThreadIsRejectedWithoutConsumingQueue() throws Exception {
        SubsystemLoop loop = new SubsystemLoop();
        AtomicInteger calls = new AtomicInteger();
        loop.enqueue(calls::incrementAndGet);
        assertTrue(onOtherThread(loop::update) instanceof IllegalStateException);
        assertEquals(0, calls.get());
        loop.update();
        assertEquals(1, calls.get());
    }

    @Test
    public void shutdownOnWrongThreadIsRejectedWithoutClosingLoop() throws Exception {
        SubsystemLoop loop = new SubsystemLoop();
        Hooks hooks = loop.add(new Hooks());
        assertTrue(onOtherThread(loop::shutdown) instanceof IllegalStateException);
        AtomicInteger calls = new AtomicInteger();
        loop.enqueue(calls::incrementAndGet);
        loop.update();
        assertEquals(1, calls.get());
        assertEquals(0, hooks.shutdown);
        loop.shutdown();
        assertEquals(1, hooks.shutdown);
    }

    @Test
    public void clearFlowsInsideFlowCallbackStopsRemainingActionsAndOtherFlows() {
        SubsystemLoop loop = new SubsystemLoop();
        List<String> calls = new ArrayList<>();
        loop.start(ActionSequence.create().run(() -> {
            calls.add("clear");
            loop.clearFlows();
        }).run(() -> calls.add("forbidden-tail")));
        loop.start(ActionSequence.create().run(() -> calls.add("forbidden-other")));
        loop.update();
        loop.update();
        assertEquals(Arrays.asList("clear"), calls);
    }

    @Test
    public void shutdownInsideFlowCallbackStopsRemainingActionsAndPeriodic() {
        SubsystemLoop loop = new SubsystemLoop();
        Hooks hooks = loop.add(new Hooks());
        List<String> calls = new ArrayList<>();
        loop.start(ActionSequence.create().run(() -> {
            calls.add("shutdown");
            loop.shutdown();
        }).run(() -> calls.add("forbidden-tail")));
        loop.start(ActionSequence.create().run(() -> calls.add("forbidden-other")));
        loop.update();
        loop.update();
        assertEquals(Arrays.asList("shutdown"), calls);
        assertEquals(0, hooks.periodic);
        assertEquals(1, hooks.shutdown);
    }

    @Test
    public void clearFlowsDiscardsQueuedStartsButPreservesOrdinaryQueuedCalls() {
        SubsystemLoop loop = new SubsystemLoop();
        List<String> calls = new ArrayList<>();
        loop.enqueue(() -> calls.add("before"));
        loop.start(ActionSequence.create().run(() -> calls.add("forbidden")));
        loop.enqueue(() -> calls.add("after"));
        loop.clearFlows();
        loop.update();
        assertEquals(Arrays.asList("before", "after"), calls);
        loop.start(ActionSequence.create().run(() -> calls.add("new-flow")));
        loop.update();
        assertEquals(Arrays.asList("before", "after", "new-flow"), calls);
    }

    @Test
    public void pendingStartPublishedByCallbackIsRemovedByQueuedStopAll() {
        SubsystemLoop loop = new SubsystemLoop();
        List<String> calls = new ArrayList<>();
        loop.enqueue(() -> {
            calls.add("start-callback");
            loop.start(ActionSequence.create().run(() -> calls.add("forbidden")));
        });
        // At the loop boundary, stopAll's flow cancellation is clearFlows().
        loop.enqueue(loop::clearFlows);
        loop.enqueue(() -> calls.add("ordinary"));
        loop.update();
        loop.update();
        assertEquals(Arrays.asList("start-callback", "ordinary"), calls);
    }

    @Test
    public void clearFromFlowCallbackAlsoRemovesNewlyQueuedStart() {
        SubsystemLoop loop = new SubsystemLoop();
        AtomicInteger calls = new AtomicInteger();
        loop.start(ActionSequence.create().run(() -> {
            loop.start(ActionSequence.create().run(calls::incrementAndGet));
            loop.clearFlows();
        }).run(calls::incrementAndGet));
        loop.update();
        loop.update();
        assertEquals(0, calls.get());
    }

    @Test
    public void initiallyStoppedLoopRunsNoCommandsFlowsOrPeriodic() {
        AtomicBoolean keepRunning = new AtomicBoolean(false);
        SubsystemLoop loop = new SubsystemLoop(() -> 0L, keepRunning::get);
        Hooks hooks = loop.add(new Hooks());
        AtomicInteger calls = new AtomicInteger();
        loop.enqueue(calls::incrementAndGet);
        loop.start(ActionSequence.create().run(calls::incrementAndGet));
        loop.update();
        loop.update();
        assertEquals(0, calls.get());
        assertEquals(0, hooks.periodic);
    }

    @Test
    public void commandStopPreventsLaterCommandsFlowsAndPeriodic() {
        AtomicBoolean keepRunning = new AtomicBoolean(true);
        SubsystemLoop loop = new SubsystemLoop(() -> 0L, keepRunning::get);
        Hooks hooks = loop.add(new Hooks());
        List<String> calls = new ArrayList<>();
        // Publish an active flow before the stopping command is drained.
        loop.start(ActionSequence.create().run(() -> calls.add("forbidden-flow")));
        loop.enqueue(() -> {
            calls.add("stop-command");
            keepRunning.set(false);
        });
        loop.enqueue(() -> calls.add("forbidden-command"));
        loop.start(ActionSequence.create().run(() -> calls.add("forbidden-queued-flow")));
        loop.update();
        loop.update();
        assertEquals(Arrays.asList("stop-command"), calls);
        assertEquals(0, hooks.periodic);
    }

    @Test
    public void flowStopPreventsNextStepOtherFlowsAndPeriodic() {
        AtomicBoolean keepRunning = new AtomicBoolean(true);
        SubsystemLoop loop = new SubsystemLoop(() -> 0L, keepRunning::get);
        Hooks hooks = loop.add(new Hooks());
        List<String> calls = new ArrayList<>();
        loop.start(ActionSequence.create().run(() -> {
            calls.add("stop-flow");
            keepRunning.set(false);
        }).run(() -> calls.add("forbidden-tail")));
        loop.start(ActionSequence.create().run(() -> calls.add("forbidden-other-flow")));
        loop.update();
        loop.update();
        assertEquals(Arrays.asList("stop-flow"), calls);
        assertEquals(0, hooks.periodic);
    }

    @Test
    public void periodicStopPreventsRemainingHooks() {
        AtomicBoolean keepRunning = new AtomicBoolean(true);
        SubsystemLoop loop = new SubsystemLoop(() -> 0L, keepRunning::get);
        AtomicInteger firstCalls = new AtomicInteger();
        loop.add(new SubsystemLifecycle() {
            @Override public void periodic() {
                firstCalls.incrementAndGet();
                keepRunning.set(false);
            }
        });
        Hooks second = loop.add(new Hooks());
        Hooks third = loop.add(new Hooks());
        loop.update();
        loop.update();
        assertEquals(1, firstCalls.get());
        assertEquals(0, second.periodic);
        assertEquals(0, third.periodic);
    }

    @Test
    public void fifoEndMarkerPreservesCallbacksAcrossCommandBudgets() {
        AtomicBoolean running = new AtomicBoolean(true);
        SubsystemLoop loop = new SubsystemLoop(() -> 0L, running::get);
        Hooks hooks = loop.add(new Hooks());
        AtomicInteger callbacks = new AtomicInteger();
        AtomicInteger forbiddenCallbacks = new AtomicInteger();
        for (int i = 0; i < 130; i++) {
            loop.enqueue(callbacks::incrementAndGet);
        }
        loop.enqueue(() -> running.set(false));
        loop.enqueue(forbiddenCallbacks::incrementAndGet);

        loop.update();
        assertEquals(64, callbacks.get());
        assertTrue(running.get());
        assertEquals(1, hooks.periodic);
        assertEquals(0, forbiddenCallbacks.get());

        loop.update();
        assertEquals(128, callbacks.get());
        assertTrue(running.get());
        assertEquals(2, hooks.periodic);
        assertEquals(0, forbiddenCallbacks.get());

        loop.update();
        assertEquals(130, callbacks.get());
        assertFalse(running.get());
        assertEquals(0, forbiddenCallbacks.get());
        assertEquals(2, hooks.periodic);
    }

    @Test(timeout = 8000)
    public void commandThreadCanClearActiveAndQueuedLegacyFlows() throws Exception {
        SubsystemLoop loop = new SubsystemLoop();
        AtomicInteger forbidden = new AtomicInteger();
        AtomicInteger ordinary = new AtomicInteger();
        loop.start(ActionSequence.create().waitUntil(() -> false).run(forbidden::incrementAndGet));
        assertFalse(loop.isIdle());
        loop.update();
        loop.start(ActionSequence.create().run(forbidden::incrementAndGet));
        loop.enqueue(ordinary::incrementAndGet);
        assertNull(onOtherThread(loop::clearFlows));
        assertFalse("ordinary queued calls remain", loop.isIdle());
        loop.update();
        assertEquals(0, forbidden.get());
        assertEquals(1, ordinary.get());
        assertTrue(loop.isIdle());
    }

    private static Throwable onOtherThread(Runnable action) throws InterruptedException {
        AtomicReference<Throwable> failure = new AtomicReference<>();
        Thread thread = new Thread(() -> {
            try {
                action.run();
            } catch (Throwable t) {
                failure.set(t);
            }
        }, "subsystem-test-non-owner");
        thread.start();
        thread.join(5000);
        assertFalse("worker did not finish", thread.isAlive());
        return failure.get();
    }

    private static final class Hooks implements SubsystemLifecycle {
        int periodic;
        int shutdown;

        @Override public void periodic() { periodic++; }
        @Override public void onShutdown() { shutdown++; }
    }
}
