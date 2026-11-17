package org.firstinspires.ftc.teamcode.common.subsystem;

import org.junit.Test;

import java.util.concurrent.CancellationException;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static org.junit.Assert.*;

public class CommandTasksTest {
    private static final long TEST_TIMEOUT_MILLIS = 8000;
    private static final long WAIT_MILLIS = 2000;

    @Test(timeout = TEST_TIMEOUT_MILLIS)
    public void defaultExecutionUsesManagedDaemonInsteadOfCallingThread() throws Exception {
        CommandTasks tasks = new CommandTasks(() -> true);
        CountDownLatch started = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        AtomicReference<Thread> worker = new AtomicReference<>();
        try {
            assertTrue(tasks.isIdle());
            tasks.execute(() -> {
                worker.set(Thread.currentThread());
                started.countDown();
                awaitOrCancel(release);
            });
            await(started);
            assertNotSame(Thread.currentThread(), worker.get());
            assertTrue(worker.get().isDaemon());
            assertFalse(tasks.isIdle());
            release.countDown();
            join(worker.get());
            assertTrue(tasks.isIdle());
            tasks.throwIfFailed();
        } finally {
            release.countDown();
            tasks.shutdown();
        }
    }

    @Test(timeout = TEST_TIMEOUT_MILLIS)
    public void twoSubmissionsCanRunConcurrentlyOnDifferentThreads() throws Exception {
        CommandTasks tasks = new CommandTasks(() -> true);
        CountDownLatch bothStarted = new CountDownLatch(2);
        CountDownLatch release = new CountDownLatch(1);
        AtomicReference<Thread> first = new AtomicReference<>();
        AtomicReference<Thread> second = new AtomicReference<>();
        try {
            tasks.execute(() -> {
                first.set(Thread.currentThread());
                bothStarted.countDown();
                awaitOrCancel(release);
            });
            tasks.execute(() -> {
                second.set(Thread.currentThread());
                bothStarted.countDown();
                awaitOrCancel(release);
            });
            // Neither task can finish until both have reached this rendezvous.
            await(bothStarted);
            assertNotSame(first.get(), second.get());
            assertTrue(first.get().isDaemon());
            assertTrue(second.get().isDaemon());
            assertFalse(tasks.isIdle());
            release.countDown();
            join(first.get());
            join(second.get());
            assertTrue(tasks.isIdle());
            tasks.throwIfFailed();
        } finally {
            release.countDown();
            tasks.shutdown();
        }
    }

    @Test(timeout = TEST_TIMEOUT_MILLIS)
    public void sealRejectsNewTasksWithoutCancellingActiveTask() throws Exception {
        CommandTasks tasks = new CommandTasks(() -> true);
        CountDownLatch started = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        AtomicReference<Thread> worker = new AtomicReference<>();
        AtomicBoolean tailRan = new AtomicBoolean();
        AtomicBoolean rejectedActionRan = new AtomicBoolean();
        try {
            tasks.execute(() -> {
                worker.set(Thread.currentThread());
                started.countDown();
                awaitOrCancel(release);
                CommandTasks.checkCancelled();
                tailRan.set(true);
            });
            await(started);
            tasks.seal();
            assertThrows(IllegalStateException.class,
                    () -> tasks.execute(() -> rejectedActionRan.set(true)));
            assertFalse(tasks.isIdle());
            release.countDown();
            join(worker.get());
            tasks.throwIfFailed();
            assertTrue(tailRan.get());
            assertFalse(rejectedActionRan.get());
            assertTrue(tasks.isIdle());
        } finally {
            release.countDown();
            tasks.shutdown();
        }
    }

    @Test(timeout = TEST_TIMEOUT_MILLIS)
    public void workerFailureReachesCallerAndCancelsSibling() throws Exception {
        CommandTasks tasks = new CommandTasks(() -> true);
        RuntimeException expected = new RuntimeException("worker failed");
        CountDownLatch siblingStarted = new CountDownLatch(1);
        CountDownLatch siblingInterrupted = new CountDownLatch(1);
        CountDownLatch releaseSibling = new CountDownLatch(1);
        AtomicReference<Thread> sibling = new AtomicReference<>();
        AtomicReference<Thread> failing = new AtomicReference<>();
        AtomicBoolean siblingCancelled = new AtomicBoolean();
        AtomicBoolean siblingTailRan = new AtomicBoolean();
        try {
            tasks.execute(() -> {
                sibling.set(Thread.currentThread());
                siblingStarted.countDown();
                try {
                    releaseSibling.await();
                    siblingTailRan.set(true);
                } catch (InterruptedException interrupted) {
                    siblingCancelled.set(CommandTasks.isCancelled());
                    siblingInterrupted.countDown();
                }
            });
            await(siblingStarted);
            tasks.execute(() -> {
                failing.set(Thread.currentThread());
                throw expected;
            });
            await(siblingInterrupted);
            join(sibling.get());
            join(failing.get());
            IllegalStateException reported = assertThrows(
                    IllegalStateException.class, tasks::throwIfFailed);
            assertSame(expected, reported.getCause());
            assertTrue(siblingCancelled.get());
            assertFalse(siblingTailRan.get());
            assertTrue(tasks.isIdle());
            assertThrows(IllegalStateException.class, () -> tasks.execute(() -> {}));
        } finally {
            releaseSibling.countDown();
            tasks.shutdown();
        }
    }

    @Test(timeout = TEST_TIMEOUT_MILLIS)
    public void shutdownInterruptsWaitingWorkerAndWaitsForItsCleanup() throws Exception {
        assertShutdownWaitsForCleanup(false);
    }

    @Test(timeout = TEST_TIMEOUT_MILLIS)
    public void alreadyInterruptedShutdownCallerStillCleansUpAndRestoresInterrupt() throws Exception {
        assertShutdownWaitsForCleanup(true);
    }

    @Test(timeout = TEST_TIMEOUT_MILLIS)
    public void cancelOthersCancelsSiblingButNotManagedCaller() throws Exception {
        CommandTasks tasks = new CommandTasks(() -> true);
        CountDownLatch siblingStarted = new CountDownLatch(1);
        CountDownLatch siblingInterrupted = new CountDownLatch(1);
        CountDownLatch releaseSibling = new CountDownLatch(1);
        CountDownLatch callerFinished = new CountDownLatch(1);
        AtomicReference<Thread> sibling = new AtomicReference<>();
        AtomicReference<Thread> caller = new AtomicReference<>();
        AtomicBoolean callerTailRan = new AtomicBoolean();
        AtomicBoolean callerCancelled = new AtomicBoolean(true);
        AtomicBoolean callerInterrupted = new AtomicBoolean(true);
        AtomicBoolean siblingCancelled = new AtomicBoolean();
        try {
            tasks.execute(() -> {
                sibling.set(Thread.currentThread());
                siblingStarted.countDown();
                try {
                    releaseSibling.await();
                } catch (InterruptedException interrupted) {
                    siblingCancelled.set(CommandTasks.isCancelled());
                    siblingInterrupted.countDown();
                }
            });
            await(siblingStarted);
            tasks.execute(() -> {
                caller.set(Thread.currentThread());
                try {
                    tasks.cancelOthers();
                    callerCancelled.set(CommandTasks.isCancelled());
                    callerInterrupted.set(Thread.currentThread().isInterrupted());
                    CommandTasks.checkCancelled();
                    callerTailRan.set(true);
                } finally {
                    callerFinished.countDown();
                }
            });
            await(callerFinished);
            await(siblingInterrupted);
            join(caller.get());
            join(sibling.get());
            tasks.throwIfFailed();
            assertTrue(siblingCancelled.get());
            assertFalse(callerCancelled.get());
            assertFalse(callerInterrupted.get());
            assertTrue(callerTailRan.get());
            assertTrue(tasks.isIdle());
        } finally {
            releaseSibling.countDown();
            tasks.shutdown();
        }
    }

    @Test(timeout = TEST_TIMEOUT_MILLIS)
    public void cancelledCallerCannotCancelTheTaskThatIsStoppingIt() throws Exception {
        CommandTasks tasks = new CommandTasks(() -> true);
        CountDownLatch siblingStarted = new CountDownLatch(1);
        CountDownLatch releaseSibling = new CountDownLatch(1);
        CountDownLatch attemptedRetaliation = new CountDownLatch(1);
        CountDownLatch winnerFinished = new CountDownLatch(1);
        AtomicReference<Thread> sibling = new AtomicReference<>();
        AtomicReference<Thread> winner = new AtomicReference<>();
        AtomicBoolean rejected = new AtomicBoolean();
        AtomicBoolean winnerSurvived = new AtomicBoolean();
        try {
            tasks.execute(() -> {
                sibling.set(Thread.currentThread());
                siblingStarted.countDown();
                try {
                    releaseSibling.await();
                } catch (InterruptedException swallowed) {
                    try {
                        tasks.cancelOthers();
                    } catch (CancellationException expected) {
                        rejected.set(true);
                    } finally {
                        attemptedRetaliation.countDown();
                    }
                }
            });
            await(siblingStarted);
            tasks.execute(() -> {
                winner.set(Thread.currentThread());
                try {
                    tasks.cancelOthers();
                    awaitOrCancel(attemptedRetaliation);
                    CommandTasks.checkCancelled();
                    winnerSurvived.set(true);
                } finally {
                    winnerFinished.countDown();
                }
            });
            await(winnerFinished);
            join(winner.get());
            join(sibling.get());
            tasks.throwIfFailed();
            assertTrue(rejected.get());
            assertTrue(winnerSurvived.get());
        } finally {
            releaseSibling.countDown();
            attemptedRetaliation.countDown();
            tasks.shutdown();
        }
    }

    @Test(timeout = TEST_TIMEOUT_MILLIS)
    public void checkCancelledStopsTailEvenAfterWorkerSwallowsInterrupt() throws Exception {
        CommandTasks tasks = new CommandTasks(() -> true);
        CountDownLatch started = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        AtomicReference<Thread> worker = new AtomicReference<>();
        AtomicBoolean swallowedInterrupt = new AtomicBoolean();
        AtomicBoolean interruptStillSet = new AtomicBoolean(true);
        AtomicBoolean cancelled = new AtomicBoolean();
        AtomicBoolean guardReached = new AtomicBoolean();
        AtomicBoolean tailRan = new AtomicBoolean();
        try {
            tasks.execute(() -> {
                worker.set(Thread.currentThread());
                started.countDown();
                try {
                    release.await();
                } catch (InterruptedException ignored) {
                    swallowedInterrupt.set(true);
                }
                interruptStillSet.set(Thread.currentThread().isInterrupted());
                cancelled.set(CommandTasks.isCancelled());
                guardReached.set(true);
                CommandTasks.checkCancelled();
                tailRan.set(true);
            });
            await(started);
            tasks.cancelOthers();
            join(worker.get());
            assertTrue(swallowedInterrupt.get());
            assertFalse(interruptStillSet.get());
            assertTrue(cancelled.get());
            assertTrue(guardReached.get());
            assertFalse(tailRan.get());
            assertTrue(tasks.isIdle());
            // Cancellation must not be reported as a worker failure.
            tasks.throwIfFailed();
        } finally {
            release.countDown();
            tasks.shutdown();
        }
    }

    @Test(timeout = TEST_TIMEOUT_MILLIS)
    public void keepRunningFalseRejectsSubmission() {
        CommandTasks tasks = new CommandTasks(() -> false);
        AtomicBoolean ran = new AtomicBoolean();
        try {
            assertThrows(IllegalStateException.class,
                    () -> tasks.execute(() -> ran.set(true)));
            assertFalse(ran.get());
            assertTrue(tasks.isIdle());
            tasks.throwIfFailed();
        } finally {
            tasks.shutdown();
        }
    }

    @Test(timeout = TEST_TIMEOUT_MILLIS)
    public void keepRunningBecomingFalseCancelsAtGuardWithoutInterrupt() throws Exception {
        AtomicBoolean keepRunning = new AtomicBoolean(true);
        CommandTasks tasks = new CommandTasks(keepRunning::get);
        CountDownLatch started = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        AtomicReference<Thread> worker = new AtomicReference<>();
        AtomicBoolean cancelled = new AtomicBoolean();
        AtomicBoolean interrupted = new AtomicBoolean(true);
        AtomicBoolean tailRan = new AtomicBoolean();
        try {
            tasks.execute(() -> {
                worker.set(Thread.currentThread());
                started.countDown();
                awaitOrCancel(release);
                interrupted.set(Thread.currentThread().isInterrupted());
                cancelled.set(CommandTasks.isCancelled());
                CommandTasks.checkCancelled();
                tailRan.set(true);
            });
            await(started);
            keepRunning.set(false);
            assertThrows(IllegalStateException.class, () -> tasks.execute(() -> {}));
            release.countDown();
            join(worker.get());
            assertTrue(cancelled.get());
            assertFalse(interrupted.get());
            assertFalse(tailRan.get());
            assertTrue(tasks.isIdle());
            tasks.throwIfFailed();
        } finally {
            release.countDown();
            tasks.shutdown();
        }
    }

    @Test(timeout = TEST_TIMEOUT_MILLIS)
    public void shutdownReportsNonCooperatingWorkerWithinBoundAndWorkerIsFinallyReleased()
            throws Exception {
        CommandTasks tasks = new CommandTasks(() -> true, 50);
        CountDownLatch started = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        AtomicReference<Thread> worker = new AtomicReference<>();
        try {
            tasks.execute(() -> {
                worker.set(Thread.currentThread());
                started.countDown();
                awaitIgnoringInterrupts(release);
            });
            await(started);
            long before = System.nanoTime();
            IllegalStateException failure = assertThrows(
                    IllegalStateException.class, tasks::shutdown);
            long elapsed = System.nanoTime() - before;
            assertTrue("shutdown must honor its bound rather than wait for release",
                    elapsed < TimeUnit.MILLISECONDS.toNanos(WAIT_MILLIS));
            assertTrue(failure.getMessage().contains(worker.get().getName()));
            assertTrue(worker.get().isAlive());
            assertFalse(tasks.isIdle());
            assertThrows(IllegalStateException.class, () -> tasks.execute(() -> {}));
        } finally {
            // Never leave the deliberately non-cooperating daemon behind, even on assertion failure.
            release.countDown();
            Thread thread = worker.get();
            if (thread != null) join(thread);
            tasks.shutdown();
        }
        assertTrue(tasks.isIdle());
        tasks.throwIfFailed();
    }

    private static void assertShutdownWaitsForCleanup(boolean initiallyInterrupted)
            throws Exception {
        CommandTasks tasks = new CommandTasks(() -> true, WAIT_MILLIS);
        CountDownLatch started = new CountDownLatch(1);
        CountDownLatch waiting = new CountDownLatch(1);
        CountDownLatch cleanupStarted = new CountDownLatch(1);
        CountDownLatch releaseCleanup = new CountDownLatch(1);
        CountDownLatch shutdownDone = new CountDownLatch(1);
        AtomicBoolean cleanupFinished = new AtomicBoolean();
        AtomicBoolean interruptRestored = new AtomicBoolean();
        AtomicReference<Throwable> shutdownFailure = new AtomicReference<>();
        Thread shutdownCaller = new Thread(() -> {
            if (initiallyInterrupted) Thread.currentThread().interrupt();
            try {
                tasks.shutdown();
                interruptRestored.set(Thread.currentThread().isInterrupted());
            } catch (Throwable failure) {
                shutdownFailure.set(failure);
            } finally {
                shutdownDone.countDown();
            }
        }, "command-tasks-test-shutdown");
        shutdownCaller.setDaemon(true);
        try {
            tasks.execute(() -> {
                started.countDown();
                try {
                    waiting.await();
                } catch (InterruptedException interrupted) {
                    cleanupStarted.countDown();
                    awaitIgnoringInterrupts(releaseCleanup);
                    cleanupFinished.set(true);
                }
            });
            await(started);
            shutdownCaller.start();
            await(cleanupStarted);
            // The worker is deliberately held inside cleanup: shutdown must still be joining it.
            assertFalse("shutdown returned before worker cleanup finished",
                    shutdownDone.await(100, TimeUnit.MILLISECONDS));
            releaseCleanup.countDown();
            await(shutdownDone);
            join(shutdownCaller);
            assertNull(shutdownFailure.get());
            assertTrue(cleanupFinished.get());
            assertEquals(initiallyInterrupted, interruptRestored.get());
            assertTrue(tasks.isIdle());
            tasks.throwIfFailed();
            assertThrows(IllegalStateException.class, () -> tasks.execute(() -> {}));
            tasks.shutdown();
        } finally {
            releaseCleanup.countDown();
            waiting.countDown();
            tasks.shutdown();
            if (shutdownCaller.getState() != Thread.State.NEW) join(shutdownCaller);
        }
    }

    private static void await(CountDownLatch latch) throws InterruptedException {
        assertTrue("timed out waiting for worker rendezvous",
                latch.await(WAIT_MILLIS, TimeUnit.MILLISECONDS));
    }

    private static void awaitOrCancel(CountDownLatch latch) {
        try {
            latch.await();
        } catch (InterruptedException interrupted) {
            Thread.currentThread().interrupt();
            throw new CancellationException("test worker interrupted");
        }
    }

    private static void awaitIgnoringInterrupts(CountDownLatch latch) {
        boolean interrupted = false;
        try {
            while (true) {
                try {
                    latch.await();
                    return;
                } catch (InterruptedException ignored) {
                    interrupted = true;
                }
            }
        } finally {
            if (interrupted) Thread.currentThread().interrupt();
        }
    }

    private static void join(Thread thread) throws InterruptedException {
        assertNotNull("worker did not start", thread);
        thread.join(WAIT_MILLIS);
        assertFalse("thread did not finish: " + thread.getName(), thread.isAlive());
    }
}
