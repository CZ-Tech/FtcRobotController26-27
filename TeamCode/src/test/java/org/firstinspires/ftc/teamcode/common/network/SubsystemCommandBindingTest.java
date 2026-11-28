package org.firstinspires.ftc.teamcode.common.network;

import static org.junit.Assert.*;

import org.firstinspires.ftc.teamcode.common.subsystem.CommandTasks;
import org.firstinspires.ftc.teamcode.common.util.Param;
import org.junit.Test;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;

public class SubsystemCommandBindingTest {
    private static final long TEST_TIMEOUT_MILLIS = 8000;
    private static final long WAIT_MILLIS = 2000;
    public static class Target {
        final List<Double> powers = new ArrayList<>();
        boolean flag;
        char mode;
        @RemoteCommand public void spin(@Param("power") double power) { powers.add(power); }
        @RemoteCommand public void spin() { powers.add(1.0); }
        @RemoteCommand public void configure(@Param("on") boolean on, @Param("mode") char mode) {
            flag = on;
            this.mode = mode;
        }
        @RemoteCommand public void gate(BooleanSupplier condition) {}
        @RemoteCommand public static void staticCommand() {}
        @RemoteCommand public int nonVoid() { return 1; }
        @RemoteCommand private void hidden() {}
        public void unannotated() {}
    }
    public static class Host {
        public final Target fake = new Target();
        public static final Target ignored = new Target();
        private final Target hidden = new Target();
    }
    public static class Ambiguous {
        @RemoteCommand public void spin(double power) {}
        @RemoteCommand public void spin(int power) {}
        @RemoteCommand public void ok() {}
    }
    public static class AmbiguousHost { public final Ambiguous ambiguous = new Ambiguous(); }
    public static class Shooter {
        int count;
        double power;
        boolean fast;
        int calls;
        @RemoteCommand public void shoot(@Param("count") int count,
                @Param(value = "power", defaultValue = "0.8") double power,
                @Param(defaultValue = "true") boolean fast) {
            this.count = count;
            this.power = power;
            this.fast = fast;
            calls++;
        }
        @RemoteCommand public void aim(@Param(defaultValue = "0") double angle) {}
        @RemoteCommand public void aim() {}
        @RemoteCommand public void broken(@Param(defaultValue = "1") double a, double b) {}
        @RemoteCommand public void badDefault(@Param(defaultValue = "x") int a) {}
    }
    public static class ShooterHost { public final Shooter shooter = new Shooter(); }
    public static class OverloadShapes {
        @RemoteCommand public void chain(@Param("count") int count) {}
        @RemoteCommand public void chain(@Param("count") int count, @Param("power") double power) {}
        @RemoteCommand public void chain(@Param("count") int count, @Param("power") double power,
                                        @Param("fast") boolean fast) {}
        @RemoteCommand public void gap(@Param("count") int count) {}
        @RemoteCommand public void gap(@Param("count") int count, @Param("power") double power,
                                      @Param("fast") boolean fast) {}
        @RemoteCommand public void renamed(@Param("speed") double speed) {}
        @RemoteCommand public void renamed(@Param("power") double power, @Param("time") long time) {}
        @RemoteCommand public void typed(@Param("value") int value) {}
        @RemoteCommand public void typed(@Param("value") String value, @Param("on") boolean on) {}
    }
    public static class ShapesHost { public final OverloadShapes shapes = new OverloadShapes(); }

    public static class DirectSubsystem {
        final List<String> steps = Collections.synchronizedList(new ArrayList<>());
        final List<Thread> threads = Collections.synchronizedList(new ArrayList<>());

        // Deliberately not a RemoteCommand: an aggregate calls this as ordinary Java.
        public void write(String step) {
            steps.add(step);
            threads.add(Thread.currentThread());
        }
    }
    public static class Aggregate {
        final DirectSubsystem subsystem;
        final CountDownLatch waiting = new CountDownLatch(1);
        final CountDownLatch release = new CountDownLatch(1);
        final AtomicReference<Thread> worker = new AtomicReference<>();

        Aggregate(DirectSubsystem subsystem) { this.subsystem = subsystem; }

        @RemoteCommand public void aggregate() throws InterruptedException {
            worker.set(Thread.currentThread());
            subsystem.write("aggregate");
            subsystem.write("before");
            waitForRelease();
            subsystem.write("after");
        }
        private void waitForRelease() throws InterruptedException {
            subsystem.write("wait");
            waiting.countDown();
            release.await();
            subsystem.write("resumed");
        }
        @RemoteCommand public void tail() { subsystem.write("tail"); }
    }
    public static class WorkerHost {
        public final DirectSubsystem subsystem = new DirectSubsystem();
        public final Aggregate auto = new Aggregate(subsystem);
    }
    public static class ParallelTarget {
        final CountDownLatch bothStarted = new CountDownLatch(2);
        final CountDownLatch release = new CountDownLatch(1);
        final List<Thread> workers = Collections.synchronizedList(new ArrayList<>());
        final AtomicInteger completed = new AtomicInteger();

        @RemoteCommand public void hold() throws InterruptedException {
            workers.add(Thread.currentThread());
            bothStarted.countDown();
            release.await();
            completed.incrementAndGet();
        }
    }
    public static class ParallelHost { public final ParallelTarget parallel = new ParallelTarget(); }

    private static class Pending {
        final Queue<Runnable> queue = new ArrayDeque<>();
        boolean closed;
        void enqueue(Runnable command) {
            if (closed) throw new IllegalStateException("closed");
            queue.add(command);
        }
        void drain() { while (!queue.isEmpty()) queue.remove().run(); }
        SubsystemCommandBinding bind(Object host) {
            return new SubsystemCommandBinding(host, SubsystemCommandBinding.scan(host.getClass()), this::enqueue);
        }
    }
    private static CommandCatalog.Descriptor find(List<CommandCatalog.Descriptor> descriptors, String name) {
        for (CommandCatalog.Descriptor descriptor : descriptors) if (descriptor.name.equals(name)) return descriptor;
        return null;
    }

    @Test public void scansOnlyPublicInstanceAnnotatedVoidMethods() {
        List<CommandCatalog.Descriptor> descriptors = SubsystemCommandBinding.scan(Host.class).descriptors();
        assertEquals(2, descriptors.size());
        assertEquals("fake.configure", descriptors.get(0).name);
        assertEquals("fake.spin", descriptors.get(1).name);
        assertEquals(Arrays.asList("on", "mode"), descriptors.get(0).parameterNames);
        assertEquals(Arrays.asList("boolean", "char"), descriptors.get(0).parameterTypes);
        assertEquals(Arrays.asList(null, null), descriptors.get(0).parameterDefaults);
    }
    @Test public void overloadsListLongestSignatureAndAcceptedArities() {
        CommandCatalog.Descriptor spin = find(SubsystemCommandBinding.scan(Host.class).descriptors(), "fake.spin");
        assertEquals(Collections.singletonList("power"), spin.parameterNames);
        assertEquals(Collections.singletonList("double"), spin.parameterTypes);
        assertEquals(Collections.emptyList(), spin.typesForArity(0));
        assertEquals(Collections.singletonList("double"), spin.typesForArity(1));
        assertNull(spin.typesForArity(2));
        assertEquals(Arrays.asList(0, 1), spin.acceptedArgCounts);
        assertTrue(spin.hasUnifiedSignature);
        assertEquals(Integer.valueOf(0), spin.requiredParamCount);
        assertEquals(Collections.singletonList(false), spin.parameterRequired);
        assertEquals(Collections.singletonList(null), spin.parameterDefaults);
        assertEquals(Collections.emptyList(), spin.signatures.get(0).parameterNames);
        assertEquals(Collections.singletonList("power"), spin.signatures.get(1).parameterNames);
    }
    @Test public void contiguousOverloadsInferRequiredPrefixAndOptionalTail() {
        CommandCatalog.Descriptor chain = find(
                SubsystemCommandBinding.scan(ShapesHost.class).descriptors(), "shapes.chain");
        assertEquals(Arrays.asList(1, 2, 3), chain.acceptedArgCounts);
        assertTrue(chain.hasUnifiedSignature);
        assertEquals(Integer.valueOf(1), chain.requiredParamCount);
        assertEquals(Arrays.asList(true, false, false), chain.parameterRequired);
        assertEquals(Arrays.asList(null, null, null), chain.parameterDefaults);
    }
    @Test public void nonContiguousOrDifferentOverloadsKeepTheirOwnSignatures() {
        List<CommandCatalog.Descriptor> descriptors = SubsystemCommandBinding.scan(ShapesHost.class).descriptors();
        for (String name : Arrays.asList("shapes.gap", "shapes.renamed", "shapes.typed")) {
            CommandCatalog.Descriptor command = find(descriptors, name);
            assertFalse(name, command.hasUnifiedSignature);
            assertNull(command.requiredParamCount);
            assertNull(command.parameterRequired);
            assertEquals(2, command.signatures.size());
        }
        CommandCatalog.Descriptor gap = find(descriptors, "shapes.gap");
        assertEquals(Arrays.asList(1, 3), gap.acceptedArgCounts);
        assertNull(gap.typesForArity(2));
        CommandCatalog.Descriptor renamed = find(descriptors, "shapes.renamed");
        assertEquals(Collections.singletonList("speed"), renamed.signatures.get(0).parameterNames);
        assertEquals(Arrays.asList("power", "time"), renamed.signatures.get(1).parameterNames);
        CommandCatalog.Descriptor typed = find(descriptors, "shapes.typed");
        assertEquals(Collections.singletonList("int"), typed.typesForArity(1));
        assertEquals(Arrays.asList("java.lang.String", "boolean"), typed.typesForArity(2));
    }
    @Test public void sameArityOverloadsAreNotPublished() {
        List<CommandCatalog.Descriptor> descriptors = SubsystemCommandBinding.scan(AmbiguousHost.class).descriptors();
        assertEquals(1, descriptors.size());
        assertEquals("ambiguous.ok", descriptors.get(0).name);
        assertNull(find(descriptors, "ambiguous.spin"));
    }
    @Test public void injectedExecutorDefersExecutionUntilDrain() {
        Host host = new Host(); Pending pending = new Pending();
        assertTrue(pending.bind(host).submit("fake.spin", new Object[]{0.5}));
        assertTrue(host.fake.powers.isEmpty());
        pending.drain();
        assertEquals(Collections.singletonList(0.5), host.fake.powers);
    }
    @Test public void injectedExecutorControlsDispatchOrder() {
        Host host = new Host(); Pending pending = new Pending();
        SubsystemCommandBinding binding = pending.bind(host);
        assertTrue(binding.submit("fake.spin", new Object[]{0.1}));
        assertTrue(binding.submit("fake.spin", null));
        assertTrue(binding.submit("fake.spin", new Object[]{0.9}));
        assertEquals(3, pending.queue.size());
        pending.drain();
        assertEquals(Arrays.asList(0.1, 1.0, 0.9), host.fake.powers);
    }
    @Test public void submitCopiesCallerArguments() {
        Host host = new Host(); Pending pending = new Pending();
        Object[] args = {0.25};
        assertTrue(pending.bind(host).submit("fake.spin", args));
        args[0] = 0.99;
        pending.drain();
        assertEquals(Collections.singletonList(0.25), host.fake.powers);
    }
    @Test public void convertedPrimitivesReachMethodOnlyAtDrain() {
        Host host = new Host(); Pending pending = new Pending();
        assertTrue(pending.bind(host).submit("fake.configure", new Object[]{true, 'x'}));
        assertFalse(host.fake.flag);
        pending.drain();
        assertTrue(host.fake.flag);
        assertEquals('x', host.fake.mode);
    }
    @Test public void unknownCommandsAndInvalidAritiesAreRefused() {
        Pending pending = new Pending(); SubsystemCommandBinding binding = pending.bind(new Host());
        assertFalse(binding.submit("fake.missing", null));
        assertFalse(binding.submit("fake.spin", new Object[]{1.0, 2.0}));
        assertFalse(binding.submit("fake.gate", new Object[]{null}));
        assertTrue(pending.queue.isEmpty());
    }
    @Test public void closedQueueRejectsWithoutExecuting() {
        Host host = new Host(); Pending pending = new Pending(); pending.closed = true;
        assertFalse(pending.bind(host).submit("fake.spin", new Object[]{0.9}));
        assertTrue(host.fake.powers.isEmpty());
        assertTrue(pending.queue.isEmpty());
    }
    @Test public void defaultsAreListedWithAcceptedArities() {
        CommandCatalog.Descriptor shoot = find(SubsystemCommandBinding.scan(ShooterHost.class).descriptors(), "shooter.shoot");
        assertEquals(Arrays.asList("count", "power", "arg2"), shoot.parameterNames);
        assertEquals(Arrays.asList("int", "double", "boolean"), shoot.parameterTypes);
        assertEquals(Arrays.asList(null, "0.8", "true"), shoot.parameterDefaults);
        assertNull(shoot.typesForArity(0));
        assertEquals(Collections.singletonList("int"), shoot.typesForArity(1));
        assertEquals(Arrays.asList("int", "double"), shoot.typesForArity(2));
        assertEquals(Arrays.asList("int", "double", "boolean"), shoot.typesForArity(3));
        assertNull(shoot.typesForArity(4));
        assertTrue(shoot.hasUnifiedSignature);
        assertEquals(Integer.valueOf(1), shoot.requiredParamCount);
        assertEquals(Arrays.asList(true, false, false), shoot.parameterRequired);
        assertEquals(Arrays.asList(1, 2, 3), shoot.acceptedArgCounts);
        assertEquals(Collections.singletonList("count"), shoot.signatures.get(0).parameterNames);
        assertEquals(Arrays.asList(null, "0.8"), shoot.signatures.get(1).parameterDefaults);
    }
    @Test public void omittedArgumentsAreFilledFromDefaults() {
        ShooterHost host = new ShooterHost(); Pending pending = new Pending(); SubsystemCommandBinding binding = pending.bind(host);
        assertTrue(binding.submit("shooter.shoot", new Object[]{4}));
        assertEquals(0, host.shooter.calls);
        pending.drain();
        assertEquals(4, host.shooter.count); assertEquals(0.8, host.shooter.power, 0); assertTrue(host.shooter.fast);
        assertTrue(binding.submit("shooter.shoot", new Object[]{5, 0.3})); pending.drain();
        assertEquals(0.3, host.shooter.power, 0); assertTrue(host.shooter.fast);
        assertTrue(binding.submit("shooter.shoot", new Object[]{6, 0.1, false})); pending.drain();
        assertEquals(6, host.shooter.count); assertFalse(host.shooter.fast); assertEquals(3, host.shooter.calls);
        assertFalse(binding.submit("shooter.shoot", new Object[0]));
    }
    @Test(timeout = TEST_TIMEOUT_MILLIS)
    public void submitAllValidatesEveryCallBeforeSubmittingAnything() {
        Host host = new Host();
        Pending pending = new Pending();
        SubsystemCommandBinding binding = pending.bind(host);
        CommandDispatcher.Call valid = new CommandDispatcher.Call("fake.spin", new Object[]{0.25});
        List<CommandDispatcher.Call> invalidCalls = Arrays.asList(
                new CommandDispatcher.Call("fake.missing", new Object[0]),
                new CommandDispatcher.Call("fake.spin", new Object[]{0.1, 0.2}),
                null);
        for (CommandDispatcher.Call invalid : invalidCalls) {
            assertFalse(binding.submitAll(Arrays.asList(valid, invalid)));
            assertTrue("a bad later call must prevent all submission", pending.queue.isEmpty());
            assertTrue(host.fake.powers.isEmpty());
        }
        assertFalse(binding.submitAll(null));
        assertTrue(binding.submitAll(Collections.emptyList()));
        assertTrue(pending.queue.isEmpty());
    }

    @Test(timeout = TEST_TIMEOUT_MILLIS)
    public void submitAllSubmitsExactlyOneRunnableForTheWholeGroup() {
        Host host = new Host();
        Pending pending = new Pending();
        SubsystemCommandBinding binding = pending.bind(host);
        assertTrue(binding.submitAll(Arrays.asList(
                new CommandDispatcher.Call("fake.spin", new Object[]{0.1}),
                new CommandDispatcher.Call("fake.spin", new Object[0]),
                new CommandDispatcher.Call("fake.spin", new Object[]{0.9}))));
        assertEquals(1, pending.queue.size());
        assertTrue(host.fake.powers.isEmpty());
        pending.drain();
        assertEquals(Arrays.asList(0.1, 1.0, 0.9), host.fake.powers);
        assertTrue(pending.queue.isEmpty());
    }

    @Test(timeout = TEST_TIMEOUT_MILLIS)
    public void submitAllCapturesCallArgumentsAndListBeforeExecution() {
        Host host = new Host();
        Pending pending = new Pending();
        SubsystemCommandBinding binding = pending.bind(host);
        Object[] original = {0.25};
        CommandDispatcher.Call first = new CommandDispatcher.Call("fake.spin", original);
        original[0] = 0.99;
        Object[] exposed = first.arguments();
        exposed[0] = 0.88;
        List<CommandDispatcher.Call> calls = new ArrayList<>(Arrays.asList(first,
                new CommandDispatcher.Call("fake.spin", new Object[]{0.75})));
        assertTrue(binding.submitAll(calls));
        original[0] = 0.66;
        first.arguments()[0] = 0.55;
        calls.clear();
        calls.add(new CommandDispatcher.Call("fake.spin", new Object[]{0.44}));
        assertEquals(1, pending.queue.size());
        assertTrue(host.fake.powers.isEmpty());
        pending.drain();
        assertEquals(Arrays.asList(0.25, 0.75), host.fake.powers);
    }

    @Test(timeout = TEST_TIMEOUT_MILLIS)
    public void aggregateDirectCallsAndWaitStayOnOneWorkerWithoutControlLoop() throws Exception {
        WorkerHost host = new WorkerHost();
        CommandTasks tasks = new CommandTasks(() -> true);
        SubsystemCommandBinding binding = new SubsystemCommandBinding(
                host, SubsystemCommandBinding.scan(WorkerHost.class), tasks::execute);
        try {
            assertTrue(binding.submitAll(Arrays.asList(
                    new CommandDispatcher.Call("auto.aggregate", new Object[0]),
                    new CommandDispatcher.Call("auto.tail", new Object[0]))));
            await(host.auto.waiting);
            Thread worker = host.auto.worker.get();
            assertNotSame(Thread.currentThread(), worker);
            assertTrue(worker.isDaemon());
            assertFalse(tasks.isIdle());
            assertEquals(Arrays.asList("aggregate", "before", "wait"), host.subsystem.steps);
            host.auto.release.countDown();
            join(worker);
            tasks.throwIfFailed();
            assertEquals(Arrays.asList("aggregate", "before", "wait", "resumed", "after", "tail"),
                    host.subsystem.steps);
            assertEquals(6, host.subsystem.threads.size());
            for (Thread execution : host.subsystem.threads) assertSame(worker, execution);
            assertTrue(tasks.isIdle());
        } finally {
            host.auto.release.countDown();
            tasks.shutdown();
        }
    }

    @Test(timeout = TEST_TIMEOUT_MILLIS)
    public void differentSubmissionsCanRunConcurrentlyOnManagedWorkers() throws Exception {
        ParallelHost host = new ParallelHost();
        CommandTasks tasks = new CommandTasks(() -> true);
        SubsystemCommandBinding binding = new SubsystemCommandBinding(
                host, SubsystemCommandBinding.scan(ParallelHost.class), tasks::execute);
        try {
            assertTrue(binding.submit("parallel.hold", new Object[0]));
            assertTrue(binding.submit("parallel.hold", new Object[0]));
            // Both methods must be active before either is allowed to finish.
            await(host.parallel.bothStarted);
            List<Thread> workers = new ArrayList<>(host.parallel.workers);
            assertEquals(2, workers.size());
            assertNotSame(workers.get(0), workers.get(1));
            for (Thread worker : workers) {
                assertNotSame(Thread.currentThread(), worker);
                assertTrue(worker.isDaemon());
            }
            assertEquals(0, host.parallel.completed.get());
            host.parallel.release.countDown();
            for (Thread worker : workers) join(worker);
            tasks.throwIfFailed();
            assertEquals(2, host.parallel.completed.get());
            assertTrue(tasks.isIdle());
        } finally {
            host.parallel.release.countDown();
            tasks.shutdown();
        }
    }

    @Test(timeout = TEST_TIMEOUT_MILLIS)
    public void reflectedInterruptedExceptionCancelsWithoutTailOrRecordedFailure() throws Exception {
        WorkerHost host = new WorkerHost();
        CommandTasks tasks = new CommandTasks(() -> true);
        AtomicBoolean interruptRestored = new AtomicBoolean();
        SubsystemCommandBinding binding = new SubsystemCommandBinding(
                host, SubsystemCommandBinding.scan(WorkerHost.class), invocation -> tasks.execute(() -> {
                    try {
                        invocation.run();
                    } finally {
                        interruptRestored.set(Thread.currentThread().isInterrupted());
                    }
                }));
        try {
            assertTrue(binding.submitAll(Arrays.asList(
                    new CommandDispatcher.Call("auto.aggregate", new Object[0]),
                    new CommandDispatcher.Call("auto.tail", new Object[0]))));
            await(host.auto.waiting);
            Thread worker = host.auto.worker.get();
            // Do not cancel through the owner: its cancelled flag would suppress even a
            // wrongly wrapped failure and hide a regression in reflection unwrapping.
            worker.interrupt();
            join(worker);
            assertTrue("reflection must restore the interrupted flag", interruptRestored.get());
            assertEquals(Arrays.asList("aggregate", "before", "wait"), host.subsystem.steps);
            assertTrue(tasks.isIdle());
            tasks.throwIfFailed();
        } finally {
            host.auto.release.countDown();
            tasks.shutdown();
        }
    }

    private static void await(CountDownLatch latch) throws InterruptedException {
        assertTrue("timed out waiting for command rendezvous",
                latch.await(WAIT_MILLIS, TimeUnit.MILLISECONDS));
    }

    private static void join(Thread worker) throws InterruptedException {
        assertNotNull("command worker did not start", worker);
        worker.join(WAIT_MILLIS);
        assertFalse("command worker did not finish: " + worker.getName(), worker.isAlive());
    }

    @Test public void invalidAndAmbiguousDefaultsAreNotPublished() {
        List<CommandCatalog.Descriptor> descriptors = SubsystemCommandBinding.scan(ShooterHost.class).descriptors();
        assertNull(find(descriptors, "shooter.aim"));
        assertNull(find(descriptors, "shooter.broken"));
        assertNull(find(descriptors, "shooter.badDefault"));
        assertEquals(1, descriptors.size());
    }
}
