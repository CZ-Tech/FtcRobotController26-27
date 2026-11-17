package org.firstinspires.ftc.teamcode.common.network;

import static org.junit.Assert.*;

import org.firstinspires.ftc.teamcode.common.util.Param;
import org.junit.Test;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Queue;
import java.util.function.BooleanSupplier;

public class SubsystemCommandBindingTest {
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
    @Test public void productionCatalogPreservesActionsWithoutConstructingHardware() {
        List<CommandCatalog.Descriptor> descriptors = SubsystemCommandBinding.subsystemCommands().descriptors();
        List<String> names = new ArrayList<>();
        for (CommandCatalog.Descriptor descriptor : descriptors) names.add(descriptor.name);
        assertEquals(Arrays.asList("auto.stopAll", "intaker.intakerArmStop", "intaker.intakerIn",
                "intaker.setIntakePower", "intaker.setIntakerPower", "puller.down", "puller.setState",
                "puller.store", "puller.up", "shooter.setPower", "shooter.setTargetVelocity",
                "shooter.setValveClose", "shooter.setValveOpen", "shooter.setValvePos",
                "shooter.stopShooter", "shooter.updatePIDF"), names);
        assertEquals(Arrays.asList("power", "time"), find(descriptors, "intaker.setIntakerPower").parameterNames);
        assertEquals(Arrays.asList("shooterRPM"), find(descriptors, "shooter.setTargetVelocity").parameterNames);
        assertNotNull(find(descriptors, "intaker.intakerIn").typesForArity(0));
        assertNotNull(find(descriptors, "intaker.intakerIn").typesForArity(1));
        assertNull(find(descriptors, "shooter.getMotorRPM"));
        assertNull(find(descriptors, "shooter.onShutdown"));
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
    }
    @Test public void sameArityOverloadsAreNotPublished() {
        List<CommandCatalog.Descriptor> descriptors = SubsystemCommandBinding.scan(AmbiguousHost.class).descriptors();
        assertEquals(1, descriptors.size());
        assertEquals("ambiguous.ok", descriptors.get(0).name);
        assertNull(find(descriptors, "ambiguous.spin"));
    }
    @Test public void submitDefersExecutionUntilDrain() {
        Host host = new Host(); Pending pending = new Pending();
        assertTrue(pending.bind(host).submit("fake.spin", new Object[]{0.5}));
        assertTrue(host.fake.powers.isEmpty());
        pending.drain();
        assertEquals(Collections.singletonList(0.5), host.fake.powers);
    }
    @Test public void sameSubsystemCommandsArePreservedInFifoOrder() {
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
    @Test public void invalidAndAmbiguousDefaultsAreNotPublished() {
        List<CommandCatalog.Descriptor> descriptors = SubsystemCommandBinding.scan(ShooterHost.class).descriptors();
        assertNull(find(descriptors, "shooter.aim"));
        assertNull(find(descriptors, "shooter.broken"));
        assertNull(find(descriptors, "shooter.badDefault"));
        assertEquals(1, descriptors.size());
    }
}
