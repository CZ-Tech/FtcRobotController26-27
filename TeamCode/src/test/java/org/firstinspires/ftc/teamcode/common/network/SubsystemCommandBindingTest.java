package org.firstinspires.ftc.teamcode.common.network;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.common.subsystem.IntakerSchedule;
import org.firstinspires.ftc.teamcode.common.subsystem.ScheduleApi;
import org.firstinspires.ftc.teamcode.common.subsystem.Scheduler;
import org.firstinspires.ftc.teamcode.common.util.Param;
import org.junit.Test;

import java.lang.reflect.Method;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.BooleanSupplier;

public class SubsystemCommandBindingTest {
    public static class Target {
        double power = -1;
        int spinCalls;
        boolean flag;
        char mode;

        public void spin(double p) {
            power = p;
            spinCalls++;
        }

        public void spin() {
            power = 1;
            spinCalls++;
        }

        public void configure(boolean on, char m) {
            flag = on;
            mode = m;
        }

        public void gate(BooleanSupplier condition) {}
    }

    public interface FakeSchedule extends ScheduleApi<FakeSchedule> {
        FakeSchedule spin(@Param("power") double power);

        FakeSchedule spin();

        FakeSchedule configure(@Param("on") boolean on, @Param("mode") char mode);

        /** Unsupported parameter type: must not be published. */
        FakeSchedule gate(@Param("condition") BooleanSupplier condition);
    }

    public static final class FakeScheduler extends Scheduler<FakeSchedule> {
        FakeScheduler(Target target) {
            super(FakeSchedule.class, target);
        }
    }

    public static final class Host {
        public final Target target = new Target();
        public final FakeScheduler fake = new FakeScheduler(target);
    }

    public static class Ambiguous {
        public void spin(double p) {}

        public void spin(int p) {}

        public void ok() {}
    }

    public interface AmbiguousSchedule extends ScheduleApi<AmbiguousSchedule> {
        AmbiguousSchedule spin(@Param("power") double power);

        AmbiguousSchedule spin(@Param("power") int power);

        AmbiguousSchedule ok();
    }

    public static final class AmbiguousScheduler extends Scheduler<AmbiguousSchedule> {
        AmbiguousScheduler() {
            super(AmbiguousSchedule.class, new Ambiguous());
        }
    }

    public static final class AmbiguousHost {
        public final AmbiguousScheduler ambiguous = new AmbiguousScheduler();
    }

    public static class Shooter {
        int count;
        double power;
        boolean fast;
        int calls;

        public void shoot(int c, double p, boolean f) {
            count = c;
            power = p;
            fast = f;
            calls++;
        }

        public void aim(double angle) {}

        public void aim() {}

        public void broken(double a, double b) {}

        public void badDefault(int a) {}
    }

    public interface ShooterSchedule extends ScheduleApi<ShooterSchedule> {
        ShooterSchedule shoot(
                @Param("count") int count,
                @Param(value = "power", defaultValue = "0.8") double power,
                @Param(defaultValue = "true") boolean fast);

        /** Default overlaps the no-argument overload below: ambiguous, not published. */
        ShooterSchedule aim(@Param(value = "angle", defaultValue = "0") double angle);

        ShooterSchedule aim();

        /** A required parameter after a defaulted one cannot be addressed positionally. */
        ShooterSchedule broken(
                @Param(value = "a", defaultValue = "1") double a,
                @Param("b") double b);

        ShooterSchedule badDefault(@Param(value = "a", defaultValue = "x") int a);
    }

    public static final class ShooterScheduler extends Scheduler<ShooterSchedule> {
        ShooterScheduler(Shooter target) {
            super(ShooterSchedule.class, target);
        }
    }

    public static final class ShooterHost {
        public final Shooter shooter = new Shooter();
        public final ShooterScheduler scheduler = new ShooterScheduler(shooter);
    }

    private static CommandCatalog.Descriptor find(
            List<CommandCatalog.Descriptor> descriptors, String name) {
        for (CommandCatalog.Descriptor descriptor : descriptors) {
            if (descriptor.name.equals(name)) return descriptor;
        }
        return null;
    }

    @Test
    public void describesCommandsFromScheduleInterface() {
        List<CommandCatalog.Descriptor> descriptors =
                SubsystemCommandBinding.scan(Host.class).descriptors();

        assertEquals(2, descriptors.size());
        assertEquals("fake.configure", descriptors.get(0).name);
        assertEquals("fake.spin", descriptors.get(1).name);
        assertNull("unsupported parameter types are not published",
                find(descriptors, "fake.gate"));

        CommandCatalog.Descriptor configure = descriptors.get(0);
        assertEquals(Arrays.asList("on", "mode"), configure.parameterNames);
        assertEquals(Arrays.asList("boolean", "char"), configure.parameterTypes);
    }

    @Test
    public void overloadsBecomeOneCommandListingTheLongestSignature() {
        CommandCatalog.Descriptor spin = find(
                SubsystemCommandBinding.scan(Host.class).descriptors(), "fake.spin");

        assertEquals(Collections.singletonList("power"), spin.parameterNames);
        assertEquals(Collections.singletonList("double"), spin.parameterTypes);
        assertEquals(Collections.emptyList(), spin.typesForArity(0));
        assertEquals(Collections.singletonList("double"), spin.typesForArity(1));
        assertNull(spin.typesForArity(2));
    }

    @Test
    public void overloadsWithTheSameArgumentCountAreNotPublished() {
        List<CommandCatalog.Descriptor> descriptors =
                SubsystemCommandBinding.scan(AmbiguousHost.class).descriptors();

        assertNull(find(descriptors, "ambiguous.spin"));
        assertEquals(1, descriptors.size());
        assertEquals("ambiguous.ok", descriptors.get(0).name);
    }

    @Test
    public void submitOnlyPublishesTheScheduleUntilTheSchedulerTicks() {
        Host host = new Host();
        SubsystemCommandBinding binding = new SubsystemCommandBinding(
                host, SubsystemCommandBinding.scan(Host.class));

        assertTrue(binding.submit("fake.spin", new Object[]{0.5}));
        assertEquals("nothing may run on the submitting thread", 0, host.target.spinCalls);

        host.fake.update();
        assertEquals(1, host.target.spinCalls);
        assertEquals(0.5, host.target.power, 0.0);
    }

    @Test
    public void overloadIsChosenByArgumentCount() {
        Host host = new Host();
        SubsystemCommandBinding binding = new SubsystemCommandBinding(
                host, SubsystemCommandBinding.scan(Host.class));

        assertTrue(binding.submit("fake.spin", new Object[0]));
        host.fake.update();
        assertEquals(1.0, host.target.power, 0.0);

        assertTrue(binding.submit("fake.spin", new Object[]{0.25}));
        host.fake.update();
        assertEquals(0.25, host.target.power, 0.0);
    }

    @Test
    public void convertedPrimitivesReachTheMethod() {
        Host host = new Host();
        SubsystemCommandBinding binding = new SubsystemCommandBinding(
                host, SubsystemCommandBinding.scan(Host.class));

        assertTrue(binding.submit("fake.configure", new Object[]{true, 'x'}));
        host.fake.update();
        assertTrue(host.target.flag);
        assertEquals('x', host.target.mode);
    }

    @Test
    public void unknownCommandOrArityIsRefused() {
        Host host = new Host();
        SubsystemCommandBinding binding = new SubsystemCommandBinding(
                host, SubsystemCommandBinding.scan(Host.class));

        assertFalse(binding.submit("fake.missing", new Object[0]));
        assertFalse(binding.submit("fake.spin", new Object[]{1.0, 2.0}));
        assertFalse(binding.submit("fake.gate", new Object[]{null}));
    }

    @Test
    public void latestCommandReplacesThePendingOneOfTheSameSubsystem() {
        Host host = new Host();
        SubsystemCommandBinding binding = new SubsystemCommandBinding(
                host, SubsystemCommandBinding.scan(Host.class));

        assertTrue(binding.submit("fake.spin", new Object[]{0.1}));
        assertTrue(binding.submit("fake.spin", new Object[]{0.9}));
        host.fake.update();

        assertEquals(1, host.target.spinCalls);
        assertEquals(0.9, host.target.power, 0.0);
    }

    @Test
    public void shutdownDiscardsPendingCommands() {
        Host host = new Host();
        SubsystemCommandBinding binding = new SubsystemCommandBinding(
                host, SubsystemCommandBinding.scan(Host.class));

        assertTrue(binding.submit("fake.spin", new Object[]{0.9}));
        host.fake.cancelSchedule();
        host.fake.update();

        assertEquals(0, host.target.spinCalls);
    }

    @Test
    public void defaultsMakeTrailingParametersOptionalAndAreListed() {
        List<CommandCatalog.Descriptor> descriptors =
                SubsystemCommandBinding.scan(ShooterHost.class).descriptors();
        CommandCatalog.Descriptor shoot = find(descriptors, "scheduler.shoot");

        assertEquals(Arrays.asList("count", "power", "arg2"), shoot.parameterNames);
        assertEquals(Arrays.asList("int", "double", "boolean"), shoot.parameterTypes);
        assertEquals(Arrays.asList(null, "0.8", "true"), shoot.parameterDefaults);

        assertNull(shoot.typesForArity(0));
        assertEquals(Arrays.asList("int"), shoot.typesForArity(1));
        assertEquals(Arrays.asList("int", "double"), shoot.typesForArity(2));
        assertEquals(Arrays.asList("int", "double", "boolean"), shoot.typesForArity(3));
        assertNull(shoot.typesForArity(4));
    }

    @Test
    public void omittedArgumentsAreFilledFromDefaults() {
        ShooterHost host = new ShooterHost();
        SubsystemCommandBinding binding = new SubsystemCommandBinding(
                host, SubsystemCommandBinding.scan(ShooterHost.class));

        assertTrue(binding.submit("scheduler.shoot", new Object[]{4}));
        host.scheduler.update();
        assertEquals(4, host.shooter.count);
        assertEquals(0.8, host.shooter.power, 0.0);
        assertTrue(host.shooter.fast);

        assertTrue(binding.submit("scheduler.shoot", new Object[]{5, 0.3}));
        host.scheduler.update();
        assertEquals(0.3, host.shooter.power, 0.0);
        assertTrue(host.shooter.fast);

        assertTrue(binding.submit("scheduler.shoot", new Object[]{6, 0.1, false}));
        host.scheduler.update();
        assertEquals(6, host.shooter.count);
        assertFalse(host.shooter.fast);

        // The required parameter has no default.
        assertFalse(binding.submit("scheduler.shoot", new Object[0]));
    }

    @Test
    public void invalidOrAmbiguousDefaultsAreNotPublished() {
        List<CommandCatalog.Descriptor> descriptors =
                SubsystemCommandBinding.scan(ShooterHost.class).descriptors();

        assertNull("default overlaps another overload", find(descriptors, "scheduler.aim"));
        assertNull("required parameter after a default", find(descriptors, "scheduler.broken"));
        assertNull("default does not convert", find(descriptors, "scheduler.badDefault"));
        assertEquals(1, descriptors.size());
    }

    @Test
    public void descriptorsWithoutDefaultsListNullDefaults() {
        CommandCatalog.Descriptor configure = find(
                SubsystemCommandBinding.scan(Host.class).descriptors(), "fake.configure");

        assertEquals(Arrays.asList(null, null), configure.parameterDefaults);
    }

    @Test
    public void realSubsystemPublishesIntakerCommandsWithNames() throws Exception {
        List<CommandCatalog.Descriptor> descriptors =
                SubsystemCommandBinding.subsystemCommands().descriptors();

        CommandCatalog.Descriptor in = find(descriptors, "intaker.intakerIn");
        assertEquals(Collections.singletonList("power"), in.parameterNames);
        assertEquals(Collections.singletonList("double"), in.parameterTypes);
        assertEquals(Collections.emptyList(), in.typesForArity(0));

        CommandCatalog.Descriptor stop = find(descriptors, "intaker.intakerArmStop");
        assertEquals(Collections.emptyList(), stop.parameterNames);

        // Generated interface carries the names the catalog reads.
        Method method = IntakerSchedule.class.getMethod("intakerIn", double.class);
        assertEquals("power", ((Param) method.getParameterAnnotations()[0][0]).value());
        assertNull(find(descriptors, "intaker.update"));
    }
}
