package org.firstinspires.ftc.teamcode.common.subsystem;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.common.network.CommandCatalog;
import org.firstinspires.ftc.teamcode.common.network.SubsystemCommandBinding;
import org.firstinspires.ftc.teamcode.common.util.Param;
import org.junit.Test;

import java.lang.reflect.Method;
import java.util.Arrays;

/**
 * End-to-end checks of the code {@code ScheduleProcessor} generates for a real
 * {@code @GenerateSchedule} class (the test source set runs the processor too).
 */
public class GeneratedSchedulerTest {
    public static final class Host {
        public final FixtureGun target = new FixtureGun();
        public final FixtureGunScheduler gun = new FixtureGunScheduler(target);
    }

    @Test
    public void schedulerMethodBehavesLikeScheduleThenExecute() {
        Host host = new Host();

        FixtureGunSchedule handle = host.gun.shoot(3, 0.5);

        assertTrue("it is an executed schedule", handle.isExecuted());
        assertEquals("hardware is only touched by update()", 0, host.target.count);

        host.gun.update();
        assertEquals(3, host.target.count);
        assertEquals(0.5, host.target.power, 0.0);
    }

    @Test
    public void returnedHandleCancelsThePublishedSchedule() {
        Host host = new Host();

        FixtureGunSchedule handle = host.gun.shoot(3, 0.5);
        handle.cancel();
        host.gun.update();

        assertTrue(handle.isCancelled());
        assertEquals(0, host.target.count);
    }

    @Test
    public void latestCallWinsLikeAnyOtherExecute() {
        Host host = new Host();

        FixtureGunSchedule first = host.gun.shoot(1, 0.1);
        host.gun.shoot(2, 0.2);
        host.gun.update();

        assertTrue(first.isCancelled());
        assertEquals(2, host.target.count);
    }

    @Test
    public void replacingAnActiveScheduleRunsTheCancelHook() {
        Host host = new Host();

        host.gun.schedule().shoot(1, 0.1).waitUntil(() -> false).execute();
        host.gun.update();
        assertEquals(0, host.target.cancelledCount);

        host.gun.stop();
        host.gun.update();

        assertEquals(1, host.target.cancelledCount);
        assertEquals(0, host.target.count);
    }

    @Test
    public void scheduledActionWithReturnValueIsInTheChainAndHasAShortcut() {
        Host host = new Host();

        host.gun.schedule().boost(2).execute();
        host.gun.update();
        host.gun.boost(5);
        host.gun.update();

        assertEquals(7, host.target.boosted);
    }

    @Test
    public void directMethodsStillRunImmediately() {
        assertEquals(42, new Host().gun.rpm(), 0.0);
    }

    @Test
    public void generatedInterfaceCarriesNamesAndDefaults() throws Exception {
        Method shoot = FixtureGunSchedule.class.getMethod("shoot", int.class, double.class);
        Param count = (Param) shoot.getParameterAnnotations()[0][0];
        Param power = (Param) shoot.getParameterAnnotations()[1][0];

        assertEquals("count", count.value());
        assertEquals(Param.NO_DEFAULT, count.defaultValue());
        assertEquals("power", power.value());
        assertEquals("0.8", power.defaultValue());
    }

    @Test
    public void generatedSubsystemWorksAsRemoteCommandHost() {
        Host host = new Host();
        SubsystemCommandBinding.Commands commands = SubsystemCommandBinding.scan(Host.class);
        CommandCatalog.Descriptor shoot = null;
        for (CommandCatalog.Descriptor descriptor : commands.descriptors()) {
            if (descriptor.name.equals("gun.shoot")) shoot = descriptor;
        }

        assertEquals(Arrays.asList("count", "power"), shoot.parameterNames);
        assertEquals(Arrays.asList(null, "0.8"), shoot.parameterDefaults);
        assertEquals(Arrays.asList("int"), shoot.typesForArity(1));

        SubsystemCommandBinding binding = new SubsystemCommandBinding(host, commands);
        assertTrue(binding.submit("gun.shoot", new Object[]{7}));
        host.gun.update();

        assertEquals(7, host.target.count);
        assertEquals(0.8, host.target.power, 0.0);
        assertFalse(binding.submit("gun.rpm", new Object[0]));
    }
}
