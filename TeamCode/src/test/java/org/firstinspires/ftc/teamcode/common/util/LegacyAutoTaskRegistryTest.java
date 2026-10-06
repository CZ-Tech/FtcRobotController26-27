package org.firstinspires.ftc.teamcode.common.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import org.junit.Test;

public class LegacyAutoTaskRegistryTest {

    private static final class Root {
        final LegacyCommands commands = new LegacyCommands();
    }

    @AutoTask
    public static final class LegacyCommands {
        int count;
        double power;
        boolean enabled;
        String label;

        public void reset() {
            count = 0;
            power = 0;
            enabled = false;
            label = null;
        }

        public void configure(int count, double power, boolean enabled, String label) {
            this.count = count;
            this.power = power;
            this.enabled = enabled;
            this.label = label;
        }
    }

    @Test
    public void scansCurrentObjectGraphAndInvokesLegacyCommand() {
        Root root = new Root();
        LegacyAutoTaskRegistry.scanObjectTree(root);

        Runnable task = LegacyAutoTaskRegistry.createCommandRunnable(
                "configure",
                new String[]{"3", "0.75", "true", "shoot"});

        assertNotNull(task);
        task.run();

        assertEquals(3, root.commands.count);
        assertEquals(0.75, root.commands.power, 1e-9);
        assertEquals(true, root.commands.enabled);
        assertEquals("shoot", root.commands.label);
    }

    @Test
    public void rescanningRebindsToNewObjectGraph() {
        Root oldRoot = new Root();
        Root newRoot = new Root();

        LegacyAutoTaskRegistry.scanObjectTree(oldRoot);
        LegacyAutoTaskRegistry.scanObjectTree(newRoot);

        Runnable task = LegacyAutoTaskRegistry.createCommandRunnable("reset", new String[0]);
        assertNotNull(task);

        newRoot.commands.count = 9;
        task.run();

        assertEquals(0, newRoot.commands.count);
    }
}
