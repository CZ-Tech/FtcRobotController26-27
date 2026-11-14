package org.firstinspires.ftc.teamcode.common.network;

import org.json.JSONObject;
import org.junit.Test;

import static org.junit.Assert.*;

public class AutoConfigRegistryTest {
    static final class ConfigRoot {
        @AutoConfig("测试")
        public static final class Auto {
            public static volatile double deceleration = 30;
            public static volatile int samples = 240;
            public static volatile boolean enabled = true;
            public static volatile String label = "default";
            public static final int FIXED = 10;
            private static int hidden = 9;
        }
        public static class NotAnnotated {
            public static double ignored = 1;
        }
    }

    @Test
    public void discoversOnlyAnnotatedMutableFieldsAndResetsToCapturedDefaults() throws Exception {
        AutoConfigRegistry registry = new AutoConfigRegistry(ConfigRoot.class);
        JSONObject before = new JSONObject(registry.snapshotJson());
        assertEquals(4, before.getJSONArray("fields").length());
        assertFalse(before.toString().contains("FIXED"));
        assertFalse(before.toString().contains("hidden"));
        assertFalse(before.toString().contains("ignored"));

        registry.patch(1, new JSONObject()
                .put("Auto.deceleration", 35.5)
                .put("Auto.samples", 300)
                .put("Auto.enabled", false)
                .put("Auto.label", "test"));
        assertEquals(35.5, ConfigRoot.Auto.deceleration, 0);
        assertEquals(300, ConfigRoot.Auto.samples);
        assertFalse(ConfigRoot.Auto.enabled);
        assertEquals("test", ConfigRoot.Auto.label);
        assertEquals(2, registry.revision());

        registry.reset(2);
        assertEquals(30, ConfigRoot.Auto.deceleration, 0);
        assertEquals(240, ConfigRoot.Auto.samples);
        assertTrue(ConfigRoot.Auto.enabled);
        assertEquals("default", ConfigRoot.Auto.label);
    }

    @Test
    public void validationAndRevisionGuardNeverPartiallyApply() throws Exception {
        ConfigRoot.Auto.deceleration = 30;
        AutoConfigRegistry registry = new AutoConfigRegistry(ConfigRoot.class);
        try {
            registry.patch(1, new JSONObject()
                    .put("Auto.deceleration", 40)
                    .put("Auto.samples", 2.5));
            fail("fractional int must be rejected");
        } catch (IllegalArgumentException expected) { }
        assertEquals(30, ConfigRoot.Auto.deceleration, 0);
        assertEquals(1, registry.revision());

        try {
            registry.patch(0, new JSONObject().put("Auto.deceleration", 40));
            fail("stale revision must be rejected");
        } catch (AutoConfigRegistry.RevisionConflict expected) {
            assertEquals(1, expected.actual);
        }
        assertEquals(30, ConfigRoot.Auto.deceleration, 0);
    }
}
