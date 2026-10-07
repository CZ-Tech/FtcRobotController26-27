package org.firstinspires.ftc.mockrobot;

import org.firstinspires.ftc.teamcode.common.network.RouteRepository;
import org.junit.Test;

import java.nio.file.Files;
import java.nio.file.Path;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class MockRouteStoreTest {
    @Test
    public void revisionAndIfMatchSemanticsMatchRobotStore() throws Exception {
        Path dir = Files.createTempDirectory("mockrobot-route-test");
        MockRouteStore store = new MockRouteStore(dir.resolve("routes.json"));

        RouteRepository.PutResult created = store.put("A", "[]", 0L);
        assertTrue(created.updated);
        assertEquals(1, created.entry.revision);

        RouteRepository.PutResult stale = store.put("A", "[1]", 0L);
        assertTrue(stale.preconditionFailed);
        assertEquals(1, stale.entry.revision);

        RouteRepository.PutResult updated = store.put("A", "[1]", 1L);
        assertTrue(updated.updated);
        assertEquals(2, updated.entry.revision);

        assertFalse(store.delete("A", 1L));
        assertTrue(store.delete("A", 2L));
    }
}
