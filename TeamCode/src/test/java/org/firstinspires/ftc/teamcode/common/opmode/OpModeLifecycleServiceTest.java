package org.firstinspires.ftc.teamcode.common.opmode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.common.network.OpModeLifecycleService;
import org.junit.Test;

import java.util.Arrays;
import java.util.List;

public class OpModeLifecycleServiceTest {

    private static final class FakeBackend implements OpModeLifecycleService.Backend {
        final OpModeLifecycleService service;
        int initCalls;
        int startCalls;
        int stopCalls;

        FakeBackend(OpModeLifecycleService service) {
            this.service = service;
        }

        @Override
        public List<OpModeLifecycleService.Descriptor> listAutonomous() {
            return Arrays.asList(
                    new OpModeLifecycleService.Descriptor("Auto A", "Auto"),
                    new OpModeLifecycleService.Descriptor("Auto B", "Auto"));
        }

        @Override
        public OpModeLifecycleService.BackendResult init(
                String name,
                OpModeLifecycleService.Snapshot snapshot) {
            initCalls++;
            if (!"Auto A".equals(name) && !"Auto B".equals(name)) {
                return OpModeLifecycleService.BackendResult.rejected(
                        "opmode_not_found", "not found");
            }
            service.publishInit(name);
            return OpModeLifecycleService.BackendResult.accepted();
        }

        @Override
        public OpModeLifecycleService.BackendResult start(
                String name,
                OpModeLifecycleService.Snapshot snapshot) {
            startCalls++;
            if (snapshot.phase != OpModeLifecycleService.Phase.INIT
                    || !name.equals(snapshot.activeName)) {
                return OpModeLifecycleService.BackendResult.rejected(
                        "opmode_not_initialized", "wrong state");
            }
            service.publishRunning(name);
            return OpModeLifecycleService.BackendResult.accepted();
        }

        @Override
        public OpModeLifecycleService.BackendResult stop(
                String name,
                OpModeLifecycleService.Snapshot snapshot) {
            stopCalls++;
            if (snapshot.phase != OpModeLifecycleService.Phase.STOPPED
                    && !name.equals(snapshot.activeName)) {
                return OpModeLifecycleService.BackendResult.rejected(
                        "opmode_mismatch", "wrong opmode");
            }
            service.publishStopped();
            return OpModeLifecycleService.BackendResult.accepted();
        }
    }

    @Test
    public void lifecycleFollowsSdkPublishedState() {
        OpModeLifecycleService service = new OpModeLifecycleService();
        FakeBackend backend = new FakeBackend(service);
        service.attach(backend);

        OpModeLifecycleService.Snapshot stopped = service.snapshot();
        assertEquals(OpModeLifecycleService.Phase.STOPPED, stopped.phase);
        assertTrue(stopped.controllerAvailable);

        assertTrue(service.init("Auto A", stopped.revision).accepted);
        OpModeLifecycleService.Snapshot init = service.snapshot();
        assertEquals(OpModeLifecycleService.Phase.INIT, init.phase);
        assertEquals("Auto A", init.activeName);

        assertTrue(service.start("Auto A", init.revision).accepted);
        OpModeLifecycleService.Snapshot running = service.snapshot();
        assertEquals(OpModeLifecycleService.Phase.RUNNING, running.phase);

        assertTrue(service.stop("Auto A", running.revision).accepted);
        OpModeLifecycleService.Snapshot finalState = service.snapshot();
        assertEquals(OpModeLifecycleService.Phase.STOPPED, finalState.phase);
        assertEquals(null, finalState.activeName);
    }

    @Test
    public void staleRevisionIsRejectedBeforeBackendRuns() {
        OpModeLifecycleService service = new OpModeLifecycleService();
        FakeBackend backend = new FakeBackend(service);
        service.attach(backend);
        long staleRevision = service.snapshot().revision;

        // Simulate Driver Station or another control surface changing the robot.
        service.publishInit("Auto B");

        OpModeLifecycleService.ActionResult result =
                service.init("Auto A", staleRevision);

        assertFalse(result.accepted);
        assertEquals("state_changed", result.code);
        assertEquals(0, backend.initCalls);
        assertEquals("Auto B", service.snapshot().activeName);
    }

    @Test
    public void wrongActiveNameCannotBeStartedOrStopped() {
        OpModeLifecycleService service = new OpModeLifecycleService();
        FakeBackend backend = new FakeBackend(service);
        service.attach(backend);
        service.publishInit("Auto B");

        OpModeLifecycleService.Snapshot init = service.snapshot();
        OpModeLifecycleService.ActionResult start =
                service.start("Auto A", init.revision);
        assertFalse(start.accepted);
        assertEquals("opmode_not_initialized", start.code);

        OpModeLifecycleService.ActionResult stop =
                service.stop("Auto A", service.snapshot().revision);
        assertFalse(stop.accepted);
        assertEquals("opmode_mismatch", stop.code);
        assertEquals(OpModeLifecycleService.Phase.INIT, service.snapshot().phase);
    }

    @Test
    public void detachedControllerRejectsActions() {
        OpModeLifecycleService service = new OpModeLifecycleService();
        FakeBackend backend = new FakeBackend(service);
        service.attach(backend);
        service.detach(backend);

        OpModeLifecycleService.Snapshot snapshot = service.snapshot();
        assertFalse(snapshot.controllerAvailable);

        OpModeLifecycleService.ActionResult result =
                service.init("Auto A", snapshot.revision);
        assertFalse(result.accepted);
        assertEquals("controller_unavailable", result.code);
    }
}
