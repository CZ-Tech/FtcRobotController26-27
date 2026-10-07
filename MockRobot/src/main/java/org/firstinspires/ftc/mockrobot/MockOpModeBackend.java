package org.firstinspires.ftc.mockrobot;

import org.firstinspires.ftc.teamcode.common.network.ExecutionStateStore;
import org.firstinspires.ftc.teamcode.common.network.OpModeLifecycleService;

import java.util.ArrayList;
import java.util.List;

/** Immediate desktop implementation of the same lifecycle backend used by FtcOpModeBridge. */
public final class MockOpModeBackend implements OpModeLifecycleService.Backend {
    private final OpModeLifecycleService service;
    private final MockSimulationEngine simulation;
    private final ExecutionStateStore execution;
    private final MockRouteStore routes;

    public MockOpModeBackend(
            OpModeLifecycleService service,
            MockSimulationEngine simulation,
            ExecutionStateStore execution,
            MockRouteStore routes) {
        this.service = service;
        this.simulation = simulation;
        this.execution = execution;
        this.routes = routes;
    }

    @Override
    public List<OpModeLifecycleService.Descriptor> listAutonomous() {
        List<OpModeLifecycleService.Descriptor> result = new ArrayList<>();
        for (MockOpModeProfile profile : profiles()) {
            result.add(new OpModeLifecycleService.Descriptor(profile.name, profile.group));
        }
        return result;
    }

    @Override
    public synchronized OpModeLifecycleService.BackendResult init(
            String name,
            OpModeLifecycleService.Snapshot snapshot) {
        MockOpModeProfile profile = profile(name);
        if (profile == null) {
            return OpModeLifecycleService.BackendResult.rejected(
                    "opmode_not_found",
                    "No simulated Autonomous OpMode named " + name);
        }
        if (snapshot.phase == OpModeLifecycleService.Phase.INIT
                && name.equals(snapshot.activeName)) {
            return OpModeLifecycleService.BackendResult.accepted();
        }
        if (snapshot.phase != OpModeLifecycleService.Phase.STOPPED) {
            return OpModeLifecycleService.BackendResult.rejected(
                    "opmode_not_stopped",
                    "Another OpMode is already initialized or running");
        }
        execution.publish(ExecutionStateStore.State.NOT_READY, 0, null);
        service.publishInit(name);
        simulation.onInit(profile);
        MockLog.info("OpMode", "INIT " + name);
        return OpModeLifecycleService.BackendResult.accepted();
    }

    @Override
    public synchronized OpModeLifecycleService.BackendResult start(
            String name,
            OpModeLifecycleService.Snapshot snapshot) {
        MockOpModeProfile profile = profile(name);
        if (profile == null) {
            return OpModeLifecycleService.BackendResult.rejected(
                    "opmode_not_found",
                    "No simulated Autonomous OpMode named " + name);
        }
        if (snapshot.phase != OpModeLifecycleService.Phase.INIT
                || !name.equals(snapshot.activeName)) {
            return OpModeLifecycleService.BackendResult.rejected(
                    "opmode_not_initialized",
                    "Requested OpMode is not the currently initialized OpMode");
        }
        service.publishRunning(name);
        simulation.onStart(profile);
        MockLog.info("OpMode", "START " + name);
        return OpModeLifecycleService.BackendResult.accepted();
    }

    @Override
    public synchronized OpModeLifecycleService.BackendResult stop(
            String name,
            OpModeLifecycleService.Snapshot snapshot) {
        if (snapshot.phase == OpModeLifecycleService.Phase.STOPPED) {
            return OpModeLifecycleService.BackendResult.accepted();
        }
        if (snapshot.activeName == null || !snapshot.activeName.equals(name)) {
            return OpModeLifecycleService.BackendResult.rejected(
                    "opmode_mismatch",
                    "Requested OpMode is not the active OpMode");
        }
        stopNow();
        return OpModeLifecycleService.BackendResult.accepted();
    }

    public synchronized boolean externalInit(String name) {
        OpModeLifecycleService.Snapshot snapshot = service.snapshot();
        return init(name, snapshot).accepted;
    }

    public synchronized boolean externalStart() {
        OpModeLifecycleService.Snapshot snapshot = service.snapshot();
        if (snapshot.activeName == null) return false;
        return start(snapshot.activeName, snapshot).accepted;
    }

    public synchronized void externalStop() {
        stopNow();
    }

    public MockOpModeProfile profile(String name) {
        if (name == null || routes.get(name) == null) return null;
        return new MockOpModeProfile(name, "AzConductor", name, true);
    }

    public List<MockOpModeProfile> profiles() {
        List<MockOpModeProfile> result = new ArrayList<>();
        for (org.firstinspires.ftc.teamcode.common.network.RouteRepository.Entry route : routes.list()) {
            result.add(new MockOpModeProfile(route.name, "AzConductor", route.name, true));
        }
        return result;
    }

    private void stopNow() {
        simulation.onStop();
        execution.publish(ExecutionStateStore.State.NOT_READY, 0, null);
        service.publishStopped();
        MockLog.info("OpMode", "STOP");
    }
}
