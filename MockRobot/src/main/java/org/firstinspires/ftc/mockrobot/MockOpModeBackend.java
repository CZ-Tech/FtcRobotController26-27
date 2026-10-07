package org.firstinspires.ftc.mockrobot;

import org.firstinspires.ftc.teamcode.common.network.ControlGate;
import org.firstinspires.ftc.teamcode.common.network.ExecutionStateStore;
import org.firstinspires.ftc.teamcode.common.opmode.OpModeLifecycleService;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** Immediate desktop implementation of the same lifecycle backend used by FtcOpModeBridge. */
public final class MockOpModeBackend implements OpModeLifecycleService.Backend {
    private final OpModeLifecycleService service;
    private final MockSimulationEngine simulation;
    private final ControlGate control;
    private final ExecutionStateStore execution;
    private final List<MockOpModeProfile> profiles;

    public MockOpModeBackend(
            OpModeLifecycleService service,
            MockSimulationEngine simulation,
            ControlGate control,
            ExecutionStateStore execution,
            List<MockOpModeProfile> profiles) {
        this.service = service;
        this.simulation = simulation;
        this.control = control;
        this.execution = execution;
        this.profiles = Collections.unmodifiableList(new ArrayList<>(profiles));
    }

    @Override
    public List<OpModeLifecycleService.Descriptor> listAutonomous() {
        List<OpModeLifecycleService.Descriptor> result = new ArrayList<>();
        for (MockOpModeProfile profile : profiles) {
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
        control.deactivate();
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
        if (name == null) return null;
        for (MockOpModeProfile profile : profiles) {
            if (profile.name.equals(name)) return profile;
        }
        return null;
    }

    public List<MockOpModeProfile> profiles() {
        return profiles;
    }

    private void stopNow() {
        simulation.onStop();
        control.deactivate();
        execution.publish(ExecutionStateStore.State.NOT_READY, 0, null);
        service.publishStopped();
        MockLog.info("OpMode", "STOP");
    }
}
