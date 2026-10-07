package org.firstinspires.ftc.teamcode.common.opmode;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.robotcore.internal.opmode.RegisteredOpModes;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Adapter between FTC SDK OpMode lifecycle APIs and the hardware-free lifecycle service.
 */
public final class FtcOpModeBridge
        implements OpModeManagerImpl.Notifications, OpModeLifecycleService.Backend, AutoCloseable {
    private final FtcEventLoop eventLoop;
    private final OpModeManagerImpl manager;
    private final OpModeLifecycleService service;

    public FtcOpModeBridge(FtcEventLoop eventLoop, OpModeLifecycleService service) {
        if (eventLoop == null) throw new IllegalArgumentException("eventLoop == null");
        if (service == null) throw new IllegalArgumentException("service == null");
        this.eventLoop = eventLoop;
        this.manager = eventLoop.getOpModeManager();
        this.service = service;

        manager.registerListener(this);
        service.attach(this);

        if (isDefaultName(manager.getActiveOpModeName())) {
            service.publishStopped();
        }
    }

    @Override
    public List<OpModeLifecycleService.Descriptor> listAutonomous() {
        List<OpModeLifecycleService.Descriptor> result = new ArrayList<>();
        for (OpModeMeta meta : RegisteredOpModes.getInstance().getOpModes()) {
            if (meta.flavor == OpModeMeta.Flavor.AUTONOMOUS) {
                result.add(new OpModeLifecycleService.Descriptor(meta.name, meta.group));
            }
        }
        result.sort(Comparator
                .comparing((OpModeLifecycleService.Descriptor it) -> it.group)
                .thenComparing(it -> it.name));
        return result;
    }

    @Override
    public OpModeLifecycleService.BackendResult init(
            String name,
            OpModeLifecycleService.Snapshot snapshot) {
        OpModeMeta meta = metadata(name);
        if (meta == null || meta.flavor != OpModeMeta.Flavor.AUTONOMOUS) {
            return OpModeLifecycleService.BackendResult.rejected(
                    "opmode_not_found",
                    "No registered Autonomous OpMode named " + name);
        }

        if (snapshot.phase == OpModeLifecycleService.Phase.INIT
                && name.equals(snapshot.activeName)) {
            return OpModeLifecycleService.BackendResult.accepted();
        }

        if (snapshot.phase != OpModeLifecycleService.Phase.STOPPED
                || !isDefaultName(manager.getActiveOpModeName())) {
            return OpModeLifecycleService.BackendResult.rejected(
                    "opmode_not_stopped",
                    "Another OpMode is already initialized or running");
        }

        // The SDK applies this asynchronously on the event-loop thread and will refuse
        // the transition if another control surface leaves the default OpMode first.
        manager.initOpMode(name, true);
        return OpModeLifecycleService.BackendResult.accepted();
    }

    @Override
    public OpModeLifecycleService.BackendResult start(
            String name,
            OpModeLifecycleService.Snapshot snapshot) {
        if (snapshot.phase != OpModeLifecycleService.Phase.INIT
                || !same(name, snapshot.activeName)
                || !same(name, manager.getActiveOpModeName())) {
            return OpModeLifecycleService.BackendResult.rejected(
                    "opmode_not_initialized",
                    "Requested OpMode is not the currently initialized OpMode");
        }

        manager.startActiveOpMode();
        return OpModeLifecycleService.BackendResult.accepted();
    }

    @Override
    public OpModeLifecycleService.BackendResult stop(
            String name,
            OpModeLifecycleService.Snapshot snapshot) {
        if (snapshot.phase == OpModeLifecycleService.Phase.STOPPED) {
            return OpModeLifecycleService.BackendResult.accepted();
        }
        if (!same(name, snapshot.activeName) || !same(name, manager.getActiveOpModeName())) {
            return OpModeLifecycleService.BackendResult.rejected(
                    "opmode_mismatch",
                    "Requested OpMode is not the active OpMode");
        }

        // requestOpModeStop is explicitly safe from arbitrary threads and only stops the
        // captured OpMode instance if it is still active when the event loop consumes it.
        OpMode active = manager.getActiveOpMode();
        eventLoop.requestOpModeStop(active);
        return OpModeLifecycleService.BackendResult.accepted();
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {
        if (opMode instanceof OpModeManagerImpl.DefaultOpMode) {
            service.publishStopped();
        } else {
            service.publishInit(manager.getActiveOpModeName());
        }
    }

    @Override
    public void onOpModePreStart(OpMode opMode) {
        if (opMode instanceof OpModeManagerImpl.DefaultOpMode) {
            service.publishStopped();
        } else {
            service.publishRunning(manager.getActiveOpModeName());
        }
    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        if (!(opMode instanceof OpModeManagerImpl.DefaultOpMode)) {
            service.publishStopped();
        }
    }

    @Override
    public void close() {
        manager.unregisterListener(this);
        service.detach(this);
    }

    private static OpModeMeta metadata(String name) {
        if (name == null || name.trim().isEmpty()) return null;
        return RegisteredOpModes.getInstance().getOpModeMetadata(name);
    }

    private static boolean isDefaultName(String name) {
        return name == null
                || name.isEmpty()
                || OpModeManager.DEFAULT_OP_MODE_NAME.equals(name);
    }

    private static boolean same(String a, String b) {
        return a == null ? b == null : a.equals(b);
    }
}
