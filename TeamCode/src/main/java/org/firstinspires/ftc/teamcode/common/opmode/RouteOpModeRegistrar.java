package org.firstinspires.ftc.teamcode.common.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.robotcore.internal.network.NetworkConnectionHandler;
import org.firstinspires.ftc.robotcore.internal.network.RobotCoreCommandList;
import org.firstinspires.ftc.robotcore.internal.opmode.InstanceOpModeRegistrar;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.robotcore.internal.opmode.RegisteredOpModes;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.common.network.OpModeLifecycleService;
import org.firstinspires.ftc.teamcode.common.network.RouteRepository;
import org.firstinspires.ftc.teamcode.common.network.RouteStore;
import org.firstinspires.ftc.teamcode.opmode.auto.RouteAutoOpMode;

/** Registers every persisted AzConductor route as a real FTC Autonomous OpMode. */
public final class RouteOpModeRegistrar {
    private static final Object LOCK = new Object();
    private static boolean installed;
    private static boolean refreshPending;
    private static boolean refreshWatcherRunning;

    private static final InstanceOpModeRegistrar INSTANCE_REGISTRAR = manager -> {
        RouteStore store = new RouteStore(AppUtil.getDefContext());
        for (RouteRepository.Entry route : store.list()) {
            OpModeMeta meta = new OpModeMeta.Builder()
                    .setName(route.name)
                    .setGroup("AzConductor")
                    .setFlavor(OpModeMeta.Flavor.AUTONOMOUS)
                    .setSource(OpModeMeta.Source.ANDROID_STUDIO)
                    .setDescription("AzConductor route")
                    .build();
            manager.register(meta, new RouteAutoOpMode(route.name));
        }
    };

    private RouteOpModeRegistrar() {}

    @OpModeRegistrar
    public static void install(OpModeManager ignored) {
        synchronized (LOCK) {
            if (installed) return;
            installed = true;
            RegisteredOpModes registered = RegisteredOpModes.getInstance();
            registered.addInstanceOpModeRegistrar(INSTANCE_REGISTRAR);
            // Initial registration pass reaches @OpModeRegistrar after instance registrars,
            // so invoke one instance refresh immediately to expose persisted routes on the DS.
            registered.registerInstanceOpModes();
        }
    }

    /** Called after the route repository changes. Refresh immediately only while stopped. */
    public static void onRoutesChanged(OpModeLifecycleService lifecycle) {
        synchronized (LOCK) {
            if (lifecycle.snapshot().phase != OpModeLifecycleService.Phase.STOPPED) {
                refreshPending = true;
                startRefreshWatcher(lifecycle);
                return;
            }
        }
        refreshNow();
    }

    /** Flushes a deferred route-list refresh once the active OpMode has stopped. */
    public static void flushPendingIfStopped(OpModeLifecycleService lifecycle) {
        synchronized (LOCK) {
            if (!refreshPending
                    || lifecycle.snapshot().phase != OpModeLifecycleService.Phase.STOPPED) {
                return;
            }
            refreshPending = false;
        }
        refreshNow();
    }

    private static void startRefreshWatcher(OpModeLifecycleService lifecycle) {
        if (refreshWatcherRunning) return;
        refreshWatcherRunning = true;
        Thread watcher = new Thread(() -> {
            try {
                while (true) {
                    synchronized (LOCK) {
                        if (!refreshPending) return;
                    }
                    if (lifecycle.snapshot().phase == OpModeLifecycleService.Phase.STOPPED) {
                        flushPendingIfStopped(lifecycle);
                        return;
                    }
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                        return;
                    }
                }
            } finally {
                synchronized (LOCK) {
                    refreshWatcherRunning = false;
                }
            }
        }, "RouteOpModeRefresh");
        watcher.setDaemon(true);
        watcher.start();
    }

    private static void refreshNow() {
        RegisteredOpModes registered = RegisteredOpModes.getInstance();
        registered.registerInstanceOpModes();
        String opModeList = SimpleGson.getInstance().toJson(registered.getOpModes());
        NetworkConnectionHandler.getInstance().sendCommand(
                new Command(RobotCoreCommandList.CMD_NOTIFY_OP_MODE_LIST, opModeList));
    }
}
