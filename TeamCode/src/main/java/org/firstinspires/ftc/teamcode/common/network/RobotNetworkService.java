package org.firstinspires.ftc.teamcode.common.network;

import android.content.Context;
import android.util.Log;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
import org.firstinspires.ftc.teamcode.common.opmode.FtcOpModeBridge;
import org.firstinspires.ftc.teamcode.common.opmode.RouteOpModeRegistrar;

/**
 * Process-wide owner of the hardware-free V2 network stack.
 *
 * <p>The service owns no Robot, OpMode, subsystem, or hardware object. OpModes only
 * interact with its data stores/gate from their main loop.</p>
 */
public final class RobotNetworkService {
    private static final String TAG = "RobotNetworkService";
    private static volatile RobotNetworkV2 instance;
    private static FtcEventLoop eventLoop;
    private static FtcOpModeBridge opModeBridge;

    private RobotNetworkService() {}

    @OpModeRegistrar
    public static synchronized void init(Context context, OpModeManager ignored) {
        if (instance != null) return;

        RobotNetworkV2 network = new RobotNetworkV2(context.getApplicationContext());
        network.routes.setChangeListener(() -> RouteOpModeRegistrar.onRoutesChanged(network.opModes));
        try {
            network.start();
            instance = network;
            attachOpModeBridgeIfReady();
            Log.i(TAG, "Network V2 listening on port " + RobotNetworkV2.DEFAULT_PORT);
        } catch (Exception e) {
            Log.e(TAG, "Failed to start Network V2", e);
            throw new RuntimeException("Failed to start Network V2", e);
        }
    }

    @OnCreateEventLoop
    public static synchronized void attachEventLoop(Context context, FtcEventLoop value) {
        eventLoop = value;
        attachOpModeBridgeIfReady();
    }

    public static RobotNetworkV2 get() {
        RobotNetworkV2 value = instance;
        if (value == null) {
            throw new IllegalStateException("RobotNetworkService has not been initialized");
        }
        return value;
    }

    private static void attachOpModeBridgeIfReady() {
        if (instance == null || eventLoop == null) return;
        if (opModeBridge != null) {
            opModeBridge.close();
            opModeBridge = null;
        }
        opModeBridge = new FtcOpModeBridge(eventLoop, instance.opModes);
    }
}
