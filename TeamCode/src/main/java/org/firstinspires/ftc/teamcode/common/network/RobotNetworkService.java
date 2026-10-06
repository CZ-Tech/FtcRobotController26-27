package org.firstinspires.ftc.teamcode.common.network;

import android.content.Context;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

/**
 * Process-wide owner of the hardware-free V2 network stack.
 *
 * <p>The service owns no Robot, OpMode, subsystem, or hardware object. OpModes only
 * interact with its data stores/gate from their main loop.</p>
 */
public final class RobotNetworkService {
    private static final String TAG = "RobotNetworkService";
    private static volatile RobotNetworkV2 instance;

    private RobotNetworkService() {}

    @OpModeRegistrar
    public static synchronized void init(Context context, OpModeManager ignored) {
        if (instance != null) return;

        RobotNetworkV2 network = new RobotNetworkV2(context.getApplicationContext());
        try {
            network.start();
            instance = network;
            Log.i(TAG, "Network V2 listening on port " + RobotNetworkV2.DEFAULT_PORT);
        } catch (Exception e) {
            Log.e(TAG, "Failed to start Network V2", e);
            throw new RuntimeException("Failed to start Network V2", e);
        }
    }

    public static RobotNetworkV2 get() {
        RobotNetworkV2 value = instance;
        if (value == null) {
            throw new IllegalStateException("RobotNetworkService has not been initialized");
        }
        return value;
    }
}
