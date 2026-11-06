package org.firstinspires.ftc.teamcode.common.network;

import android.content.Context;
import android.util.Log;

import org.firstinspires.ftc.teamcode.common.network.http.PemTls;
import org.firstinspires.ftc.teamcode.common.network.http.RobotHttpServer;

import java.io.ByteArrayOutputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.security.GeneralSecurityException;

/**
 * Hardware-free composition root for the new network stack.
 *
 * <p>OpModes publish {@link #runtime} and {@link #execution}, and may attach a
 * {@link #commandDispatcher} sink to accept remote commands.</p>
 */
public final class RobotNetworkV2 {
    private static final String TAG = "RobotNetworkV2";
    public static final int DEFAULT_PORT = 8888;
    public static final int HTTPS_PORT = 8889;
    private static final String ASSET_DIR = "ftc_cert";

    public final SessionLease session = new SessionLease();
    public final RouteStore routes;
    public final ExecutionStateStore execution = new ExecutionStateStore();
    public final RobotRuntimeStore runtime = new RobotRuntimeStore();
    public final CommandCatalog commands = new CommandCatalog();
    public final OpModeLifecycleService opModes = new OpModeLifecycleService();
    /** An OpMode that runs remote commands attaches a sink here while it is RUNNING. */
    public final CommandDispatcher commandDispatcher = new CommandDispatcher();

    private final RobotHttpServer server;

    public RobotNetworkV2(Context context) {
        this(context, DEFAULT_PORT);
    }

    public RobotNetworkV2(Context context, int port) {
        routes = new RouteStore(context);
        try {
            commands.publish(SubsystemCommandBinding.subsystemCommands().descriptors());
        } catch (RuntimeException | LinkageError e) {
            Log.e(TAG, "Failed to publish subsystem commands", e);
        }
        RobotApiRouter router = new RobotApiRouter(
                session, routes, execution, runtime, commands, opModes, commandDispatcher);
        server = new RobotHttpServer(port, router);
        enableTlsIfCertificateBundled(context);
    }

    /**
     * HTTPS is optional: the certificate is bundled into the APK by the fetchCert Gradle task.
     * If it is missing or unusable the robot keeps serving plain HTTP.
     */
    private void enableTlsIfCertificateBundled(Context context) {
        try {
            String chain = readAsset(context, ASSET_DIR + "/fullchain.pem");
            String key = readAsset(context, ASSET_DIR + "/privkey.pem");
            PemTls.Material material = PemTls.load(chain, key);
            server.enableTls(HTTPS_PORT, material.context);
            Log.i(TAG, "HTTPS enabled on port " + HTTPS_PORT + ", certificate "
                    + material.leaf.getSubjectX500Principal().getName()
                    + " valid until " + material.leaf.getNotAfter());
        } catch (FileNotFoundException e) {
            Log.w(TAG, "No bundled TLS certificate (" + ASSET_DIR + "); HTTPS disabled");
        } catch (IOException | GeneralSecurityException | RuntimeException e) {
            Log.e(TAG, "Bundled TLS certificate is unusable; HTTPS disabled", e);
        }
    }

    private static String readAsset(Context context, String path) throws IOException {
        try (InputStream in = context.getAssets().open(path)) {
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            byte[] buffer = new byte[4096];
            int n;
            while ((n = in.read(buffer)) > 0) out.write(buffer, 0, n);
            return new String(out.toByteArray(), StandardCharsets.UTF_8);
        }
    }

    public void start() throws IOException {
        server.start();
    }

    public void stop() {
        server.stop();
    }

    public boolean isRunning() {
        return server.isRunning();
    }
}
