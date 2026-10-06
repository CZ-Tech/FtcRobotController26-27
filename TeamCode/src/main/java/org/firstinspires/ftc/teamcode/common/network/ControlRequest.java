package org.firstinspires.ftc.teamcode.common.network;

/**
 * Data-only request crossing from a network worker to the robot control thread.
 * This type must never contain Runnable, Robot, hardware, or subsystem references.
 */
public final class ControlRequest {
    public enum Type {
        EXECUTE_SAVED_PATH,
        EXECUTE_INLINE_PATH
    }

    public final long id;
    public final Type type;
    public final String pathName;
    public final String inlineJson;
    public final long createdAtMs;

    private ControlRequest(long id,
                           Type type,
                           String pathName,
                           String inlineJson,
                           long createdAtMs) {
        this.id = id;
        this.type = type;
        this.pathName = pathName;
        this.inlineJson = inlineJson;
        this.createdAtMs = createdAtMs;
    }

    static ControlRequest saved(long id, String pathName) {
        return new ControlRequest(
                id, Type.EXECUTE_SAVED_PATH, pathName, null, now());
    }

    static ControlRequest inline(long id, String json) {
        return new ControlRequest(
                id, Type.EXECUTE_INLINE_PATH, null, json, now());
    }

    private static long now() {
        return android.os.SystemClock.elapsedRealtime();
    }
}
