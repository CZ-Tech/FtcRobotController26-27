package org.firstinspires.ftc.teamcode.common.network;

import java.util.Collections;
import java.util.List;

/**
 * Data-only request crossing from a network worker to the robot control thread.
 * This type must never contain Runnable, Robot, hardware, or subsystem references.
 */
public final class ControlRequest {
    public enum Type {
        EXECUTE_SAVED_PATH,
        EXECUTE_INLINE_PATH,
        RUN_COMMAND
    }

    public final long id;
    public final Type type;
    public final String pathName;
    public final String inlineJson;
    public final String commandName;
    public final List<Object> commandArgs;
    public final long createdAtMs;

    private ControlRequest(long id,
                           Type type,
                           String pathName,
                           String inlineJson,
                           String commandName,
                           List<Object> commandArgs,
                           long createdAtMs) {
        this.id = id;
        this.type = type;
        this.pathName = pathName;
        this.inlineJson = inlineJson;
        this.commandName = commandName;
        this.commandArgs = commandArgs == null
                ? Collections.emptyList()
                : Collections.unmodifiableList(commandArgs);
        this.createdAtMs = createdAtMs;
    }

    static ControlRequest saved(long id, String pathName) {
        return new ControlRequest(
                id, Type.EXECUTE_SAVED_PATH, pathName, null, null, null, now());
    }

    static ControlRequest inline(long id, String json) {
        return new ControlRequest(
                id, Type.EXECUTE_INLINE_PATH, null, json, null, null, now());
    }

    static ControlRequest command(long id, String name, List<Object> args) {
        return new ControlRequest(
                id, Type.RUN_COMMAND, null, null, name, args, now());
    }

    private static long now() {
        return android.os.SystemClock.elapsedRealtime();
    }
}
