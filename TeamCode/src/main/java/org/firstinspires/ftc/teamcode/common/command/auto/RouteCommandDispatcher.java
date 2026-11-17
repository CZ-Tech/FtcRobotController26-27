package org.firstinspires.ftc.teamcode.common.command.auto;

import android.util.Log;

import org.firstinspires.ftc.teamcode.common.network.CommandArgs;
import org.firstinspires.ftc.teamcode.common.network.CommandCatalog;

import java.util.Collections;
import java.util.List;

/**
 * Executes the command attached to one finished route frame.
 *
 * <p>A route frame may carry {@code command} and {@code commandParams}; the loader only
 * emits them as a {@link SplineTrajectoryLoader.TrajectoryEvent}. This class is the single
 * place that resolves the name against {@code GET /api/v2/commands}, converts the string
 * parameters with the descriptor's types, and hands the typed call to a {@link Sink}
 * (on the robot: {@code SubsystemCommandBinding}).</p>
 *
 * <p><b>Fail fast.</b> Every result other than {@link Result#DISPATCHED} and
 * {@link Result#IGNORED} is fatal: the caller must abort the route immediately instead of
 * skipping the command or retrying it. Nothing here substitutes defaults or drops
 * arguments silently.</p>
 *
 * <p>The command fires when its frame completes. To run it and then dwell, attach it to
 * the arrival waypoint and follow that waypoint with a {@code {"wait": n}} frame.</p>
 */
public final class RouteCommandDispatcher {
    /** Same contract as {@code CommandDispatcher.Sink}: false means "not accepted". */
    public interface Sink {
        boolean submit(String name, Object[] args);
    }

    public enum Result {
        /** No {@code command} on this frame; nothing to do. */
        IGNORED,
        DISPATCHED,
        /** Name is not in the published catalog. */
        UNKNOWN_COMMAND,
        /** The catalog has no signature with this argument count. */
        ARG_MISMATCH,
        /** A parameter cannot be converted to the declared type. */
        BAD_ARGS,
        /** The sink refused the call (for example no matching remote method). */
        REJECTED,
        /** The sink threw; the call was not queued. */
        THREW
    }

    private static final String TAG = "RouteCommand";

    private final Sink sink;
    private final CommandCatalog catalog;

    public RouteCommandDispatcher(Sink sink, CommandCatalog catalog) {
        if (sink == null) throw new IllegalArgumentException("sink == null");
        if (catalog == null) throw new IllegalArgumentException("catalog == null");
        this.sink = sink;
        this.catalog = catalog;
    }

    /** Dispatches the event's command; {@code null} and command-less events are ignored. */
    public Result dispatch(SplineTrajectoryLoader.TrajectoryEvent event) {
        if (event == null) return Result.IGNORED;
        return dispatch(event.frameIndex, event.command, event.commandParams);
    }

    public Result dispatch(int frameIndex, String command, List<String> params) {
        if (command == null || command.trim().isEmpty()) return Result.IGNORED;
        String name = command.trim();

        CommandCatalog.Descriptor descriptor = catalog.find(name);
        if (descriptor == null) {
            Log.e(TAG, "frame[" + frameIndex + "]: unknown command '" + name + "'");
            return Result.UNKNOWN_COMMAND;
        }

        List<String> values = params == null
                ? Collections.<String>emptyList()
                : params;
        List<String> types = descriptor.typesForArity(values.size());
        if (types == null) {
            Log.e(TAG, "frame[" + frameIndex + "]: " + name
                    + " does not accept " + values.size() + " argument(s)");
            return Result.ARG_MISMATCH;
        }

        Object[] args;
        try {
            args = CommandArgs.convertAll(types, values);
        } catch (IllegalArgumentException e) {
            Log.e(TAG, "frame[" + frameIndex + "]: bad arguments for " + name
                    + ": " + e.getMessage());
            return Result.BAD_ARGS;
        }

        try {
            if (!sink.submit(name, args)) {
                Log.e(TAG, "frame[" + frameIndex + "]: " + name + " was rejected");
                return Result.REJECTED;
            }
        } catch (RuntimeException e) {
            Log.e(TAG, "frame[" + frameIndex + "]: " + name + " threw", e);
            return Result.THREW;
        }

        Log.i(TAG, "frame[" + frameIndex + "]: dispatched " + name);
        return Result.DISPATCHED;
    }

    /** True when the caller must abort the route instead of continuing. */
    public static boolean fatal(Result result) {
        return result != Result.DISPATCHED && result != Result.IGNORED;
    }
}
