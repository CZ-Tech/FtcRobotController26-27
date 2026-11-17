package org.firstinspires.ftc.teamcode.common.command.auto;

import android.util.Log;

import org.firstinspires.ftc.teamcode.common.network.CommandArgs;
import org.firstinspires.ftc.teamcode.common.network.CommandCatalog;
import org.firstinspires.ftc.teamcode.common.network.CommandDispatcher;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Validates and submits the commands attached to one finished route frame.
 *
 * <p>The loader emits legacy {@code command}/{@code commandParams} or an ordered
 * {@code commands} list as data. This dispatcher resolves every name and converts every
 * parameter before handing the entire frame to a {@link BatchSink} exactly once. The
 * sink must accept the list as one managed task: execute calls in order within that task,
 * while tasks from different frames may run concurrently. Aggregate command bodies use
 * ordinary Java calls and sleep/polling, not a route-command DSL.</p>
 *
 * <p>Dispatch does not wait for task completion. Every result other than
 * {@link Result#DISPATCHED} and {@link Result#IGNORED} is fatal to the caller's route;
 * task lifetime and cancellation remain the owning OpMode's responsibility.</p>
 */
public final class RouteCommandDispatcher {
    /** Legacy single-call adapter. A frame with multiple calls is rejected, never split. */
    public interface Sink {
        boolean submit(String name, Object[] args);
    }

    /** Accept or reject one complete, validated, ordered frame as a single managed task. */
    @FunctionalInterface
    public interface BatchSink {
        boolean submit(List<CommandDispatcher.Call> calls);
    }

    public enum Result {
        /** No commands on this frame; nothing to do. */
        IGNORED,
        DISPATCHED,
        /** A name is not in the published catalog. */
        UNKNOWN_COMMAND,
        /** The catalog has no signature with this argument count. */
        ARG_MISMATCH,
        /** A parameter cannot be converted to the declared type. */
        BAD_ARGS,
        /** The sink refused the batch, or a legacy sink received multiple calls. */
        REJECTED,
        /** The sink threw while submitting the batch. */
        THREW
    }

    private static final String TAG = "RouteCommand";

    private final BatchSink sink;
    private final CommandCatalog catalog;

    public RouteCommandDispatcher(Sink sink, CommandCatalog catalog) {
        this(catalog, singleCallAdapter(sink));
    }

    private RouteCommandDispatcher(CommandCatalog catalog, BatchSink sink) {
        if (sink == null) throw new IllegalArgumentException("sink == null");
        if (catalog == null) throw new IllegalArgumentException("catalog == null");
        this.sink = sink;
        this.catalog = catalog;
    }

    /** Batch entry point without introducing ambiguous functional-interface constructors. */
    public static RouteCommandDispatcher forBatch(BatchSink sink, CommandCatalog catalog) {
        return new RouteCommandDispatcher(catalog, sink);
    }

    private static BatchSink singleCallAdapter(final Sink sink) {
        if (sink == null) throw new IllegalArgumentException("sink == null");
        return calls -> {
            if (calls.size() != 1) return false;
            CommandDispatcher.Call call = calls.get(0);
            return sink.submit(call.name, call.arguments());
        };
    }

    /** Dispatches the event's complete list; null and command-less events are ignored. */
    public Result dispatch(SplineTrajectoryLoader.TrajectoryEvent event) {
        if (event == null) return Result.IGNORED;
        return dispatchCommands(event.frameIndex, event.commands);
    }

    /** Compatibility entry point for a legacy single command. */
    public Result dispatch(int frameIndex, String command, List<String> params) {
        if (command == null || command.trim().isEmpty()) return Result.IGNORED;
        return dispatchCommands(frameIndex, Collections.singletonList(
                new SplineTrajectoryLoader.TrajectoryCommand(command,
                        params == null ? Collections.<String>emptyList() : params)));
    }

    private Result dispatchCommands(
            int frameIndex, List<SplineTrajectoryLoader.TrajectoryCommand> commands) {
        if (commands.isEmpty()) return Result.IGNORED;
        List<CommandDispatcher.Call> calls = new ArrayList<>(commands.size());
        for (SplineTrajectoryLoader.TrajectoryCommand command : commands) {
            String name = command.name;
            CommandCatalog.Descriptor descriptor = catalog.find(name);
            if (descriptor == null) {
                Log.e(TAG, "frame[" + frameIndex + "]: unknown command '" + name + "'");
                return Result.UNKNOWN_COMMAND;
            }

            List<String> types = descriptor.typesForArity(command.params.size());
            if (types == null) {
                Log.e(TAG, "frame[" + frameIndex + "]: " + name
                        + " does not accept " + command.params.size() + " argument(s)");
                return Result.ARG_MISMATCH;
            }

            try {
                calls.add(new CommandDispatcher.Call(name,
                        CommandArgs.convertAll(types, command.params)));
            } catch (IllegalArgumentException e) {
                Log.e(TAG, "frame[" + frameIndex + "]: bad arguments for " + name
                        + ": " + e.getMessage());
                return Result.BAD_ARGS;
            }
        }

        try {
            if (!sink.submit(Collections.unmodifiableList(calls))) {
                Log.e(TAG, "frame[" + frameIndex + "]: command batch was rejected");
                return Result.REJECTED;
            }
        } catch (RuntimeException e) {
            Log.e(TAG, "frame[" + frameIndex + "]: command batch submission threw", e);
            return Result.THREW;
        }

        Log.i(TAG, "frame[" + frameIndex + "]: dispatched " + calls.size() + " command(s)");
        return Result.DISPATCHED;
    }

    /** True when the caller must abort the route instead of continuing. */
    public static boolean fatal(Result result) {
        return result != Result.DISPATCHED && result != Result.IGNORED;
    }
}
