package org.firstinspires.ftc.teamcode.common.command.auto;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Non-blocking trajectory sequencer for spline autonomous paths.
 *
 * <p>The preferred API is cooperative and must be ticked by the OpMode loop:</p>
 * <pre>
 * loader.start(json, motionDriver);
 * while (opModeIsActive()) {
 *     loader.update();      // one bounded-time state-machine tick
 *     TrajectoryEvent event = loader.pollEvent();
 *     ...                   // other robot/network/subsystem work
 * }
 * </pre>
 *
 * <p>This class deliberately does not implement a hidden loop, sleep, join, or async
 * hardware execution. MOVE steps are delegated to a {@link MotionDriver}; that driver's
 * {@link MotionDriver#update()} contract is also one bounded-time control tick.</p>
 *
 */
public final class SplineTrajectoryLoader {

    public enum FailurePolicy {
        STOP_PATH,
        CONTINUE
    }

    public enum ExecutionStatus {
        IDLE,
        RUNNING,
        COMPLETED,
        CANCELLED,
        EMPTY,
        PARSE_ERROR,
        TRACKER_FAILURE
    }

    /**
     * Non-blocking motion backend contract.
     *
     * <p>All methods are called from the OpMode/main control thread. Implementations must
     * return promptly and must not contain their own wait loop.</p>
     */
    public interface MotionDriver {
        /** Initialize path geometry/pose. Must return immediately. */
        void begin(Pose2D pose, SplineTracker.PathPoint start);

        /** Start one movement segment. Must not wait for completion. */
        void startMove(SplineTracker.PathPoint target);

        /** Execute exactly one control iteration and return the current segment result. */
        SplineTracker.SegmentResult update();

        /** Cancel the currently active motion and clear motor output. */
        void cancel();

        /** Last known result; must not perform control work. */
        SplineTracker.SegmentResult getLastResult();
    }

    /**
     * Pure-data event emitted at a trajectory waypoint. The Loader never executes the
     * command/marker in the new API; the OpMode decides what to do with it.
     */
    public static final class TrajectoryEvent {
        public final int frameIndex;
        public final String marker;
        public final String command;
        public final List<String> commandParams;

        private TrajectoryEvent(int frameIndex,
                                String marker,
                                String command,
                                List<String> commandParams) {
            this.frameIndex = frameIndex;
            this.marker = marker;
            this.command = command;
            this.commandParams = Collections.unmodifiableList(
                    new ArrayList<>(commandParams));
        }

        public boolean hasMarker() {
            return marker != null && !marker.isEmpty();
        }

        public boolean hasCommand() {
            return command != null && !command.isEmpty();
        }

        @Override
        public String toString() {
            return "TrajectoryEvent{frame=" + frameIndex
                    + ", marker='" + marker + '\''
                    + ", command='" + command + '\''
                    + ", params=" + commandParams + "}";
        }
    }

    public static final class ExecutionResult {
        public final ExecutionStatus status;
        public final int frameIndex;
        public final SplineTracker.SegmentResult segmentResult;
        public final String message;

        private ExecutionResult(ExecutionStatus status,
                                int frameIndex,
                                SplineTracker.SegmentResult segmentResult,
                                String message) {
            this.status = status;
            this.frameIndex = frameIndex;
            this.segmentResult = segmentResult;
            this.message = message;
        }

        public boolean completed() {
            return status == ExecutionStatus.COMPLETED;
        }

        public boolean running() {
            return status == ExecutionStatus.RUNNING;
        }

        public boolean terminal() {
            return status != ExecutionStatus.IDLE && status != ExecutionStatus.RUNNING;
        }

        @Override
        public String toString() {
            return "ExecutionResult{" + status
                    + ", frame=" + frameIndex
                    + ", segment=" + segmentResult
                    + (message != null ? ", message='" + message + "'" : "")
                    + "}";
        }
    }

    public static boolean HONOR_DELAY_AFTER_ARRIVE = false;
    public static boolean WARN_UNUSED_DURATION = true;

    private enum StepType { MOVE, WAIT }

    private static final class Step {
        final StepType type;
        final int frameIndex;
        final double waitSeconds;
        final SplineTracker.PathPoint point;
        final String marker;
        final String command;
        final List<String> commandParams;

        private Step(StepType type,
                     int frameIndex,
                     double waitSeconds,
                     SplineTracker.PathPoint point,
                     String marker,
                     String command,
                     List<String> commandParams) {
            this.type = type;
            this.frameIndex = frameIndex;
            this.waitSeconds = waitSeconds;
            this.point = point;
            this.marker = marker;
            this.command = command;
            this.commandParams = commandParams;
        }

        static Step waitFrame(int frameIndex,
                              double seconds,
                              String marker,
                              String command,
                              List<String> commandParams) {
            return new Step(
                    StepType.WAIT,
                    frameIndex,
                    seconds,
                    null,
                    marker,
                    command,
                    commandParams);
        }

        static Step moveFrame(int frameIndex,
                              SplineTracker.PathPoint point,
                              String marker,
                              String command,
                              List<String> commandParams) {
            return new Step(
                    StepType.MOVE,
                    frameIndex,
                    0,
                    point,
                    marker,
                    command,
                    commandParams);
        }

        TrajectoryEvent event() {
            if ((marker == null || marker.isEmpty())
                    && (command == null || command.isEmpty())) {
                return null;
            }
            return new TrajectoryEvent(frameIndex, marker, command, commandParams);
        }
    }

    /** Parsed immutable-ish plan for one run. */
    private static final class Plan {
        final List<Step> steps;

        Plan(List<Step> steps) {
            this.steps = Collections.unmodifiableList(new ArrayList<>(steps));
        }
    }

    private final MotionDriver motionDriver;
    private FailurePolicy failurePolicy = FailurePolicy.STOP_PATH;
    private Plan plan;
    private int stepIndex = -1;
    private boolean pathBegun;
    private boolean stepStarted;
    private long waitStartedNanos;
    private TrajectoryEvent pendingEvent;
    private ExecutionResult lastResult =
            new ExecutionResult(ExecutionStatus.IDLE, -1, null, "idle");

    public SplineTrajectoryLoader(MotionDriver motionDriver) {
        if (motionDriver == null) throw new IllegalArgumentException("motionDriver == null");
        this.motionDriver = motionDriver;
    }

    public SplineTrajectoryLoader setFailurePolicy(FailurePolicy policy) {
        this.failurePolicy = policy != null ? policy : FailurePolicy.STOP_PATH;
        return this;
    }

    /**
     * Parse and arm a trajectory. Does not move hardware and does not block.
     */
    public ExecutionResult start(String jsonString) {
        if (isRunning()) cancel();
        resetRunState();

        ParseResult parsed = parse(jsonString);
        if (parsed.error != null) {
            lastResult = parsed.error;
            return lastResult;
        }

        plan = parsed.plan;
        if (plan.steps.isEmpty()) {
            lastResult = new ExecutionResult(ExecutionStatus.EMPTY, -1, null, "empty trajectory");
            return lastResult;
        }

        stepIndex = 0;
        lastResult = new ExecutionResult(ExecutionStatus.RUNNING, 0, null, "running");
        return lastResult;
    }

    /**
     * Advance the trajectory state machine by at most one bounded unit of work.
     *
     * <p>No loops are used here. At most one MotionDriver.update() call occurs per call.</p>
     */
    public ExecutionResult update() {
        if (!isRunning()) return lastResult;

        // Preserve event ordering without an unbounded event queue. The caller must
        // consume the current waypoint event before the next step can advance.
        if (pendingEvent != null) return lastResult;

        if (plan == null || stepIndex < 0 || stepIndex >= plan.steps.size()) {
            return complete();
        }

        Step step = plan.steps.get(stepIndex);
        if (step.type == StepType.WAIT) {
            return updateWait(step);
        }
        return updateMove(step);
    }

    /**
     * Return and clear the current trajectory event. There is deliberately only one
     * event slot; Loader progression pauses until it is consumed, so events never build up.
     */
    public TrajectoryEvent pollEvent() {
        TrajectoryEvent event = pendingEvent;
        pendingEvent = null;
        return event;
    }

    public void cancel() {
        if (isRunning()) {
            motionDriver.cancel();
        }
        pendingEvent = null;
        stepStarted = false;
        waitStartedNanos = 0;
        if (lastResult.status == ExecutionStatus.RUNNING) {
            lastResult = new ExecutionResult(
                    ExecutionStatus.CANCELLED,
                    currentFrameIndex(),
                    motionDriver.getLastResult(),
                    "cancelled");
        }
    }

    public boolean isRunning() {
        return lastResult.status == ExecutionStatus.RUNNING;
    }

    public boolean isFinished() {
        return lastResult.terminal();
    }

    public ExecutionStatus getStatus() {
        return lastResult.status;
    }

    public ExecutionResult getLastResult() {
        return lastResult;
    }

    public int getCurrentFrameIndex() {
        return currentFrameIndex();
    }

    public int getStepIndex() {
        return stepIndex;
    }

    public int getStepCount() {
        return plan == null ? 0 : plan.steps.size();
    }

    private ExecutionResult updateWait(Step step) {
        if (!stepStarted) {
            stepStarted = true;
            waitStartedNanos = System.nanoTime();
            return lastResult;
        }

        double elapsedSeconds = (System.nanoTime() - waitStartedNanos) / 1e9;
        if (elapsedSeconds < step.waitSeconds) {
            return lastResult;
        }

        finishStep(step);
        return lastResult;
    }

    private ExecutionResult updateMove(Step step) {
        if (!pathBegun) {
            // The first geometric waypoint establishes the path pose/tangent and is not
            // itself a movement segment.
            motionDriver.begin(
                    new Pose2D(
                            DistanceUnit.INCH,
                            step.point.x,
                            step.point.y,
                            AngleUnit.DEGREES,
                            step.point.heading),
                    step.point);
            pathBegun = true;
            finishStep(step);
            return lastResult;
        }

        if (!stepStarted) {
            motionDriver.startMove(step.point);
            stepStarted = true;
            return lastResult;
        }

        // Exactly one control tick. Implementations must not block.
        SplineTracker.SegmentResult segment = motionDriver.update();
        if (segment == null || segment.status == SplineTracker.SegmentStatus.RUNNING) {
            return lastResult;
        }

        if (segment.status == SplineTracker.SegmentStatus.ARRIVED) {
            finishStep(step);
            return lastResult;
        }

        if (failurePolicy == FailurePolicy.CONTINUE) {
            Log.w("SplineAuto", "frame[" + step.frameIndex + "] skipped after " + segment.status);
            advanceStep();
            return lastResult;
        }

        motionDriver.cancel();
        lastResult = new ExecutionResult(
                ExecutionStatus.TRACKER_FAILURE,
                step.frameIndex,
                segment,
                "move failed: " + segment.status);
        return lastResult;
    }

    private void finishStep(Step step) {
        pendingEvent = step.event();
        advanceStep();
    }

    private void advanceStep() {
        stepStarted = false;
        waitStartedNanos = 0;
        stepIndex++;

        if (plan != null && stepIndex >= plan.steps.size() && pendingEvent == null) {
            complete();
        } else if (isRunning()) {
            lastResult = new ExecutionResult(
                    ExecutionStatus.RUNNING,
                    currentFrameIndex(),
                    motionDriver.getLastResult(),
                    "running");
        }
    }

    private ExecutionResult complete() {
        lastResult = new ExecutionResult(
                ExecutionStatus.COMPLETED,
                currentFrameIndex(),
                motionDriver.getLastResult(),
                "completed");
        return lastResult;
    }

    private int currentFrameIndex() {
        if (plan == null || plan.steps.isEmpty()) return -1;
        if (stepIndex < 0) return -1;
        if (stepIndex >= plan.steps.size()) {
            return plan.steps.get(plan.steps.size() - 1).frameIndex;
        }
        return plan.steps.get(stepIndex).frameIndex;
    }

    private void resetRunState() {
        plan = null;
        stepIndex = -1;
        pathBegun = false;
        stepStarted = false;
        waitStartedNanos = 0;
        pendingEvent = null;
        lastResult = new ExecutionResult(ExecutionStatus.IDLE, -1, null, "idle");
    }

    // ---------------------------------------------------------------------
    // Parsing: data only, no hardware / command execution
    // ---------------------------------------------------------------------

    private static final class ParseResult {
        final Plan plan;
        final ExecutionResult error;

        private ParseResult(Plan plan, ExecutionResult error) {
            this.plan = plan;
            this.error = error;
        }
    }

    private ParseResult parse(String jsonString) {
        if (jsonString == null || jsonString.trim().isEmpty()) {
            return new ParseResult(
                    null,
                    new ExecutionResult(ExecutionStatus.EMPTY, -1, null, "empty JSON"));
        }

        int parsingFrame = -1;
        try {
            JSONArray jsonArray = new JSONArray(jsonString);
            List<Step> steps = new ArrayList<>();

            for (int i = 0; i < jsonArray.length(); i++) {
                parsingFrame = i;
                JSONObject obj = jsonArray.getJSONObject(i);

                String marker = normalizeNullable(obj.optString("marker", null));
                String command = normalizeNullable(obj.optString("command", null));
                List<String> commandParams = parseCommandParams(obj.optJSONArray("commandParams"));

                if (obj.has("wait")) {
                    steps.add(Step.waitFrame(
                            i,
                            Math.max(0, obj.getDouble("wait")),
                            marker,
                            command,
                            commandParams));
                    continue;
                }

                double x = obj.getDouble("x");
                double y = obj.getDouble("y");
                double dx = obj.optDouble("dx", 0);
                double dy = obj.optDouble("dy", 0);
                double heading = obj.optDouble("heading", 0);

                double requestedDuration = obj.optDouble("duration", 0);
                if (WARN_UNUSED_DURATION && requestedDuration > 0) {
                    Log.w("SplineAuto", "waypoint[" + i + "]: duration=" + requestedDuration
                            + "s ignored by spatial follower");
                }

                steps.add(Step.moveFrame(
                        i,
                        new SplineTracker.PathPoint(x, y, dx, dy, heading),
                        marker,
                        command,
                        commandParams));

                double delayAfterArrive = obj.optDouble("delayAfterArrive", 0);
                if (delayAfterArrive > 0) {
                    if (HONOR_DELAY_AFTER_ARRIVE) {
                        steps.add(Step.waitFrame(
                                i,
                                delayAfterArrive,
                                null,
                                null,
                                Collections.emptyList()));
                    } else {
                        Log.w("SplineAuto", "waypoint[" + i + "]: delayAfterArrive="
                                + delayAfterArrive + "s ignored; use {\"wait\": n} or enable "
                                + "HONOR_DELAY_AFTER_ARRIVE");
                    }
                }
            }

            if (steps.isEmpty()) {
                return new ParseResult(
                        new Plan(steps),
                        new ExecutionResult(ExecutionStatus.EMPTY, -1, null, "empty trajectory"));
            }

            return new ParseResult(new Plan(steps), null);
        } catch (JSONException e) {
            Log.e("SplineAuto", "JSON parse failed at frame=" + parsingFrame, e);
            return new ParseResult(
                    null,
                    new ExecutionResult(
                            ExecutionStatus.PARSE_ERROR,
                            parsingFrame,
                            null,
                            e.getMessage()));
        }
    }

    private static List<String> parseCommandParams(JSONArray array) {
        if (array == null || array.length() == 0) return Collections.emptyList();
        List<String> result = new ArrayList<>(array.length());
        for (int i = 0; i < array.length(); i++) {
            result.add(array.optString(i, ""));
        }
        return Collections.unmodifiableList(result);
    }

    private static String normalizeNullable(String value) {
        if (value == null || value.isEmpty() || "null".equals(value)) return null;
        return value;
    }

}
