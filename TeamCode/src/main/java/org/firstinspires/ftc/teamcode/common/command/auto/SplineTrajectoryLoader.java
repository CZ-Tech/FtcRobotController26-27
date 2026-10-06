package org.firstinspires.ftc.teamcode.common.command.auto;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.TaskLoopFrame;
import org.firstinspires.ftc.teamcode.common.util.HttpJsonService;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Public facade for spline autonomous execution.
 *
 * <p>Responsibilities:</p>
 * <ul>
 *     <li>parse trajectory JSON;</li>
 *     <li>resolve marker / command tasks;</li>
 *     <li>sequence move / wait steps;</li>
 *     <li>decide whether a failed segment stops the whole trajectory;</li>
 *     <li>execute endpoint tasks only after a successful arrival/hold.</li>
 * </ul>
 *
 * <p>It intentionally does not know anything about Newton projection, Hermite math,
 * velocity PID, cross-track PID, or motor power composition. Those belong to
 * {@link SplineTracker}.</p>
 */
public final class SplineTrajectoryLoader {

    public enum FailurePolicy {
        /** Recommended: a STALLED/ABORTED segment stops the remaining trajectory. */
        STOP_PATH,
        /** Skip the failed step's task and continue to later steps. */
        CONTINUE
    }

    public enum ExecutionStatus {
        COMPLETED,
        EMPTY,
        PARSE_ERROR,
        TRACKER_FAILURE
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

        @Override
        public String toString() {
            return "ExecutionResult{" + status
                    + ", frame=" + frameIndex
                    + ", segment=" + segmentResult
                    + (message != null ? ", message='" + message + "'" : "")
                    + "}";
        }
    }

    /** Preserve old data behavior unless explicitly enabled. */
    public static boolean HONOR_DELAY_AFTER_ARRIVE = false;

    /** Task execution policy lives here, not in the low-level follower. */
    public static boolean ASYNC_TASKS = false;
    public static boolean DEBUG_RUN_TASKS = true;
    public static boolean DO_NOT_RUN_TASKS = false;

    /** duration is a legacy field; the spatial follower intentionally ignores it. */
    public static boolean WARN_UNUSED_DURATION = true;

    private enum StepType { MOVE, WAIT }

    private static final class Step {
        final StepType type;
        final int frameIndex;
        final double waitSeconds;
        final SplineTracker.PathPoint point;
        final Runnable task;

        private Step(StepType type,
                     int frameIndex,
                     double waitSeconds,
                     SplineTracker.PathPoint point,
                     Runnable task) {
            this.type = type;
            this.frameIndex = frameIndex;
            this.waitSeconds = waitSeconds;
            this.point = point;
            this.task = task;
        }

        static Step waitFrame(int frameIndex, double seconds, Runnable task) {
            return new Step(StepType.WAIT, frameIndex, seconds, null, task);
        }

        static Step moveFrame(int frameIndex, SplineTracker.PathPoint point, Runnable task) {
            return new Step(StepType.MOVE, frameIndex, 0, point, task);
        }
    }

    private final Robot robot;
    private final SplineTracker tracker;
    private final Map<String, Runnable> markerTasks = new HashMap<>();
    private FailurePolicy failurePolicy = FailurePolicy.STOP_PATH;

    /** Recommended public entry point. */
    public SplineTrajectoryLoader(Robot robot) {
        if (robot == null) throw new IllegalArgumentException("robot == null");
        this.robot = robot;
        this.tracker = new SplineTracker(robot);
    }

    /**
     * Compatibility/testing constructor. Existing code that already creates a tracker can
     * keep doing so, but new application code normally does not need direct tracker access.
     */
    public SplineTrajectoryLoader(SplineTracker tracker) {
        if (tracker == null) throw new IllegalArgumentException("tracker == null");
        this.tracker = tracker;
        this.robot = tracker.getRobot();
    }

    public SplineTrajectoryLoader setFailurePolicy(FailurePolicy policy) {
        this.failurePolicy = policy != null ? policy : FailurePolicy.STOP_PATH;
        return this;
    }

    public SplineTrajectoryLoader addMarkerTask(String marker, Runnable task) {
        if (marker == null || marker.isEmpty()) {
            throw new IllegalArgumentException("marker is empty");
        }
        if (task == null) {
            markerTasks.remove(marker);
        } else {
            markerTasks.put(marker, task);
        }
        return this;
    }

    /** Advanced access for dashboard/debugging; normal autonomous code need not use it. */
    public SplineTracker getTracker() {
        return tracker;
    }

    /** Execute a complete JSON trajectory synchronously. */
    public ExecutionResult execute(String jsonString) {
        HttpJsonService.scanObjectTree(robot);

        if (jsonString == null || jsonString.trim().isEmpty()) {
            return new ExecutionResult(ExecutionStatus.EMPTY, -1, null, "empty JSON");
        }

        int parsingFrame = -1;
        try {
            JSONArray jsonArray = new JSONArray(jsonString);
            if (jsonArray.length() == 0) {
                return new ExecutionResult(ExecutionStatus.EMPTY, -1, null, "empty trajectory");
            }

            List<Step> steps = new ArrayList<>();

            for (int i = 0; i < jsonArray.length(); i++) {
                parsingFrame = i;
                JSONObject obj = jsonArray.getJSONObject(i);
                Runnable task = resolveFrameTask(obj, i);

                if (obj.has("wait")) {
                    double waitSeconds = Math.max(0, obj.getDouble("wait"));
                    steps.add(Step.waitFrame(i, waitSeconds, task));
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

                SplineTracker.PathPoint point = new SplineTracker.PathPoint(
                        x, y, dx, dy, heading);
                steps.add(Step.moveFrame(i, point, task));

                double delayAfterArrive = obj.optDouble("delayAfterArrive", 0);
                if (delayAfterArrive > 0) {
                    if (HONOR_DELAY_AFTER_ARRIVE) {
                        steps.add(Step.waitFrame(i, delayAfterArrive, null));
                    } else {
                        Log.w("SplineAuto", "waypoint[" + i + "]: delayAfterArrive="
                                + delayAfterArrive + "s ignored; use {\"wait\": n} or enable "
                                + "HONOR_DELAY_AFTER_ARRIVE");
                    }
                }
            }

            return executeSteps(steps);

        } catch (JSONException e) {
            Log.e("SplineAuto", "JSON parse failed at frame=" + parsingFrame, e);
            return new ExecutionResult(
                    ExecutionStatus.PARSE_ERROR,
                    parsingFrame,
                    null,
                    e.getMessage());
        } catch (Exception e) {
            Log.e("SplineAuto", "trajectory execution exception at frame=" + parsingFrame, e);
            return new ExecutionResult(
                    ExecutionStatus.TRACKER_FAILURE,
                    parsingFrame,
                    tracker.getLastResult(),
                    e.getMessage());
        }
    }

    private ExecutionResult executeSteps(List<Step> steps) {
        boolean pathBegun = false;
        int lastFrame = -1;

        for (Step step : steps) {
            lastFrame = step.frameIndex;

            if (step.type == StepType.WAIT) {
                SplineTracker.SegmentResult result = tracker.hold(step.waitSeconds);
                if (!result.arrived()) {
                    ExecutionResult failure = trackerFailure(step.frameIndex, result, "wait failed");
                    if (failurePolicy == FailurePolicy.STOP_PATH) return failure;
                    continue;
                }
                runTask(step.task, "wait@" + step.frameIndex);
                continue;
            }

            if (!pathBegun) {
                // First path point defines the global pose and initial Hermite tangent.
                tracker.begin(new Pose2D(
                        DistanceUnit.INCH, step.point.x, step.point.y,
                        AngleUnit.DEGREES, step.point.heading), step.point);
                pathBegun = true;

                // A task attached to the first waypoint means "at path start".
                runTask(step.task, "start@" + step.frameIndex);
                continue;
            }

            SplineTracker.SegmentResult result = tracker.followTo(step.point);
            if (!result.arrived()) {
                ExecutionResult failure = trackerFailure(
                        step.frameIndex, result, "move failed: " + result.status);
                if (failurePolicy == FailurePolicy.STOP_PATH) return failure;
                continue; // intentionally skip this step's endpoint task
            }

            runTask(step.task, "arrive@" + step.frameIndex);
        }

        return new ExecutionResult(ExecutionStatus.COMPLETED, lastFrame,
                tracker.getLastResult(), "completed");
    }

    private ExecutionResult trackerFailure(int frame,
                                           SplineTracker.SegmentResult result,
                                           String message) {
        Log.w("SplineAuto", "frame[" + frame + "] " + message + ": " + result);
        return new ExecutionResult(
                ExecutionStatus.TRACKER_FAILURE,
                frame,
                result,
                message);
    }

    private Runnable resolveFrameTask(JSONObject obj, int index) {
        Runnable task = markerTask(obj, index);
        if (task == null) task = commandTask(obj, index);
        return task;
    }

    private Runnable markerTask(JSONObject obj, int index) {
        String rawMarker = obj.optString("marker", null);
        if (rawMarker == null || rawMarker.isEmpty() || "null".equals(rawMarker)) return null;

        Runnable task = markerTasks.get(rawMarker);
        if (task == null) {
            Log.w("SplineAuto", "waypoint[" + index + "]: marker='" + rawMarker
                    + "' not registered; falling back to command");
        }
        return task;
    }

    private Runnable commandTask(JSONObject obj, int index) {
        String command = obj.optString("command", null);
        if (command == null || command.isEmpty()) return null;

        JSONArray cpArr = obj.optJSONArray("commandParams");
        String[] commandParams;
        if (cpArr != null) {
            commandParams = new String[cpArr.length()];
            for (int j = 0; j < cpArr.length(); j++) {
                commandParams[j] = cpArr.optString(j, "");
            }
        } else {
            commandParams = new String[0];
        }

        Runnable task = HttpJsonService.createCommandRunnable(command, commandParams);
        if (task != null) {
            Log.i("SplineAuto", "Resolved command: " + command
                    + ", params=" + Arrays.toString(commandParams));
        } else {
            Log.w("SplineAuto", "Unresolved command at waypoint[" + index + "]: '"
                    + command + "'");
        }
        return task;
    }

    private void runTask(Runnable task, String location) {
        if (task == null || DO_NOT_RUN_TASKS) return;
        if (!DEBUG_RUN_TASKS && org.firstinspires.ftc.teamcode.common.Globals.DEBUG) return;

        if (ASYNC_TASKS) {
            Log.d("SplineAuto", "[ASYNC] " + location);
            TaskLoopFrame.runOnce(task);
            return;
        }

        Log.d("SplineAuto", "[BLOCK] " + location + ": start");
        try {
            task.run();
            Log.d("SplineAuto", "[BLOCK] " + location + ": done");
        } catch (Exception e) {
            Log.e("SplineAuto", "task failed at " + location, e);
        }
    }

    public static ExecutionResult executeJsonTrajectory(String jsonString, Robot robot) {
        return new SplineTrajectoryLoader(robot).execute(jsonString);
    }

    /** Compatibility helper for existing code that already owns a tracker. */
    public static ExecutionResult executeJsonTrajectory(String jsonString, SplineTracker tracker) {
        return new SplineTrajectoryLoader(tracker).execute(jsonString);
    }
}
