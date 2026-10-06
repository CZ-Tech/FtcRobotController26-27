package org.firstinspires.ftc.teamcode.common.command;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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
 * {@link SplineTracker} 专用的轨迹加载器。
 *
 * <p>JSON 支持两种关键帧：路径点（有 {@code x,y}，可带 {@code dx,dy,heading,duration,
 * marker,command,commandParams}）和等待点（{@code {"wait": 秒}}）。</p>
 *
 * <h3>执行流程（两趟）</h3>
 * <ol>
 *   <li><b>第一趟</b>：只解析，收集全部路径关键帧与执行计划（路径点 / 等待点交替），
 *       并把 marker 或 command 解析成 Runnable；</li>
 *   <li>用 {@link SplineTracker#applyScheduleConfig()} 同步 {@code SCHED_*} 参数后，
 *       用 {@link PathTimeline} 给<b>整条路径</b>排一张时间表（弧长采样 + 双向加减速扫描，
 *       {@code duration} 作为该段请求时长下限），并绑定到 SplineTracker；</li>
 *   <li><b>第二趟</b>：按计划顺序执行 —— 路径点走 {@code setPose/startMove/addPoint}，
 *       等待点走 {@code addTime}。段序与 addPoint 调用序一一对应，
 *       运行期由牛顿法投影出的实际进度查时间表（进度时钟）。</li>
 * </ol>
 *
 * <p>注意：{@code delayAfterArrive} 默认仍与旧版一样<b>不生效</b>（只告警；要启用把
 * {@link #HONOR_DELAY_AFTER_ARRIVE} 设为 true），需要等待请用独立的
 * {@code {"wait": 秒}} 关键帧 —— 等待点走真实计时，不参与运动时间表，
 * 因此它也不是时间表里的 {@code v=0} 锚点：等待之后那一段是按"带边界速度进入"排的，
 * 重新起步的加速时间没有被预算进去。</p>
 */
public class SplineTrajectoryLoader {

    private final SplineTracker tracker;
    private final Map<String, Runnable> markerTasks = new HashMap<>();

    /**
     * 是否把 JSON 的 {@code delayAfterArrive} 当作"到点后驻留"执行。
     * <p>默认 false = 与旧 SplineTrajectoryLoader 行为一致（忽略该字段，只告警）。
     * 把路由从 Pinpoint 路径迁到 spline 路径时，如果原路由靠
     * {@code delayAfterArrive}（真实数据里是 2.5~3.0s 的射击时间）控时序，
     * 需要打开这个开关，否则动作时序会全部丢失。</p>
     */
    public static boolean HONOR_DELAY_AFTER_ARRIVE = false;

    /** 执行计划条目：路径点或等待点。 */
    private static final class Step {
        final boolean wait;
        final double waitSeconds;
        final double x, y, dx, dy, heading;
        final Runnable task;

        private Step(boolean wait, double waitSeconds,
                     double x, double y, double dx, double dy, double heading, Runnable task) {
            this.wait = wait;
            this.waitSeconds = waitSeconds;
            this.x = x;
            this.y = y;
            this.dx = dx;
            this.dy = dy;
            this.heading = heading;
            this.task = task;
        }

        static Step waitFrame(double seconds, Runnable task) {
            return new Step(true, seconds, 0, 0, 0, 0, 0, task);
        }

        static Step waypoint(double x, double y, double dx, double dy, double heading, Runnable task) {
            return new Step(false, 0, x, y, dx, dy, heading, task);
        }
    }

    public SplineTrajectoryLoader(SplineTracker tracker) {
        this.tracker = tracker;
    }

    /**
     * 注册标记任务。
     * @param marker JSON 中的 marker 名称
     * @param task   到达该标记时执行的任务
     * @return this（链式调用）
     */
    public SplineTrajectoryLoader addMarkerTask(String marker, Runnable task) {
        markerTasks.put(marker, task);
        return this;
    }

    /**
     * 解析 JSON 并驱动 SplineTracker。
     *
     * JSON 格式示例：
     * <pre>
     * [
     *     {"x": 0,  "y": 0,  "dx": 0, "dy": 0, "heading": 0},
     *     {"x": 24, "y": 0,  "dx": 0, "dy": 0, "heading": 0, "marker": "shoot", "duration": 1.2},
     *     {"wait": 1.5},
     *     {"x": 48, "y": 24, "dx": 0, "dy": 0, "heading": 90},
     *     {"wait": 2.0, "marker": "done"}
     * ]
     * </pre>
     *
     * @param jsonString 轨迹 JSON 字符串
     */
    public void execute(String jsonString) {
        HttpJsonService.scanObjectTree(tracker.getRobot());
        int frameIndex = -1;
        try {
            JSONArray jsonArray = new JSONArray(jsonString);
            if (jsonArray.length() == 0) return;

            // ---------- 第一趟：解析，收集运动关键帧 + 执行计划 ----------
            List<PathTimeline.Waypoint> waypoints = new ArrayList<>();
            List<Step> steps = new ArrayList<>();

            for (int i = 0; i < jsonArray.length(); i++) {
                frameIndex = i;
                JSONObject obj = jsonArray.getJSONObject(i);

                if (obj.has("wait")) {
                    // --- 等待点 ---（marker / command 与路径点一致）
                    double waitSeconds = obj.getDouble("wait");
                    Runnable task = markerTask(obj, i);
                    if (task == null) task = commandTask(obj, i);
                    steps.add(Step.waitFrame(waitSeconds, task));
                    continue;
                }

                // --- 路径点 ---
                double x = obj.getDouble("x");
                double y = obj.getDouble("y");
                double dx = obj.optDouble("dx", 0);
                double dy = obj.optDouble("dy", 0);
                double heading = obj.optDouble("heading", 0);
                double requestedDuration = obj.optDouble("duration", 0);

                // 优先级：marker → command
                Runnable task = markerTask(obj, i);
                if (task == null) task = commandTask(obj, i);

                waypoints.add(new PathTimeline.Waypoint(x, y, dx, dy, heading, requestedDuration));
                steps.add(Step.waypoint(x, y, dx, dy, heading, task));

                // delayAfterArrive：AzConductor 的"到点驻留"（真实路由里是 2.5~3.0s 的射击时间）
                double delayAfterArrive = obj.optDouble("delayAfterArrive", 0);
                if (delayAfterArrive > 0) {
                    if (HONOR_DELAY_AFTER_ARRIVE) {
                        steps.add(Step.waitFrame(delayAfterArrive, null));
                    } else {
                        Log.w("SplineAuto", "waypoint[" + i + "]: delayAfterArrive=" + delayAfterArrive
                                + "s 被忽略（与旧行为一致）；要等待请插入 {\"wait\": n} 帧，"
                                + "或把 SplineTrajectoryLoader.HONOR_DELAY_AFTER_ARRIVE 设为 true");
                    }
                }
            }

            // ---------- 第二趟：整条路径先排时间表（纯 wait 帧的 JSON 没有运动段） ----------
            if (waypoints.size() >= 2) {
                SplineTracker.applyScheduleConfig();
                PathTimeline timeline = new PathTimeline(waypoints, 0.0);
                if (PathTimeline.LOG_SCHEDULE) Log.i("SplineAuto", timeline.describe());
                tracker.setTimeline(timeline);      // 段序与后续 addPoint 调用序一一对应
            } else {
                tracker.setTimeline(null);
            }

            // ---------- 第三趟：按计划执行（没有路径点时 wait 帧同样要执行） ----------
            boolean firstWaypoint = true;
            for (Step step : steps) {
                if (step.wait) {
                    if (step.task != null) tracker.addTime(step.waitSeconds, step.task);
                    else tracker.addTime(step.waitSeconds);
                    continue;
                }

                if (firstWaypoint) {
                    // 第一个路径点：对齐里程计 + 起步
                    tracker.setPose(new Pose2D(
                            DistanceUnit.INCH, step.x, step.y,
                            AngleUnit.DEGREES, step.heading
                    ));
                    if (step.task != null) {
                        tracker.startMove(step.x, step.y, step.dx, step.dy, step.task);
                    } else {
                        tracker.startMove(step.x, step.y, step.dx, step.dy);
                    }
                    // 同步朝向，覆盖 startMove 从里程计读取的朝向
                    tracker.heading = step.heading;
                    tracker.preH = step.heading;
                    firstWaypoint = false;
                } else {
                    if (step.task != null) {
                        tracker.addPoint(step.x, step.y, step.dx, step.dy, step.heading, step.task);
                    } else {
                        tracker.addPoint(step.x, step.y, step.dx, step.dy, step.heading);
                    }
                }
            }

            // 解绑，避免下一条路径（或之后手工链式 addPoint）误用本次时间表
            tracker.setTimeline(null);

        } catch (JSONException e) {
            Log.e("SplineAuto", "JSON 解析失败（frame=" + frameIndex + "），整条路径不执行", e);
        } catch (Exception e) {
            Log.e("SplineAuto", "执行路径异常（frame=" + frameIndex + "）", e);
        }
    }

    /**
     * 静态便捷方法。
     */
    public static void executeJsonTrajectory(String jsonString, SplineTracker tracker) {
        new SplineTrajectoryLoader(tracker).execute(jsonString);
    }

    /**
     * marker 字段 → 已注册的 marker 任务。未注册时记录 [BRK3] 诊断日志。
     */
    private Runnable markerTask(JSONObject obj, int index) {
        String rawMarker = obj.optString("marker", null);
        if (rawMarker == null || rawMarker.isEmpty() || "null".equals(rawMarker)) return null;

        Runnable task = markerTasks.get(rawMarker);
        if (task == null) {
            Log.w("SplineAuto", "[BRK3] waypoint[" + index + "]: marker='" + rawMarker
                    + "' not in markerTasks (size=" + markerTasks.size() + "), fallback to command");
        }
        return task;
    }

    /**
     * command / commandParams 字段 → @AutoTask 命令。未指定时返回 null（普通路径点）。
     */
    private Runnable commandTask(JSONObject obj, int index) {
        String command = obj.optString("command", null);
        if (command == null || command.isEmpty()) {
            Log.d("SplineAuto", "[INFO] waypoint[" + index + "]: plain waypoint, no marker/command");
            return null;
        }
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
        return resolveCommandTask(command, commandParams);
    }

    /**
     * 将 JSON 中的 command 名称解析为可执行的 Runnable。
     * 通过 {@link HttpJsonService#createCommandRunnable(String, String[])}
     * 桥接 AzConductor 的 command/commandParams 字段与 @AutoTask 命令注册表。
     */
    private Runnable resolveCommandTask(String commandName, String[] commandParams) {
        Runnable task = HttpJsonService.createCommandRunnable(commandName, commandParams);
        if (task != null) {
            Log.i("SplineAuto", "[BRK1-OK] Resolved command task: " + commandName
                    + ", params=" + Arrays.toString(commandParams));
        } else {
            Log.w("SplineAuto", "[BRK1] Unresolved command: '" + commandName
                    + "' — check /commands endpoint for matching name");
        }
        return task;
    }
}
