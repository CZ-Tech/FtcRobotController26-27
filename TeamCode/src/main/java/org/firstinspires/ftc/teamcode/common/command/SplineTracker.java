package org.firstinspires.ftc.teamcode.common.command;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.TaskLoopFrame;


public class SplineTracker {
    public static boolean DEBUG_RUN_FUNC = true;
    public static boolean DO_NOT_RUN_FUNC = false;
    public static boolean ASYNC_TASKS = false;   // true = 异步 fire-and-forget（旧行为）; false = 默认阻塞等待任务完成

    //region Point Details
    public double[] startPoint;
    public double lx, ly, ldx, ldy;
    public double x, y, dx, dy;
    public double heading, preH;
    public Runnable fn;

    // Tunable gains
    public static double K_HEADING = 0.02;     // 朝向P

    // Tangential velocity control
    public static double MAX_TANGENTIAL_VEL = 50.0;  // 机器能达到的最大速度(inch/s)
    public static int DERIVATIVE_SAMPLE_COUNT = 200;

    public static int SAMPLE_COUNT = 20;  // 曲线最近点计算取样数量
    public static int TIME_MAP_SAMPLES = 200;  // u→时间映射采样数

    // 加速度限制：对段内期望速度变化率（in/s²）。与段首、段尾速度约束结合，
    // 自动解算出当前沿样条的最大允许速度。设 INFINITY 或 <=0 时禁用限速。
    public static double MAX_ACCEL = 50;         // 切向加速度上限 (in/s²), 80≈0.2g

    public static int NEWTON_ITERS = 3;
    public static double END_U_THRESHOLD = 0.935;
    public static double END_POS_THRESHOLD = 1.0;
    public static double END_HEADING_THRESHOLD = 0.5;

    public static double TURN_DEAD_AREA = 0.0;
    public static double motorVoltage = 12;
    public static double VOLTAGE_COMP_WEIGHT = 0.0;

    // —— Look-ahead pursuit ——
    public static double LOOKAHEAD_MS = 15;       // n: prediction horizon in milliseconds

    // —— Position-error PID (shared gains for X and Y) ——
    public static double K_POS_P = 0.7;
    public static double K_POS_I = 0.6;
    public static double K_POS_D = 0.07;
    public static double POS_I_MAX = 0.5;
    public static double POS_D_ALPHA = 0.9;  // 滤波强度，[0,1]，越小滤波越强，但对变化敏感度越低
    public static double POS_I_WINDOW = 10.0;
    public static double POS_DEAD_ZONE = 0.0;

    public ElapsedTime runtime = new ElapsedTime();
    private final Robot robot;
    private double gotoX, gotoY, gotoH;

    // Velocity-profile state (per segment, retained from original)
    private double segmentMaxDerivative;
    private double segmentArcLength = 0;
    private double prevDesiredVel = 0;

    // Position PID state (per segment)
    private final PosPIDState posXState = new PosPIDState();
    private final PosPIDState posYState = new PosPIDState();
    private double prevTime = 0;
    private double segmentTotalTime = 0;   // 当前段预计算总时长 (s)
    private double[] timeMapCache;          // 当前段 u→t 映射
    private double[] velMapCache;           // 当前段 u→v 映射

    //region 路径级时间表（进度时钟）
    /**
     * 路径级时间表。非 null 时启用"进度时钟"：
     * 用牛顿法投影出的实际进度查时间表得到"虚拟已用时"，替代旧的固定计时器
     * （{@code runtime.seconds()} + 段内 precomputeTimeMap）。
     */
    private PathTimeline timeline;
    /** 当前段在时间表中的序号，每次 addPoint 递增。 */
    private int timelineSegment = -1;
    /** true = 进度时钟（默认）；false = 旧的 wall-clock 行为，便于 A/B 回归。 */
    public static boolean USE_PROGRESS_CLOCK = true;
    /**
     * 段内进度停滞超过该秒数则视为到达并跳出（&lt;=0 关闭）。两种时钟都生效。
     *
     * <p>提交版本（HEAD）本来有一个 {@code MAX_SEGMENT_TIMEOUT = 5.0} 的固定段超时，
     * 工作区把它删掉了；这里用"零进度 2s"作为更精确的替代，避免退化成整条自动挂死。</p>
     */
    public static double STALL_TIMEOUT_S = 2.0;
    /** 每周期 6 条 Log.i 会明显吃掉控制周期，默认关闭。 */
    public static boolean LOG_VERBOSE = false;

    // —— 时间表参数（@Config 可在 FTC Dashboard 上 A/B，由 applyScheduleConfig() 同步到 PathTimeline）——
    // 注意：进度时钟下真正生效的是下面这几个 SCHED_*；MAX_TANGENTIAL_VEL / MAX_ACCEL /
    //       TIME_MAP_SAMPLES 只在"没有时间表"或 USE_PROGRESS_CLOCK=false 的旧路径里起作用。
    //       当前控制律里没有任何前馈项：驱动只来自位置误差 PID + 转向 P。
    /** 切向速度上限 (in/s)。 */
    public static double SCHED_V_MAX = 50.0;
    /** 切向加速度上限 (in/s^2)。 */
    public static double SCHED_A_MAX = 50.0;
    /** 速度上限的下限 (in/s)。 */
    public static double SCHED_CAP_FLOOR = 2.0;
    /** 横向加速度上限 (in/s^2)，<=0 关闭。 */
    public static double SCHED_A_LAT = 0.0;
    /** >0 时把整条路径时间缩放到该秒数（只变慢）。 */
    public static double SCHED_TARGET_TIME_S = 0.0;
    /** {@code |(dx,dy)|} 的用法：PathTimeline.TANGENT_ABSOLUTE / _RELATIVE / _OFF。 */
    public static int SCHED_TANGENT_MODE = PathTimeline.TANGENT_ABSOLUTE;
    /**
     * 0 = 纯进度时钟（默认，严格按"u* 完全替代计时器"的设计）。
     * &gt;0 = 限幅混合时钟：虚拟时钟按真实时间推进，但最多领先实际进度该秒数。
     *
     * <p>为什么需要这个开关：纯进度时钟的前视量 = {@code v_sched * LOOKAHEAD}，
     * 而路径起步时 v_sched→0，前视量只有 ~0.04in，位置 P 项给不出克服静摩擦
     * （本底盘约需 0.09 功率）的力，要靠积分项慢慢爬出来 —— 起步可能晚 2s 左右。
     * 混合时钟在这种情况下让目标点继续前进（误差变大 → 有力起步），同时把
     * "领先实际进度"的额度限制在该秒数内，因此仍然不会切角。</p>
     *
     * <p>上机建议先试 0.2~0.4。</p>
     */
    public static double SCHED_HYBRID_LEAD_S = 0.0;

    private final ElapsedTime stallTimer = new ElapsedTime();
    private double stallLastU = 0;
    /** 本段到达过的最大 u*：单调钳制，防止投影跨周期回退把虚拟时钟倒着走。 */
    private double lastUStar = 0;
    //endregion

    /**
     * Per-axis position-PID accumulator.
     */
    private static class PosPIDState {
        double integral = 0;
        double lastError = 0;
        double lastFilteredDeriv = 0;
        boolean firstRun = true;

        void reset() {
            integral = 0;
            lastError = 0;
            lastFilteredDeriv = 0;
            firstRun = true;
        }
    }
    //endregion

    //region Route Generator
    private double[] spline_fit(double x0, double dx0, double x1, double dx1) {
        double a = 2 * x0 + dx0 - 2 * x1 + dx1;
        double b = -3 * x0 - 2 * dx0 + 3 * x1 - dx1;
        double c = dx0;
        double d = x0;
        return new double[]{a, b, c, d};
    }

    private double spline_get(double[] spline, double u) {
        return spline[0] * u * u * u + spline[1] * u * u + spline[2] * u + spline[3];
    }

    private double splineDerivative(double[] spline, double u) {
        return 3 * spline[0] * u * u + 2 * spline[1] * u + spline[2];
    }

    /**
     * 牛顿法：机器人位置在样条上的投影参数 u，即"实际进度"。
     * <p>实现在 {@link PathTimeline#projectU}，与离线自测共用同一份代码。</p>
     */
    private double findClosestU(double[] splineX, double[] splineY, double rx, double ry) {
        return PathTimeline.projectU(splineX, splineY, rx, ry, SAMPLE_COUNT, NEWTON_ITERS);
    }

    private double angleDiff(double target, double current) {
        double diff = target - current;
        while (diff > 180) diff -= 360;
        while (diff <= -180) diff += 360;
        return diff;
    }

    /**
     * Single-axis position PID with EMA-filtered derivative, anti-windup integral,
     * and dead-zone handling. Returns a power command in [-1, 1].
     */
    private double computePositionPID(double error, double dt,
                                       double kp, double ki, double kd,
                                       double iWindow, double iMax,
                                       double dAlpha, double deadZone,
                                       PosPIDState state, double motorPowerGain) {
        if (state.firstRun) {
            state.firstRun = false;
            state.lastError = error;
            state.integral = 0;
            state.lastFilteredDeriv = 0;
            return kp * error * motorPowerGain;
        }

        if (Math.abs(error) < deadZone) {
            state.integral = 0;
            return 0;
        }

        // Proportional
        double pTerm = kp * error;

        // Integral with conditional anti-windup
        if (Math.abs(error) < iWindow) {
            state.integral += error * dt;
        } else {
            state.integral = 0;
        }
        state.integral = Math.max(-iMax, Math.min(iMax, state.integral));
        double iTerm = ki * state.integral;

        // Derivative with EMA filtering
        double rawDeriv = (error - state.lastError) / dt;
        double filteredDeriv = dAlpha * rawDeriv
                + (1 - dAlpha) * state.lastFilteredDeriv;
        double dTerm = kd * filteredDeriv;

        state.lastError = error;
        state.lastFilteredDeriv = filteredDeriv;

        return (pTerm + iTerm + dTerm) * motorPowerGain;
    }
    //endregion

    //region Initialization
    public SplineTracker(Robot robot) {
        this.robot = robot;
        robot.odoDrivetrain.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.odoDrivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.odo.resetPosAndIMU();
    }

    public SplineTracker reset() {
        robot.odo.resetPosAndIMU();
        return this;
    }

    public SplineTracker setPose(Pose2D pose) {
        robot.odo.setGlobalPose(pose);
        return this;
    }

    /**
     * 绑定路径级时间表。绑定后每次 {@link #addPoint} 按调用顺序取用对应段的调度，
     * 运行期用牛顿法投影出的实际进度查这张时间表（进度时钟）。
     *
     * @param timeline 由 {@code SplineTrajectoryLoader} 预先排好的整条路径时间表
     */
    public SplineTracker setTimeline(PathTimeline timeline) {
        this.timeline = timeline;
        this.timelineSegment = -1;
        return this;
    }

    /** 把 {@code SCHED_*} 参数同步到 {@link PathTimeline}；排时间表前调用。 */
    public static void applyScheduleConfig() {
        PathTimeline.V_MAX = SCHED_V_MAX;
        PathTimeline.A_MAX = SCHED_A_MAX;
        PathTimeline.CAP_FLOOR = SCHED_CAP_FLOOR;
        PathTimeline.A_LAT = SCHED_A_LAT;
        PathTimeline.TARGET_PATH_TIME_S = SCHED_TARGET_TIME_S;
        PathTimeline.TANGENT_SPEED_MODE = SCHED_TANGENT_MODE;
    }
    //endregion

    //region Start Robot
    public SplineTracker startMove() {
        return this.startMove(0, 0, 0, 0, () -> {});
    }

    public SplineTracker startMove(Runnable fn) {
        return this.startMove(0, 0, 0, 0, fn);
    }

    public SplineTracker startMove(double x, double y) {
        return this.startMove(x, y, 0, 0, () -> {});
    }

    public SplineTracker startMove(double x, double y, Runnable fn) {
        return this.startMove(x, y, 0, 0, fn);
    }

    public SplineTracker startMove(double x, double y, double dx, double dy) {
        return this.startMove(x, y, dx, dy, () -> {});
    }

    public SplineTracker startMove(double[] point) {
        return this.startMove(point[0], point[1], point[2], point[3]);
    }

    public SplineTracker startMove(double x, double y, double dx, double dy, Runnable fn) {
        startPoint = new double[]{x, y, dx, dy};
        this.x = x;
        this.y = y;
        this.dx = dx;
        this.dy = dy;
        this.fn = fn;
        this.timelineSegment = -1;   // 新路径：段序从头开始
        if ((!Globals.DEBUG || DEBUG_RUN_FUNC) && !DO_NOT_RUN_FUNC) {
            executeTask("startMove[0]", fn);
        }
        this.heading = getHeading();
        this.preH = getHeading();

        runtime.reset();

        return this;
    }
    //endregion

    //region Control
    public SplineTracker stopMotor() {
        robot.odoDrivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.odoDrivetrain.stopMotor();
        return this;
    }

    //region Clock-driven u→t mapping (breaks zero-speed deadlock)
    /**
     * 预计算沿 spline 的速度曲线，生成 u→t / u→v 映射表。
     * 模拟加速度限制 + 减速度距离帽，数值积分得出段内任意时刻的目标 u。
     */
    private void precomputeTimeMap(double[] splineX, double[] splineY,
                                   double segMaxDeriv, double segTotalArc) {
        int N = TIME_MAP_SAMPLES;
        double[] cumArc = new double[N + 1];
        double[] timeMap = new double[N + 1];
        double[] velMap = new double[N + 1];

        // Pass 1: cumulative arc length
        cumArc[0] = 0;
        double prevSx = spline_get(splineX, 0);
        double prevSy = spline_get(splineY, 0);
        for (int i = 1; i <= N; i++) {
            double u = (double) i / N;
            double sx = spline_get(splineX, u);
            double sy = spline_get(splineY, u);
            cumArc[i] = cumArc[i - 1] + Math.hypot(sx - prevSx, sy - prevSy);
            prevSx = sx;
            prevSy = sy;
        }
        double totalArc = cumArc[N];

        // Pass 2: forward velocity-profile simulation
        timeMap[0] = 0;
        velMap[0] = 0;
        double prevV = 0;

        for (int i = 1; i <= N; i++) {
            double u = (double) i / N;

            // Derivative magnitude at this u
            double ds_du = Math.hypot(splineDerivative(splineX, u),
                                       splineDerivative(splineY, u));
            if (ds_du < 1e-6) {
                ds_du = segTotalArc;          // degenerate waypoint fallback
                if (ds_du < 1e-6) ds_du = 1.0;
            }

            // Raw target velocity from curvature ratio
            double rawTarget = (segMaxDeriv > 1e-6)
                    ? (ds_du / segMaxDeriv) * MAX_TANGENTIAL_VEL
                    : 0;

            // Deceleration distance cap
            double remainingArc = totalArc - cumArc[i];
            double maxVelDist = Math.sqrt(2.0 * MAX_ACCEL * remainingArc);
            double targetV = Math.min(rawTarget, maxVelDist);

            // Arc length of this u-step
            double ds = cumArc[i] - cumArc[i - 1];

            // Estimate dt from previous velocity, apply acceleration limit
            double vEst = Math.max(prevV, 0.001);
            double dtEst = ds / vEst;
            double maxDV = MAX_ACCEL * dtEst;
            double desiredV = prevV + Math.max(-maxDV, Math.min(maxDV, targetV - prevV));

            // Refine dt with average velocity
            double vAvg = (prevV + desiredV) / 2.0;
            if (vAvg < 0.001) vAvg = 0.001;
            double dt = ds / vAvg;

            // Re-apply accel limit with refined dt
            maxDV = MAX_ACCEL * dt;
            desiredV = prevV + Math.max(-maxDV, Math.min(maxDV, targetV - prevV));

            timeMap[i] = timeMap[i - 1] + dt;
            velMap[i] = desiredV;
            prevV = desiredV;
        }

        this.timeMapCache = timeMap;
        this.velMapCache = velMap;
        this.segmentTotalTime = timeMap[N];
    }

    /**
     * 二分查找 elapsed 时间对应的 u 值，线性插值。
     */
    private double lookupU(double elapsed) {
        double[] tm = this.timeMapCache;
        if (tm == null) return 0;
        int N = tm.length - 1;
        if (elapsed <= 0) return 0;
        if (elapsed >= tm[N]) return 1.0;

        int lo = 0, hi = N;
        while (lo < hi - 1) {
            int mid = (lo + hi) >>> 1;
            if (tm[mid] <= elapsed) lo = mid;
            else hi = mid;
        }

        double frac = (elapsed - tm[lo]) / (tm[hi] - tm[lo]);
        return (lo + frac) / N;
    }
    //endregion

    public SplineTracker addPoint(double x, double y, double dx, double dy) {
        return this.addPoint(x, y, dx, dy, () -> {});
    }

    public SplineTracker addPoint(double x, double y, double dx, double dy, Runnable fn) {
        return this.addPoint(x, y, dx, dy, this.heading, fn);
    }

    public SplineTracker addPoint(double x, double y, double dx, double dy, double heading) {
        return this.addPoint(x, y, dx, dy, heading, () -> {});
    }

    public SplineTracker addPoint(double x, double y, double dx, double dy, double heading, Runnable fn) {
        // Record last point (start of this spline segment)
        this.lx = this.x;
        this.ly = this.y;
        this.ldx = this.dx;
        this.ldy = this.dy;

        // Set target
        this.x = x;
        this.y = y;
        this.dx = dx;
        this.dy = dy;
        this.fn = fn;

        // Build position splines
        double[] splineX = spline_fit(this.lx, this.ldx, this.x, this.dx);
        double[] splineY = spline_fit(this.ly, this.ldy, this.y, this.dy);

        // Unwrap target heading to shortest path from current heading
        double unwrappedHeading = heading;
        double hDiff = heading - this.heading;
        while (hDiff > 180) { unwrappedHeading -= 360; hDiff -= 360; }
        while (hDiff <= -180) { unwrappedHeading += 360; hDiff += 360; }

        double[] splineH = spline_fit(this.heading, 0, unwrappedHeading, 0);

        // Precompute segment max derivative magnitude for velocity normalization
        segmentMaxDerivative = 0;
        for (int i = 0; i <= DERIVATIVE_SAMPLE_COUNT; i++) {
            double u = (double) i / DERIVATIVE_SAMPLE_COUNT;
            double sdx = splineDerivative(splineX, u);
            double sdy = splineDerivative(splineY, u);
            double mag = Math.hypot(sdx, sdy);
            if (mag > segmentMaxDerivative) segmentMaxDerivative = mag;
        }

        // Pre-compute segment arc length for deceleration distance cap
        {
            double prevSx = spline_get(splineX, 0);
            double prevSy = spline_get(splineY, 0);
            segmentArcLength = 0;
            for (int i = 1; i <= DERIVATIVE_SAMPLE_COUNT; i++) {
                double u = (double) i / DERIVATIVE_SAMPLE_COUNT;
                double sx = spline_get(splineX, u);
                double sy = spline_get(splineY, u);
                segmentArcLength += Math.hypot(sx - prevSx, sy - prevSy);
                prevSx = sx; prevSy = sy;
            }
        }

        // ---- 路径级时间表：取本段的调度 ----
        // 有调度且开启进度时钟 → 用调度时长；否则必须维护段内 u→t 映射，
        // 不然 lookupU() 会读到 null（首段）或上一段的陈旧表，A/B 开关会失效。
        timelineSegment++;
        PathTimeline.Segment sched = (timeline != null && timelineSegment >= 0
                && timelineSegment < timeline.segmentCount())
                ? timeline.segment(timelineSegment) : null;
        final boolean useProgressClock = (sched != null) && USE_PROGRESS_CLOCK;
        if (useProgressClock) {
            segmentTotalTime = sched.duration();
        } else {
            // Pre-compute clock-driven u→t mapping (breaks zero-speed deadlock)
            precomputeTimeMap(splineX, splineY, segmentMaxDerivative, segmentArcLength);
        }

        // Reset velocity-profile and position-PID state for this segment
        prevDesiredVel = 0;
        prevTime = 0;
        posXState.reset();
        posYState.reset();

        double segmentStartTime = runtime.seconds();
        stallTimer.reset();
        stallLastU = 0;
        lastUStar = 0;
        double virtualClock = 0;                       // 段内虚拟时钟（秒）
        double lastClockTime = segmentStartTime;       // 混合时钟的墙钟锚点

        while (robot.opMode.opModeIsActive()) {
            robot.odo.update();

            double curH = getHeading();
            double rx = getX();
            double ry = getY();

            double rawMotorPowerGain = Math.abs(motorVoltage / robot.getVoltage());
            double motorPowerGain = 1.0 + (rawMotorPowerGain - 1.0) * VOLTAGE_COMP_WEIGHT;

            // ── 实际进度：牛顿法把机器人投影到样条上（唯一衡量"跑了多少"的量） ──
            // 单调钳制：findClosestU 是无记忆的全局最近点搜索，折返/U 形段上可能跨周期跳到
            // 另一支；进度时钟用它反查时间，一旦回退会把目标点也往回拽。
            double uStar = findClosestU(splineX, splineY, rx, ry);
            if (uStar > lastUStar) lastUStar = uStar;
            else uStar = lastUStar;

            // ── u_target ──
            // 进度时钟（新）：用实际进度 u* 查时间表得到"虚拟已用时"，
            //   再加前视窗口 —— 即用牛顿法进度替换固定计时器。
            // 墙钟时钟（旧）：uTarget = lookupU(墙钟 + 前视)，时间表来自段内 precomputeTimeMap。
            // 两种时钟都只有"位置误差 → PID"，没有任何速度前馈。
            double lookaheadS = LOOKAHEAD_MS / 1000.0;
            double elapsed;
            double uTarget;
            if (useProgressClock) {
                double progressTime = sched.timeAtU(uStar);
                if (SCHED_HYBRID_LEAD_S > 0) {
                    // 限幅混合时钟：按真实时间推进，但最多领先实际进度 SCHED_HYBRID_LEAD_S 秒
                    double nowT = runtime.seconds();
                    double advanced = virtualClock + Math.max(0, nowT - lastClockTime);
                    lastClockTime = nowT;
                    virtualClock = Math.min(advanced, progressTime + SCHED_HYBRID_LEAD_S);
                } else {
                    virtualClock = progressTime;          // 纯进度时钟
                }
                elapsed = virtualClock;
                uTarget = sched.uAtTime(elapsed + lookaheadS);
            } else {
                elapsed = runtime.seconds() - segmentStartTime;
                uTarget = (segmentTotalTime < 0.001 || elapsed >= segmentTotalTime)
                        ? 1.0
                        : lookupU(elapsed + lookaheadS);
            }
            // 兜底：目标点永远不落后于实际进度，否则位置 PID 会把车往回拽
            if (uTarget < uStar) uTarget = uStar;

            // ── 进度停滞保护：零进度超过 STALL_TIMEOUT_S 就刹车并视为到达 ──
            if (STALL_TIMEOUT_S > 0) {
                if (uStar > stallLastU + 1e-4) {
                    stallLastU = uStar;
                    stallTimer.reset();
                } else if (stallTimer.seconds() > STALL_TIMEOUT_S) {
                    Log.w("SplineTracker", "[STALL] seg=" + timelineSegment
                            + " no progress for " + STALL_TIMEOUT_S + "s at u=" + uStar
                            + " -> brake & treat as arrived");
                    stopMotor();
                    break;
                }
            }

            // ── Velocity profile at uStar (telemetry / monitoring only) ──
            double currDerivMag = Math.hypot(splineDerivative(splineX, uStar),
                                              splineDerivative(splineY, uStar));
            if (currDerivMag < 1e-6) {
                currDerivMag = Math.hypot(this.x - this.lx, this.y - this.ly);
                if (currDerivMag < 1e-6) currDerivMag = 1.0;
            }
            double rawTargetVel = (segmentMaxDerivative > 1e-6)
                    ? (currDerivMag / segmentMaxDerivative) * MAX_TANGENTIAL_VEL
                    : 0;

            // Time step for position PID
            double currentTime = System.nanoTime() / 1e9;
            double dt = currentTime - prevTime;
            if (dt <= 0 || dt > 0.5) dt = 0.02;
            prevTime = currentTime;

            // Velocity profile at uStar (monitoring: what speed the robot's position calls for)
            double remainingArc = segmentArcLength * (1.0 - uStar);
            double maxVelFromDist = Math.sqrt(2.0 * MAX_ACCEL * remainingArc);
            double targetVel = Math.min(rawTargetVel, maxVelFromDist);
            double maxDeltaV = MAX_ACCEL * dt;
            double desiredVel = prevDesiredVel
                    + Math.max(-maxDeltaV, Math.min(maxDeltaV, targetVel - prevDesiredVel));
            prevDesiredVel = desiredVel;

            // Expected position at look-ahead point
            double targetX = spline_get(splineX, uTarget);
            double targetY = spline_get(splineY, uTarget);
            double targetH = spline_get(splineH, uTarget);

            // ── New: position-error PID (X and Y axes independently) ──
            double ex = targetX - rx;
            double ey = targetY - ry;
            double vx = computePositionPID(ex, dt, K_POS_P, K_POS_I, K_POS_D,
                    POS_I_WINDOW, POS_I_MAX, POS_D_ALPHA, POS_DEAD_ZONE,
                    posXState, motorPowerGain);
            double vy = computePositionPID(ey, dt, K_POS_P, K_POS_I, K_POS_D,
                    POS_I_WINDOW, POS_I_MAX, POS_D_ALPHA, POS_DEAD_ZONE,
                    posYState, motorPowerGain);

            // Heading control (unchanged)
            double headingError = angleDiff(targetH, curH);
            double yawPower = headingError * K_HEADING * motorPowerGain;
            if (Math.abs(headingError) < TURN_DEAD_AREA) yawPower = 0;

            vx = Range.clip(vx, -1, 1);
            vy = Range.clip(vy, -1, 1);
            yawPower = Range.clip(yawPower, -1, 1);

            // Drive robot
            if (!Globals.DEBUG)
                robot.odoDrivetrain.driveRobotFieldCentric(
                        vx, -vy, -yawPower
                );

            // FTC Dashboard
            if (Globals.DEBUG) robot.sleep(200);

            // Telemetry
            robot.telemetry.addData("u*", uStar);
            robot.telemetry.addData("uTarget", uTarget);
            robot.telemetry.addData("targetX", targetX);
            robot.telemetry.addData("targetY", targetY);
            robot.telemetry.addData("desiredH", targetH);
            robot.telemetry.addData("errorX", ex);
            robot.telemetry.addData("errorY", ey);
            robot.telemetry.addData("vxCmd", vx);
            robot.telemetry.addData("vyCmd", vy);
            robot.telemetry.addData("desiredVel", desiredVel);
            robot.telemetry.addData("elapsed", String.format("%.2f", elapsed));
            robot.telemetry.addData("segTime", String.format("%.2f", segmentTotalTime));
            robot.telemetry.addData("clock", useProgressClock
                    ? (SCHED_HYBRID_LEAD_S > 0 ? "progress+hybrid" : "progress") : "wall");
            if (!useProgressClock && velMapCache != null && uTarget < 1.0)
                robot.telemetry.addData("clockVel", String.format("%.1f",
                        velMapCache[Math.min((int)(uTarget * TIME_MAP_SAMPLES), velMapCache.length - 1)]));
            robot.telemetry.addLine();

            robot.telemetry.addData("X_POS", rx);
            robot.telemetry.addData("Y_POS", ry);
            robot.telemetry.addData("Heading", curH);
            robot.telemetry.addData("Voltage", robot.getVoltage());
            robot.telemetry.addData("MOTOR_GAIN", motorPowerGain);
            robot.telemetry.addData("Odo", robot.odo.getPosition().toString());

            if (LOG_VERBOSE) {
                Log.i("SplineTracker", "[XPow] " + vx);
                Log.i("SplineTracker", "[YPow] " + vy);
                Log.i("SplineTracker", "[ZPow] " + -yawPower);
                Log.i("SplineTracker", "[uTarget] " + uTarget);
                Log.i("SplineTracker", "[desiredVel] " + desiredVel + " in/s");
                Log.i("SplineTracker", "[elapsed] " + String.format("%.2f", elapsed) + "/" + String.format("%.2f", segmentTotalTime) + "s");
            }

            // Store for accessors
            gotoX = targetX;
            gotoY = targetY;
            gotoH = targetH;

            // Exit condition: reached end of path segment
            // 退化段（零弧长，例如"原地转向"关键帧）不能靠 u* 判定：全部采样点重合，
            // projectU 恒返回 0（HEAD 用 MAX_SEGMENT_TIMEOUT 兜住，工作区把它删掉了）。
            double endPosError = Math.hypot(this.x - rx, this.y - ry);
            boolean atEnd = (uStar >= END_U_THRESHOLD) || (segmentArcLength < 0.05);
            boolean done = atEnd
                    && endPosError < END_POS_THRESHOLD
                    && Math.abs(headingError) < END_HEADING_THRESHOLD;
            if (done) {
                // Zero-vel endpoint: actively brake so the robot doesn't coast
                boolean endVelIsZero = Math.abs(this.dx) < 0.1 && Math.abs(this.dy) < 0.1;
                if (endVelIsZero) stopMotor();
                break;
            }
        }

        this.heading = unwrappedHeading;
        this.preH = unwrappedHeading;

        // 段末任务默认阻塞执行（ASYNC_TASKS=false），期间控制回路不再刷新：
        // 必须先清零驱动，否则机器人会带着最后一次功率冲出去。
        if (!Globals.DEBUG) robot.odoDrivetrain.driveRobotFieldCentric(0, 0, 0);

        // Run function at end of segment
        if ((!Globals.DEBUG || DEBUG_RUN_FUNC) && !DO_NOT_RUN_FUNC) {
            executeTask("addPoint:segment-end", fn);
        } else {
            Log.w("SplineTracker", "[BRK4] addPoint: task SKIPPED — DEBUG=" + Globals.DEBUG
                    + ", DEBUG_RUN_FUNC=" + DEBUG_RUN_FUNC + ", DO_NOT_RUN_FUNC=" + DO_NOT_RUN_FUNC);
        }

        return this;
    }

    public SplineTracker addVelocity(double dx, double dy) {
        this.ldx = this.dx;
        this.ldy = this.dy;
        this.dx = dx;
        this.dy = dy;
        return this;
    }

    public SplineTracker addFunc(Runnable fn) {
        if ((!Globals.DEBUG || DEBUG_RUN_FUNC) && !DO_NOT_RUN_FUNC) {
            executeTask("addFunc", fn);
        } else {
            Log.w("SplineTracker", "[BRK4] addFunc: task SKIPPED — DEBUG=" + Globals.DEBUG
                    + ", DEBUG_RUN_FUNC=" + DEBUG_RUN_FUNC + ", DO_NOT_RUN_FUNC=" + DO_NOT_RUN_FUNC);
        }
        return this;
    }

    /**
     * 执行 task，默认阻塞（{@link #ASYNC_TASKS}=false）。
     * 设为 true 时恢复旧的异步 fire-and-forget 行为。
     */
    private void executeTask(String location, Runnable fn) {
        if (ASYNC_TASKS) {
            Log.d("SplineTracker", "[BRK4-ASYNC] " + location + ": fire-and-forget");
            TaskLoopFrame.runOnce(fn);
        } else {
            Log.d("SplineTracker", "[BRK4-BLOCK] " + location + ": start");
            try {
                fn.run();
                Log.d("SplineTracker", "[BRK4-BLOCK] " + location + ": done");
            } catch (Exception e) {
                Log.e("SplineTracker", "[BRK4-BLOCK] " + location + ": exception", e);
            }
        }
    }

    public SplineTracker addTime(double seconds) {
        return this.addTime(seconds, () -> {});
    }

    public SplineTracker addTime(double seconds, Runnable fn) {
        double startWait = runtime.seconds();

        while (runtime.seconds() - startWait < seconds && robot.opMode.opModeIsActive()) {
            robot.odo.update();

            double curH = getHeading();
            double headingError = angleDiff(this.heading, curH);
            double yawPower = headingError * K_HEADING;
            if (Math.abs(headingError) < TURN_DEAD_AREA) yawPower = 0;
            yawPower = Range.clip(yawPower, -1, 1);

            if (!Globals.DEBUG)
                robot.odoDrivetrain.driveRobotFieldCentric(0, 0, -yawPower);
            else
                robot.sleep(200);

            robot.telemetry.addData("addTime", seconds);
            robot.telemetry.addData("remaining", String.format("%.2f", seconds - (runtime.seconds() - startWait)));
            robot.telemetry.addLine();
        }

        if ((!Globals.DEBUG || DEBUG_RUN_FUNC) && !DO_NOT_RUN_FUNC) {
            executeTask("addTime:hold-end", fn);
        } else {
            Log.w("SplineTracker", "[BRK4] addTime: task SKIPPED — DEBUG=" + Globals.DEBUG
                    + ", DEBUG_RUN_FUNC=" + DEBUG_RUN_FUNC + ", DO_NOT_RUN_FUNC=" + DO_NOT_RUN_FUNC);
        }

        return this;
    }

    /** 注意：本质是 addPoint（位置不变、只改朝向），会占用一个时间表段号。 */
    public SplineTracker addHeading(double heading) {
        return this.addHeading(heading, () -> {});
    }

    public SplineTracker addHeading(double heading, Runnable fn) {
        return this.addPoint(this.x, this.y, this.dx, this.dy, heading, fn);
    }
    //endregion

    //region Get Data
    public double getX() {
        return this.getX(DistanceUnit.INCH);
    }

    public double getX(DistanceUnit unit) {
        return robot.odo.getPosition().getX(unit);
    }

    public double getY() {
        return this.getY(DistanceUnit.INCH);
    }

    public double getY(DistanceUnit unit) {
        return robot.odo.getPosition().getY(unit);
    }

    public double getHeading() {
        return this.getHeading(AngleUnit.DEGREES);
    }

    public double getHeading(AngleUnit unit) {
        return robot.odo.getPosition().getHeading(unit);
    }

    public Pose2D getPosition() {
        return robot.odo.getPosition();
    }

    public double[] getStartPoint() {
        return this.startPoint;
    }

    public double[] getGotoPoint() {
        return new double[]{this.x, this.y, this.dx, this.dy};
    }

    public double[] getCurrentGotoPoint() {
        return new double[]{gotoX, gotoY};
    }

    public double[] getPreviousPoint() {
        return new double[]{this.lx, this.ly, this.ldx, this.ldy};
    }
    //endregion

    public Robot getRobot() {
        return robot;
    }
}
