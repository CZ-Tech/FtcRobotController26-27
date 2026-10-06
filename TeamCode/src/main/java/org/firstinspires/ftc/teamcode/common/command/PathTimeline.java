package org.firstinspires.ftc.teamcode.common.command;

import java.util.List;
import java.util.Locale;

/**
 * 全路径时间参数化（纯数学，无 Android 依赖）。
 *
 * <p>作用：把一串关键帧先算成"整条路径的时间表"，供 {@link SplineTracker} 用
 * <b>实际进度（牛顿法投影）查询虚拟时钟</b>，替代旧版固定计时器
 * （{@code PinpointTrajectory} 的 {@code runtime.seconds()} / 段内
 * {@code precomputeTimeMap}）驱动的做法。</p>
 *
 * <h3>流程</h3>
 * <ol>
 *   <li>关键帧两两之间做三次 Hermite（端点位置 + 切向量 {@code dx/dy}），heading 单独一条样条，
 *       并按最短路径解缠；</li>
 *   <li>按弧长密集采样，得到 {@code s[j]}；</li>
 *   <li>速度上限：{@code |(dx,dy)|}（导出器给出的速度意图，见 {@link #TANGENT_SPEED_MODE}）、
 *       {@link #V_MAX}、可选曲率限制 {@link #A_LAT}；</li>
 *   <li>前向 + 反向各扫一遍，保证 {@code |dv/dt| <= A_MAX}，起点静止、终点按末点切向量
 *       决定"停住"还是"带速度穿过"；</li>
 *   <li>积分出基础时间表；</li>
 *   <li>JSON 里显式给出的 {@code duration} 作为"该段请求时长"：
 *       最终段时长 = {@code max(物理可行最短, duration)}，即 duration 只可能让段变慢；</li>
 *   <li>{@link #TARGET_PATH_TIME_S} 再把整条路径时间整体缩放（同样只变慢）。</li>
 * </ol>
 *
 * <h3>与 PinpointTrajectory 的关系</h3>
 * <p>这里算出的时间轴就是 PinpointTrajectory TIMELINE 模式里"重分配后的绝对时间"，
 * 区别只在于时间不是手写 duration，而是从几何 + 加减速限制反解出来的；运行期由
 * {@link SplineTracker} 用牛顿法投影出的实际进度去查这张表。</p>
 *
 * <p><b>注意</b>：本类刻意不加 {@code @Config}（保持可离线 javac 编译），
 * 需要在 FTC Dashboard 上调参的旋钮放在 {@link SplineTracker} 的 {@code SCHED_*} 静态字段上，
 * 由 {@link SplineTracker#applyScheduleConfig()} 同步过来。</p>
 */
public class PathTimeline {

    // ------------------------------------------------------------------
    // 可调参数
    // ------------------------------------------------------------------

    /** 切向速度上限 (in/s)。 */
    public static double V_MAX = 50.0;
    /** 切向加速度上限 (in/s^2)，正向/反向扫描都用它。 */
    public static double A_MAX = 50.0;
    /** 速度上限的下限 (in/s)，防止 dx/dy 未按 in/s 标定时整段被拖成爬行。 */
    public static double CAP_FLOOR = 2.0;
    /** 横向加速度上限 (in/s^2)，>0 时额外加一条 v <= sqrt(A_LAT / kappa)。 */
    public static double A_LAT = 0.0;
    /** >0 时把整条路径时间缩放到该秒数（只变慢，因此不会破坏加速度限制）。 */
    public static double TARGET_PATH_TIME_S = 0.0;

    /** 每段最少采样区间数。 */
    public static int MIN_INTERVALS = 64;
    /** 每段最多采样区间数。 */
    public static int MAX_INTERVALS = 400;
    /** 估算段长用的采样密度（区间 / 英寸）。 */
    public static double INTERVALS_PER_INCH = 3.0;

    /** {@code |(dx,dy)|} 直接当作 in/s 速度上限。 */
    public static final int TANGENT_ABSOLUTE = 0;
    /** {@code |(dx,dy)|} 只作为形状信号：段内最大切向量 ↔ {@link #V_MAX}。 */
    public static final int TANGENT_RELATIVE = 1;
    /** 完全忽略切向量幅值，只受 {@link #V_MAX} / {@link #A_MAX} 约束。 */
    public static final int TANGENT_OFF = 2;

    /**
     * {@code |(dx,dy)|} 的用法。0 视为"未指定"（不会因此强制停车）。
     * <p>默认 {@link #TANGENT_ABSOLUTE}：幅值直接当作 in/s 速度上限。
     * {@link #TANGENT_RELATIVE}（只让幅值的相对形状决定快慢）与 {@link #TANGENT_OFF}
     * （忽略幅值，只受 V_MAX/A_MAX 约束）留给"导出器幅值不是 in/s"的情况。</p>
     * <p>量级自查：用 {@code tools/PathSchedulePreview} 跑一遍真实路线。本仓库旧
     * B_Far6Ball 数据下三种模式分别排出 43.45s / 10.72s / 7.55s —— 若某条路径排出来
     * 的总时间远超预期，就说明这批 dx/dy 幅值不适合当 in/s 用。</p>
     */
    public static int TANGENT_SPEED_MODE = TANGENT_ABSOLUTE;

    /** 每个路径打印一条时间表摘要日志（不是每周期）。 */
    public static boolean LOG_SCHEDULE = true;

    private static final double EPS = 1e-9;
    private static final double TANGENT_EPS = 1e-6;
    private static final double V_EPS = 1e-6;

    // ------------------------------------------------------------------
    // 数据
    // ------------------------------------------------------------------

    /** 一个运动关键帧。 */
    public static final class Waypoint {
        public final double x;
        public final double y;
        public final double dx;
        public final double dy;
        public final double heading;
        /** JSON duration：该段（上一关键帧 → 本关键帧）的请求时长，<=0 表示未指定。 */
        public final double requestedDuration;

        public Waypoint(double x, double y, double dx, double dy,
                        double heading, double requestedDuration) {
            this.x = x;
            this.y = y;
            this.dx = dx;
            this.dy = dy;
            this.heading = heading;
            this.requestedDuration = requestedDuration;
        }
    }

    private final int segCount;
    private final double[][] segX;
    private final double[][] segY;
    private final double[][] segH;
    private final int[] firstSample;
    private final int[] intervals;
    /** 每个采样的累计弧长 (in)。 */
    private final double[] s;
    /** 每个采样的累计调度时间 (s)，已含 duration 覆盖与总时间缩放。 */
    private final double[] t;
    /** 每个采样的速度上限/规划速度 (in/s)，仅用于诊断与离线自测。 */
    private final double[] v;
    private final Segment[] segments;
    private final double totalArc;
    private final double totalTime;

    public PathTimeline(List<Waypoint> wps) {
        this(wps, 0.0);
    }

    public PathTimeline(List<Waypoint> wps, double targetTotalTimeOverride) {
        int n = (wps == null) ? 0 : wps.size();
        int sc = Math.max(0, n - 1);

        this.segCount = sc;
        this.segX = new double[sc][];
        this.segY = new double[sc][];
        this.segH = new double[sc][];
        this.firstSample = new int[sc];
        this.intervals = new int[sc];

        if (sc == 0) {
            this.s = new double[]{0};
            this.t = new double[]{0};
            this.v = new double[]{0};
            this.segments = new Segment[0];
            this.totalArc = 0;
            this.totalTime = 0;
            return;
        }

        // ---- 1. 每段几何 + 采样密度 ----
        int[] m = new int[sc];
        double[] requested = new double[sc];
        double prevTargetH = wps.get(0).heading;
        for (int i = 0; i < sc; i++) {
            Waypoint a = wps.get(i);
            Waypoint b = wps.get(i + 1);
            segX[i] = fit(a.x, a.dx, b.x, b.dx);
            segY[i] = fit(a.y, a.dy, b.y, b.dy);
            double target = unwrap(b.heading, prevTargetH);
            segH[i] = fit(prevTargetH, 0, target, 0);
            prevTargetH = target;
            requested[i] = b.requestedDuration;

            double len = 0;
            double px = eval(segX[i], 0);
            double py = eval(segY[i], 0);
            for (int j = 1; j <= 16; j++) {
                double u = j / 16.0;
                double cx = eval(segX[i], u);
                double cy = eval(segY[i], u);
                len += Math.hypot(cx - px, cy - py);
                px = cx;
                py = cy;
            }
            m[i] = (int) Math.round(len * INTERVALS_PER_INCH);
            if (m[i] < MIN_INTERVALS) m[i] = MIN_INTERVALS;
            if (m[i] > MAX_INTERVALS) m[i] = MAX_INTERVALS;
            if (m[i] < 1) m[i] = 1;   // 防止手工把 MIN_INTERVALS 设为 0 后出现 0/0 = NaN
        }

        // 段 i 覆盖采样下标 [firstSample[i], firstSample[i] + m[i]]，
        // 段边界采样与相邻段共用（idx(i, m[i]) == idx(i+1, 0)）。
        int total = 1;
        for (int i = 0; i < sc; i++) {
            firstSample[i] = total - 1;
            intervals[i] = m[i];
            total += m[i];
        }

        double[] sArr = new double[total];
        double[] vArr = new double[total];
        double[] ds = new double[total];

        // ---- 2. 采样 + 速度上限 ----
        double arc = 0;
        for (int i = 0; i < sc; i++) {
            Waypoint a = wps.get(i);
            Waypoint b = wps.get(i + 1);
            double t0 = Math.hypot(a.dx, a.dy);
            double t1 = Math.hypot(b.dx, b.dy);
            double tMax = Math.max(t0, t1);

            double px = eval(segX[i], 0);
            double py = eval(segY[i], 0);
            for (int j = (i == 0 ? 0 : 1); j <= m[i]; j++) {
                int idx = firstSample[i] + j;
                double u = (double) j / m[i];
                double cx = eval(segX[i], u);
                double cy = eval(segY[i], u);
                if (j > 0) {
                    ds[idx] = Math.hypot(cx - px, cy - py);
                    arc += ds[idx];
                }
                sArr[idx] = arc;
                vArr[idx] = speedCap(t0, t1, tMax, u, segX[i], segY[i]);
                px = cx;
                py = cy;
            }
        }

        // ---- 3. 首尾速度：起点静止；末点按切向量决定停住 or 带速度穿过 ----
        double vEndTangent = Math.hypot(wps.get(n - 1).dx, wps.get(n - 1).dy);
        vArr[0] = 0.0;
        vArr[total - 1] = (vEndTangent > TANGENT_EPS && TANGENT_SPEED_MODE != TANGENT_OFF)
                ? vArr[total - 1] : 0.0;

        // ---- 4. 双向加减速扫描：|dv/dt| <= A_MAX ----
        for (int j = 1; j < total; j++) {
            double lim = Math.sqrt(vArr[j - 1] * vArr[j - 1] + 2 * A_MAX * ds[j]);
            if (lim < vArr[j]) vArr[j] = lim;
        }
        for (int j = total - 2; j >= 0; j--) {
            double lim = Math.sqrt(vArr[j + 1] * vArr[j + 1] + 2 * A_MAX * ds[j + 1]);
            if (lim < vArr[j]) vArr[j] = lim;
        }

        // ---- 5. 基础时间表 ----
        double[] tArr = new double[total];
        for (int j = 1; j < total; j++) {
            double denom = Math.max(vArr[j - 1], V_EPS) + Math.max(vArr[j], V_EPS);
            tArr[j] = tArr[j - 1] + 2.0 * ds[j] / denom;
        }

        // ---- 6. duration 覆盖：只把段变慢 ----
        double[] tScaled = new double[total];
        int a0 = 0;
        for (int i = 0; i < sc; i++) {
            int b0 = firstSample[i] + intervals[i];
            double tMin = tArr[b0] - tArr[a0];
            double scale = 1.0;
            if (requested[i] > 0 && tMin > EPS && requested[i] > tMin) {
                scale = requested[i] / tMin;
            }
            for (int j = a0 + 1; j <= b0; j++) {
                tScaled[j] = tScaled[a0] + (tArr[j] - tArr[a0]) * scale;
            }
            a0 = b0;
        }

        // ---- 7. 整条路径时间缩放（只变慢） ----
        double target = (targetTotalTimeOverride > 0) ? targetTotalTimeOverride : TARGET_PATH_TIME_S;
        double tEnd = tScaled[total - 1];
        if (target > 0 && tEnd > EPS && target > tEnd) {
            double k = target / tEnd;
            for (int j = 1; j < total; j++) tScaled[j] *= k;
            tEnd = target;
        }

        this.s = sArr;
        this.v = vArr;
        this.t = tScaled;
        this.totalArc = arc;
        this.totalTime = tEnd;
        this.segments = new Segment[sc];
        for (int i = 0; i < sc; i++) this.segments[i] = new Segment(i);
    }

    // ------------------------------------------------------------------
    // 运行期接口（SplineTracker 只用这几个）
    // ------------------------------------------------------------------

    /** 一条段的调度：段内 u ↔ 段内时间，以及几何查询。 */
    public final class Segment {
        private final int index;

        private Segment(int index) {
            this.index = index;
        }

        public int index() {
            return index;
        }

        /** 段首在整条路径时间轴上的绝对时间 (s)。 */
        public double startTime() {
            return t[firstSample[index]];
        }

        /** 段调度时长 (s)，已含 duration 覆盖。 */
        public double duration() {
            return t[firstSample[index] + intervals[index]] - t[firstSample[index]];
        }

        public double arcLength() {
            return s[firstSample[index] + intervals[index]] - s[firstSample[index]];
        }

        /** 段内已用时间：u（0~1）→ [0, duration]。 */
        public double timeAtU(double u) {
            int a = firstSample[index];
            int mm = intervals[index];
            double uu = clamp01(u);
            double idx = a + uu * mm;
            int i0 = (int) Math.floor(idx);
            if (i0 >= a + mm) return t[a + mm] - t[a];
            if (i0 < a) return 0.0;
            double frac = idx - i0;
            double tt = t[i0] + (t[i0 + 1] - t[i0]) * frac;
            return tt - t[a];
        }

        /** 段内时间 → u：虚拟时钟的反查（单调表二分 + 线性插值）。 */
        public double uAtTime(double localElapsed) {
            int a = firstSample[index];
            int mm = intervals[index];
            double target = t[a] + localElapsed;
            if (target <= t[a]) return 0.0;
            if (target >= t[a + mm]) return 1.0;
            int lo = a;
            int hi = a + mm;
            while (lo < hi - 1) {
                int mid = (lo + hi) >>> 1;
                if (t[mid] <= target) lo = mid;
                else hi = mid;
            }
            double span = t[hi] - t[lo];
            double frac = (span > EPS) ? (target - t[lo]) / span : 0.0;
            return (lo - a + frac) / mm;
        }

        public double[] splineX() {
            return segX[index];
        }

        public double[] splineY() {
            return segY[index];
        }

        public double[] splineH() {
            return segH[index];
        }

        public double x(double u) {
            return eval(segX[index], u);
        }

        public double y(double u) {
            return eval(segY[index], u);
        }

        public double heading(double u) {
            return eval(segH[index], u);
        }
    }

    public int segmentCount() {
        return segCount;
    }

    public Segment segment(int i) {
        return segments[i];
    }

    public double totalArcLength() {
        return totalArc;
    }

    public double totalMotionTime() {
        return totalTime;
    }

    /** 采样点总数（诊断 / 离线自测用）。 */
    public int sampleCount() {
        return t.length;
    }

    public double sampleArc(int i) {
        return s[i];
    }

    public double sampleTime(int i) {
        return t[i];
    }

    /** 该采样点的规划速度上限（已含加减速扫描；duration 覆盖只改时间不改它）。 */
    public double sampleSpeed(int i) {
        return v[i];
    }

    /** 一行摘要，便于确认"这条路径排完要多久"。 */
    public String describe() {
        StringBuilder sb = new StringBuilder();
        sb.append("PathTimeline segs=").append(segCount)
                .append(String.format(Locale.US, " arc=%.1fin total=%.2fs offsets=[", totalArc, totalTime));
        for (int i = 0; i < segCount; i++) {
            sb.append(String.format(Locale.US, "%s%.2f", (i == 0 ? "" : ", "), segments[i].startTime()));
        }
        return sb.append("]").toString();
    }

    // ------------------------------------------------------------------
    // 采样 / 几何工具（static，便于离线单测）
    // ------------------------------------------------------------------

    /** 三次 Hermite：端点值与端点切向量。 */
    public static double[] fit(double x0, double dx0, double x1, double dx1) {
        double a = 2 * x0 + dx0 - 2 * x1 + dx1;
        double b = -3 * x0 - 2 * dx0 + 3 * x1 - dx1;
        double c = dx0;
        double d = x0;
        return new double[]{a, b, c, d};
    }

    public static double eval(double[] c, double u) {
        return ((c[0] * u + c[1]) * u + c[2]) * u + c[3];
    }

    public static double deriv(double[] c, double u) {
        return (3 * c[0] * u + 2 * c[1]) * u + c[2];
    }

    public static double secondDeriv(double[] c, double u) {
        return 6 * c[0] * u + 2 * c[1];
    }

    public static double curvature(double[] sx, double[] sy, double u) {
        double dx = deriv(sx, u);
        double dy = deriv(sy, u);
        double ddx = secondDeriv(sx, u);
        double ddy = secondDeriv(sy, u);
        double den = Math.pow(dx * dx + dy * dy, 1.5);
        if (den < 1e-9) return 0;
        return Math.abs(dx * ddy - dy * ddx) / den;
    }

    /**
     * 牛顿法求"机器人位置到样条的最近点"参数 u。
     * <p>先沿样条均匀粗采样找初值，再用极值条件 f'(u)=0 做几次牛顿迭代。
     * 运行期 {@link SplineTracker#findClosestU} 直接调用本方法，保证离线自测
     * 覆盖的就是机器上跑的那份实现。</p>
     */
    public static double projectU(double[] splineX, double[] splineY,
                                  double rx, double ry, int samples, int newtonIters) {
        if (samples < 1) samples = 1;
        double bestU = 0;
        double bestDist = Double.MAX_VALUE;
        for (int i = 0; i <= samples; i++) {
            double u = (double) i / samples;
            double ex = eval(splineX, u) - rx;
            double ey = eval(splineY, u) - ry;
            double dist = ex * ex + ey * ey;
            if (dist < bestDist) {
                bestDist = dist;
                bestU = u;
            }
        }
        for (int iter = 0; iter < newtonIters; iter++) {
            double u = bestU;
            double ex = eval(splineX, u) - rx;
            double ey = eval(splineY, u) - ry;
            double sdx = deriv(splineX, u);
            double sdy = deriv(splineY, u);
            double sddx = secondDeriv(splineX, u);
            double sddy = secondDeriv(splineY, u);
            double fprime = 2 * ex * sdx + 2 * ey * sdy;
            double fsecond = 2 * sdx * sdx + 2 * ex * sddx + 2 * sdy * sdy + 2 * ey * sddy;
            if (Math.abs(fsecond) < 1e-9) break;
            double uNew = u - fprime / fsecond;
            if (uNew < 0) uNew = 0;
            else if (uNew > 1) uNew = 1;
            if (Math.abs(uNew - u) < 1e-6) {
                bestU = uNew;
                break;
            }
            bestU = uNew;
        }
        return bestU;
    }

    // ------------------------------------------------------------------
    // 内部工具
    // ------------------------------------------------------------------

    /**
     * 段的采样点速度上限。
     * <p>{@code |(dx,dy)|} 视为 0 时表示"该端点未指定速度"，不会因此强制停车；
     * 两端都未指定时该段不受切向量约束。</p>
     */
    private static double speedCap(double t0, double t1, double tMax, double u,
                                   double[] sx, double[] sy) {
        double cap = V_MAX;

        if (TANGENT_SPEED_MODE == TANGENT_ABSOLUTE) {
            boolean has0 = t0 > TANGENT_EPS;
            boolean has1 = t1 > TANGENT_EPS;
            if (has0 && has1) cap = t0 + (t1 - t0) * u;
            else if (has0) cap = t0;
            else if (has1) cap = t1;
        } else if (TANGENT_SPEED_MODE == TANGENT_RELATIVE) {
            if (tMax > TANGENT_EPS) cap = V_MAX * ((t0 + (t1 - t0) * u) / tMax);
        }

        if (!(cap <= V_MAX)) cap = V_MAX;   // NaN 也一并夹掉

        if (A_LAT > 0) {
            double k = curvature(sx, sy, u);
            double kCap = Math.sqrt(A_LAT / Math.max(k, EPS));
            if (kCap < cap) cap = kCap;
        }

        return (cap < CAP_FLOOR) ? CAP_FLOOR : cap;
    }

    private static double clamp01(double u) {
        if (u < 0) return 0;
        if (u > 1) return 1;
        return u;
    }

    /** 把 target 解缠到与 base 相差 180° 以内。 */
    private static double unwrap(double target, double base) {
        double h = target - base;
        while (h > 180) {
            target -= 360;
            h -= 360;
        }
        while (h <= -180) {
            target += 360;
            h += 360;
        }
        return target;
    }
}
