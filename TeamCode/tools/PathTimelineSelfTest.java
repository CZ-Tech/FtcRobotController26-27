import org.firstinspires.ftc.teamcode.common.command.PathTimeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * PathTimeline 的离线自测（纯 Java，不依赖 Android / gradle / FTC SDK）。
 *
 * <p>PathTimeline 刻意不引用任何 Android 类，所以可以直接用 javac 编译运行
 * （路径位于 {@code src/main/java} 之外，不会被 Android 构建打包）。</p>
 *
 * <pre>
 * 在仓库根目录执行：
 *   javac -d out Team19656/src/main/java/org/firstinspires/ftc/teamcode/common/command/PathTimeline.java Team19656/tools/PathTimelineSelfTest.java
 *   java -cp out PathTimelineSelfTest
 * </pre>
 *
 * <p>覆盖：解析解总时间、duration 只会变慢、段边界时间连续、u↔t 往返、
 * 前视不落后于实际进度、牛顿法投影、速度/加速度上限、切向量模式切换。</p>
 */
public class PathTimelineSelfTest {

    private static int failures = 0;
    private static int checks = 0;

    public static void main(String[] args) {
        PathTimeline.LOG_SCHEDULE = false;

        testStraightLineAnalyticTime();
        testDurationOverrideOnlySlows();
        testSegmentBoundariesAndStartTimes();
        testUTimeRoundTrip();
        testLookaheadNeverBehind();
        testNewtonProjection();
        testSpeedAndAccelLimits();
        testTangentOffMode();
        testRelativeVsAbsoluteOnSmallTangents();
        testDegenerateSegment();

        System.out.println();
        System.out.println(failures == 0
                ? "ALL PASS (" + checks + " checks)"
                : failures + " FAILURE(S) out of " + checks + " checks");
        if (failures > 0) System.exit(1);
    }

    // ------------------------------------------------------------------
    // 用例
    // ------------------------------------------------------------------

    /** 24in 直线、两端静止：解析解 T = 2*sqrt(L/A)，L=24, A=50 → 1.3856s */
    private static void testStraightLineAnalyticTime() {
        defaults();
        PathTimeline tl = build(
                wp(0, 0, 0, 0, 0, 0),
                wp(24, 0, 0, 0, 0, 0));
        double expected = 2.0 * Math.sqrt(24.0 / 50.0);
        close("直线段总时间", tl.totalMotionTime(), expected, 0.02);
        close("直线段弧长", tl.totalArcLength(), 24.0, 0.05);
        System.out.println("  [straight] " + tl.describe());
    }

    /** duration 只允许把段变慢；比物理最短还短的 duration 被忽略 */
    private static void testDurationOverrideOnlySlows() {
        defaults();

        PathTimeline slow = build(
                wp(0, 0, 0, 0, 0, 0),
                wp(24, 0, 0, 0, 0, 5.0));
        close("duration=5.0 被采纳", slow.segment(0).duration(), 5.0, 0.01);

        PathTimeline tooFast = build(
                wp(0, 0, 0, 0, 0, 0),
                wp(24, 0, 0, 0, 0, 0.1));
        double expected = 2.0 * Math.sqrt(24.0 / 50.0);
        close("duration=0.1 不生效（不会加速）", tooFast.segment(0).duration(), expected, 0.05);
        check("duration=0.1 段时长仍接近物理最短（离散积分略小于解析解）",
                tooFast.segment(0).duration() > expected - 0.05);
    }

    /** 段首时间 = 前段累计；总时间 = 各段之和 */
    private static void testSegmentBoundariesAndStartTimes() {
        defaults();
        PathTimeline tl = build(
                wp(0, 0, 0, 0, 0, 0),
                wp(12, 0, 0, 0, 0, 0),
                wp(12, 12, 0, 0, 90, 0));

        check("段数=2", tl.segmentCount() == 2);
        close("段0 起点时间", tl.segment(0).startTime(), 0.0, 1e-9);
        close("段1 起点时间 = 段0 时长", tl.segment(1).startTime(), tl.segment(0).duration(), 1e-9);
        close("总时间 = 段时长之和",
                tl.totalMotionTime(), tl.segment(0).duration() + tl.segment(1).duration(), 1e-9);

        boolean monotone = true;
        for (int i = 1; i < tl.sampleCount(); i++) {
            if (tl.sampleTime(i) < tl.sampleTime(i - 1) - 1e-12) monotone = false;
        }
        check("时间表单调不减", monotone);
        System.out.println("  [2 segs] " + tl.describe());
    }

    /** 段内 u → 时间 → u 往返一致 */
    private static void testUTimeRoundTrip() {
        defaults();
        PathTimeline tl = build(
                wp(0, 0, 0, 0, 0, 0),
                wp(18, 6, 0, 0, 30, 0),
                wp(30, 24, 0, 0, 120, 0));

        double worst = 0;
        for (int i = 0; i < tl.segmentCount(); i++) {
            PathTimeline.Segment seg = tl.segment(i);
            close("seg" + i + " timeAtU(0)=0", seg.timeAtU(0), 0.0, 1e-9);
            close("seg" + i + " timeAtU(1)=duration", seg.timeAtU(1), seg.duration(), 1e-6);
            close("seg" + i + " uAtTime(0)=0", seg.uAtTime(0), 0.0, 1e-9);
            close("seg" + i + " uAtTime(duration)=1", seg.uAtTime(seg.duration()), 1.0, 1e-9);
            for (int k = 1; k <= 19; k++) {
                double u = k / 20.0;
                double back = seg.uAtTime(seg.timeAtU(u));
                worst = Math.max(worst, Math.abs(back - u));
            }
        }
        check("u↔t 往返最大误差 < 0.01（实测 " + fmt(worst) + "）", worst < 0.01);
    }

    /** 前视：uAtTime(timeAtU(uStar) + dt) 永远不落后于 uStar */
    private static void testLookaheadNeverBehind() {
        defaults();
        PathTimeline tl = build(
                wp(0, 0, 0, 0, 0, 0),
                wp(20, 10, 0, 0, 45, 0),
                wp(40, -5, 0, 0, -30, 0));
        double dt = 0.015;
        boolean ok = true;
        for (int i = 0; i < tl.segmentCount(); i++) {
            PathTimeline.Segment seg = tl.segment(i);
            for (int k = 0; k <= 99; k++) {
                double u = k / 100.0;
                double target = seg.uAtTime(seg.timeAtU(u) + dt);
                if (target < u - 1e-9) ok = false;
            }
        }
        check("前视目标点不落后于实际进度", ok);
    }

    /** 牛顿法投影：把样条上的点投回去应得到同一个 u */
    private static void testNewtonProjection() {
        defaults();
        PathTimeline tl = build(
                wp(0, 0, 0, 0, 0, 0),
                wp(24, 18, 6, 12, 60, 0));

        PathTimeline.Segment seg = tl.segment(0);
        double worst = 0;
        for (int k = 1; k <= 19; k++) {
            double u = k / 20.0;
            double pu = PathTimeline.projectU(seg.splineX(), seg.splineY(),
                    seg.x(u), seg.y(u), 20, 3);
            worst = Math.max(worst, Math.abs(pu - u));
        }
        check("牛顿法投影误差 < 0.02（实测 " + fmt(worst) + "）", worst < 0.02);
    }

    /** 速度上限被切向量约束；双向扫描保证 |dv/dt| <= A_MAX */
    private static void testSpeedAndAccelLimits() {
        defaults();
        PathTimeline.TANGENT_SPEED_MODE = PathTimeline.TANGENT_ABSOLUTE;   // 本例验证"幅值即 in/s"
        PathTimeline tl = build(
                wp(0, 0, 0, 10, 0, 0),
                wp(20, 0, 10, 0, 0, 0),
                wp(20, 20, 0, 0, 90, 0));

        double maxV = 0;
        boolean accelOk = true;
        boolean speedOk = true;
        for (int i = 0; i < tl.sampleCount(); i++) {
            maxV = Math.max(maxV, tl.sampleSpeed(i));
            if (tl.sampleSpeed(i) > PathTimeline.V_MAX + 1e-6) speedOk = false;
            if (i > 0) {
                double dt = tl.sampleTime(i) - tl.sampleTime(i - 1);
                if (dt > 1e-9) {
                    double a = Math.abs(tl.sampleSpeed(i) - tl.sampleSpeed(i - 1)) / dt;
                    if (a > PathTimeline.A_MAX * (1 + 1e-6)) accelOk = false;
                }
            }
        }
        check("切向速度不超过 V_MAX", speedOk);
        check("切向加速度不超过 A_MAX", accelOk);
        check("切向量上限 10in/s 生效（峰值 " + fmt(maxV) + "）", maxV <= 10.0 + 1e-6);
        check("起点静止", tl.sampleSpeed(0) == 0.0);
        check("末点静止（末点切向量为 0）", tl.sampleSpeed(tl.sampleCount() - 1) == 0.0);
    }

    /** 忽略切向量后同样一条路径允许跑得更快 */
    private static void testTangentOffMode() {
        defaults();
        PathTimeline.TANGENT_SPEED_MODE = PathTimeline.TANGENT_OFF;
        PathTimeline tl = build(
                wp(0, 0, 0, 10, 0, 0),
                wp(20, 0, 10, 0, 0, 0),
                wp(20, 20, 0, 0, 90, 0));
        double maxV = 0;
        for (int i = 0; i < tl.sampleCount(); i++) maxV = Math.max(maxV, tl.sampleSpeed(i));
        check("TANGENT_OFF 时峰值速度 > 10in/s（实测 " + fmt(maxV) + "）", maxV > 10.0);
    }

    /**
     * RELATIVE 与 ABSOLUTE 在"小手柄"数据上的差异（本仓库旧 JSON 的典型情况）：
     * 两端幅值都很小时，RELATIVE 仍按形状跑到 V_MAX，ABSOLUTE 会把整段压到速度下限。
     */
    private static void testRelativeVsAbsoluteOnSmallTangents() {
        defaults();
        PathTimeline.TANGENT_SPEED_MODE = PathTimeline.TANGENT_RELATIVE;
        PathTimeline rel = build(
                wp(0, 0, 0, 1, 0, 0),
                wp(20, 0, 1, 0, 0, 0),
                wp(20, 20, 0, 0, 90, 0));
        double relPeak = 0;
        for (int i = 0; i < rel.sampleCount(); i++) relPeak = Math.max(relPeak, rel.sampleSpeed(i));

        PathTimeline.TANGENT_SPEED_MODE = PathTimeline.TANGENT_ABSOLUTE;
        PathTimeline abs = build(
                wp(0, 0, 0, 1, 0, 0),
                wp(20, 0, 1, 0, 0, 0),
                wp(20, 20, 0, 0, 90, 0));
        double absPeak = 0;
        for (int i = 0; i < abs.sampleCount(); i++) absPeak = Math.max(absPeak, abs.sampleSpeed(i));

        check("RELATIVE 下小幅值切向量仍能跑到 V_MAX（峰值 " + fmt(relPeak) + "）", relPeak > 30.0);
        check("ABSOLUTE 下同样数据被压到速度下限（峰值 " + fmt(absPeak) + "）", absPeak <= 2.0 + 1e-6);
        check("RELATIVE 总时间远小于 ABSOLUTE（" + fmt(rel.totalMotionTime())
                + "s vs " + fmt(abs.totalMotionTime()) + "s）",
                rel.totalMotionTime() * 2 < abs.totalMotionTime());
    }

    /**
     * 零弧长"原地转向"段（两个关键帧重合）：时间表必须是有限的 0 时长，
     * 且 uAtTime(前视) 直接给 1.0 —— 否则运行期 u* 恒为 0，出口判据永不成立（挂死）。
     */
    private static void testDegenerateSegment() {
        defaults();
        PathTimeline tl = build(
                wp(24, 24, 0, 0, 0, 0),
                wp(24, 24, 0, 0, 90, 0));

        check("退化段数=1", tl.segmentCount() == 1);
        PathTimeline.Segment seg = tl.segment(0);
        check("退化段弧长≈0", seg.arcLength() < 1e-9);
        check("退化段时长=0", Math.abs(seg.duration()) < 1e-9);
        check("退化段总时间有限", Double.isFinite(tl.totalMotionTime()));
        check("退化段 timeAtU 不产生 NaN", !Double.isNaN(seg.timeAtU(0.5)));
        check("退化段 uAtTime(前视) 直接给 1.0（运行期出口判据依赖它）", seg.uAtTime(0.015) == 1.0);
        check("退化段 heading 仍可求值", Double.isFinite(seg.heading(1.0)));

        boolean speedOk = true;
        for (int i = 0; i < tl.sampleCount(); i++) {
            if (Double.isNaN(tl.sampleSpeed(i)) || Double.isInfinite(tl.sampleSpeed(i))) speedOk = false;
        }
        check("退化段速度表无 NaN/Inf", speedOk);

        // 近退化但非零（0.15in）：弧长 > 0.05in，走正常 u* 判据
        PathTimeline near = build(
                wp(24, 24, 0, 0, 0, 0),
                wp(24, 24.15, 0, 0, 0, 0));
        check("近退化段弧长 > 0.05in（不会走退化分支）", near.segment(0).arcLength() > 0.05);
        check("近退化段时间有限", Double.isFinite(near.totalMotionTime()));
    }

    // ------------------------------------------------------------------
    // 工具
    // ------------------------------------------------------------------

    private static void defaults() {
        PathTimeline.V_MAX = 50;
        PathTimeline.A_MAX = 50;
        PathTimeline.CAP_FLOOR = 2;
        PathTimeline.A_LAT = 0;
        PathTimeline.TARGET_PATH_TIME_S = 0;
        PathTimeline.TANGENT_SPEED_MODE = PathTimeline.TANGENT_ABSOLUTE;   // 与生产默认值一致
        PathTimeline.MIN_INTERVALS = 64;
        PathTimeline.MAX_INTERVALS = 400;
        PathTimeline.INTERVALS_PER_INCH = 3.0;
    }

    private static PathTimeline.Waypoint wp(double x, double y, double dx, double dy,
                                            double heading, double duration) {
        return new PathTimeline.Waypoint(x, y, dx, dy, heading, duration);
    }

    private static PathTimeline build(PathTimeline.Waypoint... wps) {
        List<PathTimeline.Waypoint> list = new ArrayList<>();
        for (PathTimeline.Waypoint w : wps) list.add(w);
        return new PathTimeline(list, 0.0);
    }

    private static void check(String name, boolean ok) {
        checks++;
        if (!ok) {
            failures++;
            System.out.println("  FAIL: " + name);
        } else {
            System.out.println("  ok  : " + name);
        }
    }

    private static void close(String name, double actual, double expected, double tol) {
        check(name + " (got " + fmt(actual) + ", want " + fmt(expected) + "±" + fmt(tol) + ")",
                Math.abs(actual - expected) <= tol);
    }

    private static String fmt(double v) {
        return String.format(Locale.US, "%.4f", v);
    }
}
