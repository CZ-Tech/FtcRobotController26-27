import org.firstinspires.ftc.teamcode.common.command.PathTimeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * 用真实路线数据预览 PathTimeline 排出来的时间表（纯 Java 离线工具）。
 *
 * <p>用途：确认 {@code dx/dy} 幅值该按哪种模式解释
 * （TANGENT_ABSOLUTE / TANGENT_RELATIVE / TANGENT_OFF），以及整条路径排完要多久。
 * 数据取自 {@code opmode/auto/routes/B_Far6Ball.java} 的 autoStr（DECODE 赛季旧数据）。</p>
 *
 * <pre>
 * javac -d out Team19656/src/main/java/org/firstinspires/ftc/teamcode/common/command/PathTimeline.java Team19656/tools/PathSchedulePreview.java
 * java -cp out PathSchedulePreview
 * </pre>
 */
public class PathSchedulePreview {

    /** B_Far6Ball 的 8 个关键帧：{x, y, dx, dy, heading, duration} */
    private static final double[][] B_FAR_6_BALL = {
            {58.2500, -19.1473, 0.0000, 0.0000, 0.0, 0.0},   // start
            {55.0056, -15.9034, -0.9334, -0.5045, 23.0, 0.3},   // preloadShoot
            {54.9411, -62.8320, 4.1719, -46.4623, 79.198, 0.6},
            {40.7195, -66.1926, 0.2854, 0.1458, -9.832, 1.0},   // intaker2Start
            {61.9686, -65.3519, -0.2069, -0.1150, -18.669, 2.0}, // intaker2Stop
            {55.0056, -15.8091, 0.0000, 0.0000, 23.5, 0.7},   // shoot2
            {60.9111, -30.8636, 0.0000, 0.0000, 0.0, 1.0},
            {62.0838, -30.8488, 0.0000, 0.0000, 0.0, 1.0},
    };

    public static void main(String[] args) {
        PathTimeline.LOG_SCHEDULE = false;
        PathTimeline.V_MAX = 50;
        PathTimeline.A_MAX = 50;
        PathTimeline.CAP_FLOOR = 2;
        PathTimeline.A_LAT = 0;
        PathTimeline.TARGET_PATH_TIME_S = 0;

        double jsonDurationSum = 0;
        for (double[] k : B_FAR_6_BALL) jsonDurationSum += k[5];

        System.out.println("=== B_Far6Ball: 8 关键帧, JSON duration 之和 = "
                + fmt(jsonDurationSum) + "s (marker 里的射击/传送动作另算) ===");
        System.out.println();

        int[] modes = {PathTimeline.TANGENT_ABSOLUTE, PathTimeline.TANGENT_RELATIVE, PathTimeline.TANGENT_OFF};
        String[] names = {"TANGENT_ABSOLUTE (|dx,dy| 当 in/s)", "TANGENT_RELATIVE (段内最大切向量=50in/s)", "TANGENT_OFF (只受 V_MAX/A_MAX)"};

        for (int m = 0; m < modes.length; m++) {
            PathTimeline.TANGENT_SPEED_MODE = modes[m];
            PathTimeline tl = build(B_FAR_6_BALL);
            System.out.println("--- " + names[m] + " ---");
            System.out.println("总运动时间 = " + fmt(tl.totalMotionTime()) + "s, 总弧长 = " + fmt(tl.totalArcLength()) + "in");
            StringBuilder sb = new StringBuilder("  各段时长: ");
            for (int i = 0; i < tl.segmentCount(); i++) {
                sb.append(fmt(tl.segment(i).duration())).append("s ");
            }
            System.out.println(sb);
            System.out.println();
        }

        // 竞赛用法示例：只受 V_MAX/A_MAX 约束，并且把整条路径拉到 20s（只变慢）
        PathTimeline.TANGENT_SPEED_MODE = PathTimeline.TANGENT_OFF;
        PathTimeline.TARGET_PATH_TIME_S = 20.0;
        PathTimeline tl = build(B_FAR_6_BALL);
        System.out.println("--- TANGENT_OFF + TARGET_PATH_TIME_S=20s ---");
        System.out.println("总运动时间 = " + fmt(tl.totalMotionTime()) + "s");
        System.out.println("  " + tl.describe());
    }

    private static PathTimeline build(double[][] kf) {
        List<PathTimeline.Waypoint> wps = new ArrayList<>();
        for (double[] k : kf) {
            wps.add(new PathTimeline.Waypoint(k[0], k[1], k[2], k[3], k[4], k[5]));
        }
        return new PathTimeline(wps, 0.0);
    }

    private static String fmt(double v) {
        return String.format(Locale.US, "%.2f", v);
    }
}
