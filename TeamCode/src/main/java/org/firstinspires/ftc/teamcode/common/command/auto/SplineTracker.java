package org.firstinspires.ftc.teamcode.common.command.auto;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;

/**
 * Spatial spline follower.
 *
 * <p>核心思想：路径进度完全由机器人在 Hermite spline 上的空间投影决定，
 * 不再由 wall-clock / timeline / look-ahead target point 推动。</p>
 *
 * <p>平移控制拆成两个互相独立的闭环：</p>
 * <ul>
 *     <li>切向：速度 PID（默认 D=0，即 PI），负责“沿路径跑多快”</li>
 *     <li>法向：cross-track PID，负责“回到路径上”</li>
 * </ul>
 *
 * <p>最终场地坐标系平移命令：</p>
 * <pre>
 *     P = tangentPower * T_hat + crossPower * N_hat
 * </pre>
 *
 * <p>其中 dx/dy 仍然只是 Hermite 几何切线参数，不代表物理速度。</p>
 */
public class SplineTracker implements SplineTrajectoryLoader.MotionDriver {
    /**
     * Immutable geometric waypoint used by the preferred API.
     * dx/dy are Hermite tangent parameters, not physical velocity.
     */
    public static final class PathPoint {
        public final double x;
        public final double y;
        public final double dx;
        public final double dy;
        public final double heading;

        public PathPoint(double x, double y, double dx, double dy, double heading) {
            this.x = x;
            this.y = y;
            this.dx = dx;
            this.dy = dy;
            this.heading = heading;
        }

        @Override
        public String toString() {
            return String.format(
                    "PathPoint{x=%.2f,y=%.2f,dx=%.2f,dy=%.2f,h=%.1f}",
                    x, y, dx, dy, heading);
        }
    }

    public static boolean LOG_VERBOSE = false;

    // ---------------------------------------------------------------------
    // Public path state (kept compatible with the previous class)
    // ---------------------------------------------------------------------
    public double[] startPoint;
    public double lx, ly, ldx, ldy;
    public double x, y, dx, dy;
    public double heading, preH;

    // ---------------------------------------------------------------------
    // Main tuning knobs
    // ---------------------------------------------------------------------

    /** Normal cruise target speed, inch/s. */
    public static double MAX_TANGENTIAL_VEL = 40.0;

    /**
     * Non-zero target speed at the very beginning of a segment.
     * This avoids the old zero-speed / zero-error start deadlock.
     */
    public static double START_TARGET_VEL = 16.0;

    /** Distance over which START_TARGET_VEL ramps to MAX_TANGENTIAL_VEL. */
    public static double START_RAMP_DISTANCE = 6.0;

    /** Distance before the end over which target speed is reduced. */
    public static double BRAKE_DISTANCE = 16.0;

    /**
     * Near-zero-speed launch assist. This is intentionally NOT a calibrated kS;
     * it is only a coarse floor used while the robot is commanded to move but is
     * still almost stationary. Set to 0 to disable.
     */
    public static double START_MIN_POWER = 0.10;
    public static double START_ASSIST_TARGET_VEL = 3.0;
    public static double START_ASSIST_ACTUAL_VEL = 1.0;

    // Tangential velocity PID. Default D=0 -> PI, usually easier to tune on FTC odometry.
    // Units: Kp ~ power/(in/s), Ki ~ power/in, Kd ~ power/(in/s^2).
    public static double K_VEL_P = 0.018;
    public static double K_VEL_I = 0.008;
    public static double K_VEL_D = 0.0;
    public static double VEL_I_MAX_POWER = 0.35;
    public static double VEL_D_ALPHA = 0.25; // EMA new-sample weight

    // Cross-track PID. Integral defaults to zero; PD is normally sufficient.
    public static double K_CROSS_P = 0.10;
    public static double K_CROSS_I = 0.0;
    public static double K_CROSS_D = 0.010;
    public static double CROSS_I_MAX_POWER = 0.20;
    public static double CROSS_D_ALPHA = 0.25;

    // Heading controller kept intentionally simple.
    public static double K_HEADING = 0.02;
    public static double TURN_DEAD_AREA = 0.0;

    /** Low-pass filter for odometry-derived velocity. 1 = raw, smaller = smoother. */
    public static double VELOCITY_FILTER_ALPHA = 0.35;

    /** Translation command vector magnitude limit before sending to drivetrain. */
    public static double MAX_TRANSLATION_POWER = 1.0;

    // ---------------------------------------------------------------------
    // Projection / geometry tuning
    // ---------------------------------------------------------------------
    public static int ARC_SAMPLES = 240;
    public static int PROJECTION_SAMPLES = 14;
    public static int NEWTON_ITERS = 4;

    /** Search a little behind the previous projection, in path inches. */
    public static double PROJECTION_BACKTRACK_IN = 3.0;

    /** Search ahead of the previous projection, in path inches. */
    public static double PROJECTION_FORWARD_IN = 18.0;

    /** Used when spline derivative is ~0 at a Hermite endpoint. */
    public static double TANGENT_SAMPLE_DU = 0.01;

    // ---------------------------------------------------------------------
    // Completion / safety
    // ---------------------------------------------------------------------
    public static double END_U_THRESHOLD = 0.94;
    public static double END_POS_THRESHOLD = 1.0;
    public static double END_HEADING_THRESHOLD = 0.8;
    public static double END_SPEED_THRESHOLD = 3.0;
    public static double END_CAPTURE_DISTANCE = 8.0;
    public static double DEGENERATE_ARC_THRESHOLD = 0.05;

    /** <= 0 disables stall detection. A stall is NOT treated as arrival. */
    public static double STALL_TIMEOUT_S = 2.0;
    public static double STALL_PROGRESS_EPS_IN = 0.08;
    public static double STALL_END_ERROR_EPS_IN = 0.05;

    // ---------------------------------------------------------------------
    // Optional voltage compensation (closed-loop control generally needs little)
    // ---------------------------------------------------------------------
    public static double motorVoltage = 12.0;
    public static double VOLTAGE_COMP_WEIGHT = 0.0;

    public enum SegmentStatus {
        IDLE,
        RUNNING,
        ARRIVED,
        STALLED,
        ABORTED
    }

    /** Result returned by the preferred segment-oriented API. */
    public static final class SegmentResult {
        public final SegmentStatus status;
        public final double finalPositionError;
        public final double finalHeadingError;

        private SegmentResult(SegmentStatus status,
                              double finalPositionError,
                              double finalHeadingError) {
            this.status = status;
            this.finalPositionError = finalPositionError;
            this.finalHeadingError = finalHeadingError;
        }

        public boolean arrived() {
            return status == SegmentStatus.ARRIVED;
        }

        @Override
        public String toString() {
            return String.format(
                    "SegmentResult{%s,posErr=%.2f,headingErr=%.2f}",
                    status, finalPositionError, finalHeadingError);
        }
    }

    private final Robot robot;

    private double gotoX, gotoY, gotoH;
    private SegmentStatus lastSegmentStatus = SegmentStatus.IDLE;
    private SegmentResult lastResult =
            new SegmentResult(SegmentStatus.IDLE, Double.NaN, Double.NaN);
    private int segmentIndex = -1;

    private final PIDState velPID = new PIDState();
    private final PIDState crossPID = new PIDState();

    private final ElapsedTime stallTimer = new ElapsedTime();

    // ---------------------------------------------------------------------
    // Non-blocking active-segment state
    // ---------------------------------------------------------------------

    private double[] activeSplineX;
    private double[] activeSplineY;
    private double[] activeSplineH;
    private ArcTable activeArc;
    private double activeUnwrappedHeading;
    private boolean activeHeadingOnly;

    private double prevRx;
    private double prevRy;
    private double prevLoopTime;
    private double filteredVx;
    private double filteredVy;

    private double closestU;
    private double progressS;
    private double stallLastProgressS;
    private double bestEndError = Double.POSITIVE_INFINITY;

    private static class PIDState {
        double integral;
        double lastError;
        double filteredDerivative;
        boolean firstRun = true;

        void reset() {
            integral = 0;
            lastError = 0;
            filteredDerivative = 0;
            firstRun = true;
        }
    }

    /** Uniform-u sampled arc-length lookup table. */
    private static class ArcTable {
        final double[] cumulative;
        final int n;
        final double total;

        ArcTable(double[] cumulative) {
            this.cumulative = cumulative;
            this.n = cumulative.length - 1;
            this.total = cumulative[cumulative.length - 1];
        }

        double sAtU(double u) {
            u = clip01(u);
            double p = u * n;
            int i = (int) Math.floor(p);
            if (i >= n) return total;
            double f = p - i;
            return cumulative[i] + f * (cumulative[i + 1] - cumulative[i]);
        }

        double uAtS(double s) {
            if (total <= 1e-9) return 0;
            if (s <= 0) return 0;
            if (s >= total) return 1;

            int lo = 0;
            int hi = n;
            while (lo < hi - 1) {
                int mid = (lo + hi) >>> 1;
                if (cumulative[mid] <= s) lo = mid;
                else hi = mid;
            }

            double span = cumulative[hi] - cumulative[lo];
            double f = span > 1e-9 ? (s - cumulative[lo]) / span : 0;
            return (lo + f) / n;
        }
    }

    // ---------------------------------------------------------------------
    // Spline math
    // ---------------------------------------------------------------------
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

    private double splineSecondDerivative(double[] spline, double u) {
        return 6 * spline[0] * u + 2 * spline[1];
    }

    private ArcTable buildArcTable(double[] sx, double[] sy) {
        int n = Math.max(40, ARC_SAMPLES);
        double[] cum = new double[n + 1];
        double px = spline_get(sx, 0);
        double py = spline_get(sy, 0);

        for (int i = 1; i <= n; i++) {
            double u = (double) i / n;
            double cx = spline_get(sx, u);
            double cy = spline_get(sy, u);
            cum[i] = cum[i - 1] + Math.hypot(cx - px, cy - py);
            px = cx;
            py = cy;
        }
        return new ArcTable(cum);
    }

    /**
     * Local coarse-search + Newton projection.
     * The local arc-length window prevents a U-shaped/self-near segment from jumping
     * to a far-away branch of the same cubic merely because it is spatially close.
     */
    private double findClosestU(double[] sx, double[] sy, ArcTable arc,
                                double rx, double ry, double previousU) {
        double previousS = arc.sAtU(previousU);
        double lowU = arc.uAtS(previousS - Math.max(0, PROJECTION_BACKTRACK_IN));
        double highU = arc.uAtS(previousS + Math.max(0, PROJECTION_FORWARD_IN));

        if (highU < lowU + 1e-6) {
            lowU = Math.max(0, previousU - 0.05);
            highU = Math.min(1, previousU + 0.10);
        }

        // Coarse local seed.
        int samples = Math.max(4, PROJECTION_SAMPLES);
        double bestU = lowU;
        double bestD2 = Double.POSITIVE_INFINITY;
        for (int i = 0; i <= samples; i++) {
            double u = lowU + (highU - lowU) * i / samples;
            double ex = spline_get(sx, u) - rx;
            double ey = spline_get(sy, u) - ry;
            double d2 = ex * ex + ey * ey;
            if (d2 < bestD2) {
                bestD2 = d2;
                bestU = u;
            }
        }

        // Newton on D'(u) = (r(u)-p) dot r'(u) = 0.
        double u = bestU;
        for (int i = 0; i < Math.max(1, NEWTON_ITERS); i++) {
            double px = spline_get(sx, u);
            double py = spline_get(sy, u);
            double dxdu = splineDerivative(sx, u);
            double dydu = splineDerivative(sy, u);
            double ddxdu = splineSecondDerivative(sx, u);
            double ddydu = splineSecondDerivative(sy, u);

            double ex = px - rx;
            double ey = py - ry;

            double f = ex * dxdu + ey * dydu;
            double df = dxdu * dxdu + dydu * dydu + ex * ddxdu + ey * ddydu;
            if (Math.abs(df) < 1e-9) break;

            u -= f / df;
            u = Range.clip(u, lowU, highU);
        }

        // Newton can converge to a local maximum in pathological cases. Compare with bounds.
        double u0 = u;
        double d0 = distanceSqToSpline(sx, sy, rx, ry, u0);
        double dl = distanceSqToSpline(sx, sy, rx, ry, lowU);
        double dh = distanceSqToSpline(sx, sy, rx, ry, highU);
        if (dl < d0 && dl <= dh) return lowU;
        if (dh < d0) return highU;
        return u0;
    }

    private double distanceSqToSpline(double[] sx, double[] sy,
                                      double rx, double ry, double u) {
        double ex = spline_get(sx, u) - rx;
        double ey = spline_get(sy, u) - ry;
        return ex * ex + ey * ey;
    }

    /** Unit tangent in increasing-u direction, robust to zero endpoint derivatives. */
    private double[] unitTangent(double[] sx, double[] sy, double u) {
        double tx = splineDerivative(sx, u);
        double ty = splineDerivative(sy, u);
        double mag = Math.hypot(tx, ty);

        if (mag < 1e-6) {
            double du = Math.max(1e-4, TANGENT_SAMPLE_DU);
            double u0 = Math.max(0, u - du);
            double u1 = Math.min(1, u + du);
            if (u1 - u0 < 1e-6) {
                u0 = Math.max(0, u - 2 * du);
                u1 = Math.min(1, u + 2 * du);
            }
            tx = spline_get(sx, u1) - spline_get(sx, u0);
            ty = spline_get(sy, u1) - spline_get(sy, u0);
            mag = Math.hypot(tx, ty);
        }

        if (mag < 1e-6) {
            tx = spline_get(sx, 1) - spline_get(sx, 0);
            ty = spline_get(sy, 1) - spline_get(sy, 0);
            mag = Math.hypot(tx, ty);
        }

        if (mag < 1e-6) return new double[]{1, 0};
        return new double[]{tx / mag, ty / mag};
    }

    private static double clip01(double v) {
        return Math.max(0, Math.min(1, v));
    }

    private static double smoothstep01(double x) {
        x = clip01(x);
        return x * x * (3 - 2 * x);
    }

    private double angleDiff(double target, double current) {
        double diff = target - current;
        while (diff > 180) diff -= 360;
        while (diff <= -180) diff += 360;
        return diff;
    }

    /** Generic PID with integral contribution clamp and EMA-filtered derivative. */
    private double computePID(double error, double dt,
                              double kp, double ki, double kd,
                              double iMaxPower, double dAlpha,
                              PIDState state) {
        if (dt <= 1e-5 || dt > 0.25) dt = 0.02;

        if (state.firstRun) {
            state.firstRun = false;
            state.lastError = error;
            state.filteredDerivative = 0;
        }

        if (ki != 0) {
            state.integral += error * dt;
            double iTerm = ki * state.integral;
            if (iMaxPower > 0) {
                double clipped = Range.clip(iTerm, -iMaxPower, iMaxPower);
                if (clipped != iTerm) state.integral = clipped / ki;
            }
        } else {
            state.integral = 0;
        }

        double rawDerivative = (error - state.lastError) / dt;
        double alpha = clip01(dAlpha);
        state.filteredDerivative = alpha * rawDerivative
                + (1 - alpha) * state.filteredDerivative;
        state.lastError = error;

        double iTerm = ki * state.integral;
        if (iMaxPower > 0) iTerm = Range.clip(iTerm, -iMaxPower, iMaxPower);

        return kp * error + iTerm + kd * state.filteredDerivative;
    }

    // ---------------------------------------------------------------------
    // Initialization
    // ---------------------------------------------------------------------
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

    // ---------------------------------------------------------------------
    // Tick-based MotionDriver API
    // ---------------------------------------------------------------------

    @Override
    public void begin(Pose2D pose, PathPoint start) {
        if (pose == null) throw new IllegalArgumentException("pose == null");
        if (start == null) throw new IllegalArgumentException("start == null");

        setPose(pose);
        this.startPoint = new double[]{start.x, start.y, start.dx, start.dy};
        this.lx = start.x;
        this.ly = start.y;
        this.ldx = start.dx;
        this.ldy = start.dy;
        this.x = start.x;
        this.y = start.y;
        this.dx = start.dx;
        this.dy = start.dy;
        this.heading = start.heading;
        this.preH = start.heading;
        this.segmentIndex = -1;
        this.lastSegmentStatus = SegmentStatus.IDLE;
        this.lastResult = new SegmentResult(SegmentStatus.IDLE, 0, 0);
        clearActiveSegment();
    }

    /**
     * Prepare one Hermite segment. No motor output is produced here.
     */
    @Override
    public void startMove(PathPoint target) {
        if (target == null) throw new IllegalArgumentException("target == null");

        if (lastSegmentStatus == SegmentStatus.RUNNING) cancel();

        this.lx = this.x;
        this.ly = this.y;
        this.ldx = this.dx;
        this.ldy = this.dy;
        this.x = target.x;
        this.y = target.y;
        this.dx = target.dx;
        this.dy = target.dy;
        this.segmentIndex++;

        activeSplineX = spline_fit(this.lx, this.ldx, this.x, this.dx);
        activeSplineY = spline_fit(this.ly, this.ldy, this.y, this.dy);

        activeUnwrappedHeading = target.heading;
        double hDiff = target.heading - this.heading;
        while (hDiff > 180) {
            activeUnwrappedHeading -= 360;
            hDiff -= 360;
        }
        while (hDiff <= -180) {
            activeUnwrappedHeading += 360;
            hDiff += 360;
        }
        activeSplineH = spline_fit(this.heading, 0, activeUnwrappedHeading, 0);
        activeArc = buildArcTable(activeSplineX, activeSplineY);

        velPID.reset();
        crossPID.reset();
        stallTimer.reset();
        lastSegmentStatus = SegmentStatus.RUNNING;
        activeHeadingOnly = activeArc.total < DEGENERATE_ARC_THRESHOLD;

        robot.odo.update();
        prevRx = getX();
        prevRy = getY();
        prevLoopTime = System.nanoTime() / 1e9;
        filteredVx = 0;
        filteredVy = 0;
        closestU = 0;
        progressS = 0;
        stallLastProgressS = 0;
        bestEndError = Double.POSITIVE_INFINITY;
        lastResult = new SegmentResult(SegmentStatus.RUNNING, Double.NaN, Double.NaN);
    }

    /**
     * Execute exactly one control iteration.
     */
    @Override
    public SegmentResult update() {
        if (lastSegmentStatus != SegmentStatus.RUNNING) return lastResult;

        if (!robot.opMode.opModeIsActive()) {
            lastSegmentStatus = SegmentStatus.ABORTED;
            lastResult = snapshotResult(lastSegmentStatus);
            clearActiveSegment();
            return lastResult;
        }

        if (activeHeadingOnly) {
            return updateHeadingOnly();
        }

        robot.odo.update();

        double now = System.nanoTime() / 1e9;
        double dt = now - prevLoopTime;
        if (dt <= 1e-5 || dt > 0.25) dt = 0.02;
        prevLoopTime = now;

        double rx = getX();
        double ry = getY();
        double curH = getHeading();

        double rawVx = (rx - prevRx) / dt;
        double rawVy = (ry - prevRy) / dt;
        prevRx = rx;
        prevRy = ry;

        double velAlpha = clip01(VELOCITY_FILTER_ALPHA);
        filteredVx = velAlpha * rawVx + (1 - velAlpha) * filteredVx;
        filteredVy = velAlpha * rawVy + (1 - velAlpha) * filteredVy;

        closestU = findClosestU(activeSplineX, activeSplineY, activeArc, rx, ry, closestU);
        double closestS = activeArc.sAtU(closestU);
        if (closestS > progressS) progressS = closestS;
        double progressU = activeArc.uAtS(progressS);

        double pathX = spline_get(activeSplineX, closestU);
        double pathY = spline_get(activeSplineY, closestU);
        double[] tangent = unitTangent(activeSplineX, activeSplineY, closestU);
        double tx = tangent[0];
        double ty = tangent[1];
        double nx = -ty;
        double ny = tx;

        double toPathX = pathX - rx;
        double toPathY = pathY - ry;
        double crossError = toPathX * nx + toPathY * ny;
        double crossPower = computePID(
                crossError, dt,
                K_CROSS_P, K_CROSS_I, K_CROSS_D,
                CROSS_I_MAX_POWER, CROSS_D_ALPHA,
                crossPID);

        double tangentialVel = filteredVx * tx + filteredVy * ty;
        double remainingArc = Math.max(0, activeArc.total - progressS);
        double targetVel = targetTangentialVelocity(
                progressS,
                remainingArc,
                rx,
                ry,
                activeSplineX,
                activeSplineY,
                activeArc);

        double tangentPower = computePID(
                targetVel - tangentialVel, dt,
                K_VEL_P, K_VEL_I, K_VEL_D,
                VEL_I_MAX_POWER, VEL_D_ALPHA,
                velPID);

        if (START_MIN_POWER > 0
                && Math.abs(targetVel) >= START_ASSIST_TARGET_VEL
                && Math.abs(tangentialVel) <= START_ASSIST_ACTUAL_VEL
                && Math.abs(tangentPower) < START_MIN_POWER) {
            tangentPower = Math.copySign(START_MIN_POWER, targetVel);
        }

        tangentPower = Range.clip(tangentPower, -1, 1);
        crossPower = Range.clip(crossPower, -1, 1);

        double targetH = spline_get(activeSplineH, progressU);
        double headingError = angleDiff(targetH, curH);
        double yawPower = headingError * K_HEADING;
        if (Math.abs(headingError) < TURN_DEAD_AREA) yawPower = 0;
        yawPower = Range.clip(yawPower, -1, 1);

        double powerX = tangentPower * tx + crossPower * nx;
        double powerY = tangentPower * ty + crossPower * ny;

        double voltage = robot.getVoltage();
        double motorPowerGain = 1.0;
        if (voltage > 1e-6) {
            double rawGain = Math.abs(motorVoltage / voltage);
            motorPowerGain = 1.0 + (rawGain - 1.0) * VOLTAGE_COMP_WEIGHT;
        }
        powerX *= motorPowerGain;
        powerY *= motorPowerGain;
        yawPower *= motorPowerGain;

        double translationMag = Math.hypot(powerX, powerY);
        double maxTranslation = Range.clip(MAX_TRANSLATION_POWER, 0, 1);
        if (translationMag > maxTranslation && translationMag > 1e-9) {
            double scale = maxTranslation / translationMag;
            powerX *= scale;
            powerY *= scale;
        }
        yawPower = Range.clip(yawPower, -1, 1);

        if (!Globals.DEBUG) {
            robot.odoDrivetrain.driveRobotFieldCentric(powerX, -powerY, -yawPower);
        }

        gotoX = pathX;
        gotoY = pathY;
        gotoH = targetH;

        double endPosError = Math.hypot(this.x - rx, this.y - ry);
        double speedMag = Math.hypot(filteredVx, filteredVy);
        boolean done = progressU >= END_U_THRESHOLD
                && endPosError <= END_POS_THRESHOLD
                && Math.abs(headingError) <= END_HEADING_THRESHOLD
                && speedMag <= END_SPEED_THRESHOLD;

        if (STALL_TIMEOUT_S > 0) {
            boolean progressed = progressS >= stallLastProgressS + STALL_PROGRESS_EPS_IN;
            boolean endImproved = endPosError <= bestEndError - STALL_END_ERROR_EPS_IN;
            if (progressed || endImproved) {
                if (progressed) stallLastProgressS = progressS;
                if (endPosError < bestEndError) bestEndError = endPosError;
                stallTimer.reset();
            } else if (stallTimer.seconds() >= STALL_TIMEOUT_S && !done) {
                return finishActiveSegment(SegmentStatus.STALLED);
            }
        }

        if (done) {
            return finishActiveSegment(SegmentStatus.ARRIVED);
        }

        robot.telemetry.addData("seg", segmentIndex);
        robot.telemetry.addData("uClosest", String.format("%.4f", closestU));
        robot.telemetry.addData("progress", String.format("%.1f / %.1f in", progressS, activeArc.total));
        robot.telemetry.addData("crossErr", String.format("%.2f in", crossError));
        robot.telemetry.addData("vTarget", String.format("%.1f in/s", targetVel));
        robot.telemetry.addData("vTangential", String.format("%.1f in/s", tangentialVel));
        robot.telemetry.addData("pTangent", String.format("%.3f", tangentPower));
        robot.telemetry.addData("pCross", String.format("%.3f", crossPower));
        robot.telemetry.addData("targetH", String.format("%.1f", targetH));
        robot.telemetry.addData("headingErr", String.format("%.1f", headingError));
        robot.telemetry.addData("endErr", String.format("%.2f in", endPosError));
        robot.telemetry.addData("cmdXY", String.format("%.3f, %.3f", powerX, powerY));
        robot.telemetry.addData("Voltage", voltage);

        if (LOG_VERBOSE) {
            Log.i("SplineTracker", "seg=" + segmentIndex
                    + " u=" + String.format("%.4f", closestU)
                    + " s=" + String.format("%.2f", progressS)
                    + " v=" + String.format("%.1f/%.1f", tangentialVel, targetVel)
                    + " cross=" + String.format("%.2f", crossError)
                    + " pT=" + String.format("%.3f", tangentPower)
                    + " pN=" + String.format("%.3f", crossPower));
        }

        lastResult = new SegmentResult(
                SegmentStatus.RUNNING,
                endPosError,
                headingError);
        return lastResult;
    }

    private SegmentResult updateHeadingOnly() {
        robot.odo.update();
        double current = getHeading();
        double error = angleDiff(activeUnwrappedHeading, current);
        double absError = Math.abs(error);

        gotoX = getX();
        gotoY = getY();
        gotoH = activeUnwrappedHeading;

        if (absError <= END_HEADING_THRESHOLD) {
            return finishActiveSegment(SegmentStatus.ARRIVED);
        }

        if (absError <= bestEndError - 0.5) {
            bestEndError = absError;
            stallTimer.reset();
        } else if (STALL_TIMEOUT_S > 0 && stallTimer.seconds() >= STALL_TIMEOUT_S) {
            return finishActiveSegment(SegmentStatus.STALLED);
        }

        double yawPower = error * K_HEADING;
        if (Math.abs(error) < TURN_DEAD_AREA) yawPower = 0;
        yawPower = Range.clip(yawPower, -1, 1);

        if (!Globals.DEBUG) {
            robot.odoDrivetrain.driveRobotFieldCentric(0, 0, -yawPower);
        }

        lastResult = new SegmentResult(SegmentStatus.RUNNING, 0, error);
        return lastResult;
    }

    private SegmentResult finishActiveSegment(SegmentStatus status) {
        lastSegmentStatus = status;
        this.heading = activeUnwrappedHeading;
        this.preH = activeUnwrappedHeading;
        stopMotor();
        lastResult = snapshotResult(status);
        clearActiveSegment();
        return lastResult;
    }

    private SegmentResult snapshotResult(SegmentStatus status) {
        double posError = Math.hypot(this.x - getX(), this.y - getY());
        double headingError = angleDiff(this.heading, getHeading());
        return new SegmentResult(status, posError, headingError);
    }

    private void clearActiveSegment() {
        activeSplineX = null;
        activeSplineY = null;
        activeSplineH = null;
        activeArc = null;
        activeHeadingOnly = false;
    }

    @Override
    public void cancel() {
        if (lastSegmentStatus == SegmentStatus.RUNNING) {
            lastSegmentStatus = SegmentStatus.ABORTED;
            stopMotor();
            lastResult = snapshotResult(SegmentStatus.ABORTED);
        }
        clearActiveSegment();
    }

    @Override
    public SegmentResult getLastResult() {
        return lastResult;
    }

    public SplineTracker stopMotor() {
        robot.odoDrivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.odoDrivetrain.stopMotor();
        return this;
    }

    /**
     * Spatial speed command.
     *
     * <p>Start ramp depends on traveled arc length. Braking depends on remaining
     * arc length. Near the endpoint, signed along-track endpoint error is used so
     * an overshoot can command a small reverse velocity and settle back.</p>
     */
    private double targetTangentialVelocity(double progressS, double remainingArc,
                                            double rx, double ry,
                                            double[] sx, double[] sy, ArcTable arc) {
        double cruise = Math.max(0, MAX_TANGENTIAL_VEL);
        if (cruise <= 1e-6) return 0;

        // Start: non-zero initial target -> smooth ramp to cruise.
        double startBlend = START_RAMP_DISTANCE > 1e-6
                ? smoothstep01(progressS / START_RAMP_DISTANCE)
                : 1.0;
        double startVel = Math.max(0, START_TARGET_VEL)
                + (cruise - Math.max(0, START_TARGET_VEL)) * startBlend;
        startVel = Range.clip(startVel, 0, cruise);

        // Normal braking cap based purely on remaining path distance.
        double brakeCap;
        if (BRAKE_DISTANCE > 1e-6) {
            // sqrt gives a constant-deceleration-like shape without needing a time schedule.
            brakeCap = cruise * Math.sqrt(clip01(remainingArc / BRAKE_DISTANCE));
        } else {
            brakeCap = cruise;
        }

        double target = Math.min(startVel, brakeCap);

        // Terminal capture: use SIGNED along-track endpoint error.
        // This fixes the "overshoot u=1 then no tangential force to come back" failure mode.
        if (remainingArc <= END_CAPTURE_DISTANCE) {
            double[] endTangent = unitTangent(sx, sy, 1.0);
            double endErrorX = this.x - rx;
            double endErrorY = this.y - ry;
            double alongError = endErrorX * endTangent[0] + endErrorY * endTangent[1];

            double captureMag;
            if (BRAKE_DISTANCE > 1e-6) {
                captureMag = cruise * Math.sqrt(clip01(Math.abs(alongError) / BRAKE_DISTANCE));
            } else {
                captureMag = Math.min(cruise, Math.abs(alongError) * cruise);
            }

            target = Math.copySign(Math.min(Math.max(0, startVel), captureMag), alongError);
        }

        return target;
    }

    // ---------------------------------------------------------------------
    // Accessors
    // ---------------------------------------------------------------------
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

    /** Current nearest geometric point on the spline. */
    public double[] getCurrentGotoPoint() {
        return new double[]{gotoX, gotoY};
    }

    public double[] getPreviousPoint() {
        return new double[]{this.lx, this.ly, this.ldx, this.ldy};
    }

    public SegmentStatus getLastSegmentStatus() {
        return lastSegmentStatus;
    }

    public Robot getRobot() {
        return robot;
    }
}
