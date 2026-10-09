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
import org.firstinspires.ftc.teamcode.common.command.auto.spline.SplineController;
import org.firstinspires.ftc.teamcode.common.command.auto.spline.SplineGeometry;
import org.firstinspires.ftc.teamcode.common.command.auto.spline.VelocityEstimator;

/**
 * Spatial spline follower.
 *
 * <p>核心思想：路径进度完全由机器人在 Hermite spline 上的空间投影决定，
 * 不再由 wall-clock / timeline / look-ahead target point 推动。</p>
 *
 * <p>平移控制拆成两个互相独立的通道：</p>
 * <ul>
 *     <li>切向：默认直接功率，只有显式限速或末端制动时才调节功率</li>
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
        /** Segment ending at this point: cruise power in [0, 1]. */
        public final double maxPower;
        /** Optional segment speed ceiling in inch/s; NaN disables it. */
        public final double maxSpeed;
        /** Optional desired speed at this point in inch/s; NaN means no braking. */
        public final double endSpeed;
        /** Braking-zone length measured back from this point, in inches. */
        public final double brakeZoneIn;
        /** Positive-power cap inside that zone; NaN uses the tracker default. */
        public final double brakeForwardPower;

        public PathPoint(double x, double y, double dx, double dy, double heading) {
            this(x, y, dx, dy, heading, 1.0, Double.NaN, Double.NaN, 0.0, Double.NaN);
        }

        public PathPoint(double x, double y, double dx, double dy, double heading,
                         double maxPower, double maxSpeed, double endSpeed,
                         double brakeZoneIn, double brakeForwardPower) {
            this.x = x;
            this.y = y;
            this.dx = dx;
            this.dy = dy;
            this.heading = heading;
            this.maxPower = maxPower;
            this.maxSpeed = maxSpeed;
            this.endSpeed = endSpeed;
            this.brakeZoneIn = brakeZoneIn;
            this.brakeForwardPower = brakeForwardPower;
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

    /** Conservative *measured* effective deceleration in inch/s^2. Must be calibrated. */
    public static double SAFE_BRAKE_DECEL = 30.0;
    /** Negative tangential motor-power magnitude requested by the Bang-Bang brake. */
    public static double BRAKE_REVERSE_POWER = 0.75;
    /** Default positive power ceiling once inside a JSON braking zone. */
    public static double DEFAULT_BRAKE_FORWARD_POWER = 0.30;
    /** One-pole filter time constant, seconds; applies ONLY within a braking zone. */
    public static double BRAKE_FILTER_TAU_S = 0.07;

    /** Optional maxSpeed control; never active unless maxSpeed is in the JSON. */
    public static double SPEED_LIMIT_KP = 0.075;
    public static double SPEED_LIMIT_FULL_POWER_BELOW_IN_S = 6.0;
    public static double SPEED_LIMIT_MAX_REVERSE_POWER = 0.35;

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
    /** Maximum lateral offset at the finish line for a pass-through handoff. */
    public static double PASS_MAX_CROSS_ERROR_IN = 8.0;
    public static double STOP_POS_THRESHOLD_IN = 1.0;
    public static double STOP_HEADING_THRESHOLD_DEG = 3.0;
    public static double STOP_SPEED_THRESHOLD_IN_S = 3.0;
    public static double STOP_CAPTURE_DISTANCE_IN = 3.0;
    public static double STOP_CAPTURE_ENTRY_SPEED_IN_S = 6.0;
    public static double STOP_CAPTURE_KP = 0.12;
    public static double STOP_CAPTURE_MAX_POWER = 0.25;
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

    private final SplineController controller = new SplineController();
    private final VelocityEstimator velocityEstimator = new VelocityEstimator();
    private final SplineController.Parameters tuning = new SplineController.Parameters();

    private double segmentEndSpeed = Double.NaN;
    private boolean odometryVelocityReady;

    private final ElapsedTime stallTimer = new ElapsedTime();

    // ---------------------------------------------------------------------
    // Non-blocking active-segment state
    // ---------------------------------------------------------------------

    private SplineGeometry activeGeometry;
    private double activeUnwrappedHeading;
    private boolean activeHeadingOnly;

    private double prevLoopTime;
    private double stallLastProgressS;
    private double bestEndError = Double.POSITIVE_INFINITY;

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

        stopMotor();
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
        controller.resetCrossPid();
        clearActiveSegment();
        rebaseVelocityEstimator();
    }

    /**
     * Prepare one Hermite segment. No motor output is produced here.
     */
    @Override
    public void startMove(PathPoint target) {
        if (target == null) throw new IllegalArgumentException("target == null");

        if (lastSegmentStatus == SegmentStatus.RUNNING) cancel();

        double maxPower = Range.clip(target.maxPower, 0, 1);
        segmentEndSpeed = target.endSpeed;
        double brakeForwardPower = Double.isNaN(target.brakeForwardPower)
                ? Range.clip(DEFAULT_BRAKE_FORWARD_POWER, 0, 1)
                : Range.clip(target.brakeForwardPower, 0, 1);
        brakeForwardPower = Math.min(brakeForwardPower, maxPower);

        this.lx = this.x;
        this.ly = this.y;
        this.ldx = this.dx;
        this.ldy = this.dy;
        this.x = target.x;
        this.y = target.y;
        this.dx = target.dx;
        this.dy = target.dy;
        this.segmentIndex++;

        activeGeometry = new SplineGeometry(this.lx, this.ly, this.ldx, this.ldy,
                this.x, this.y, this.dx, this.dy, ARC_SAMPLES);
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
        controller.startSegment(activeGeometry, this.heading, activeUnwrappedHeading,
                this.x, this.y, maxPower, target.maxSpeed, target.endSpeed,
                target.brakeZoneIn, brakeForwardPower);

        stallTimer.reset();
        lastSegmentStatus = SegmentStatus.RUNNING;
        activeHeadingOnly = activeGeometry.totalLength() < DEGENERATE_ARC_THRESHOLD;
        // Passing through a waypoint keeps the prior filtered velocity and PID history.
        if (!odometryVelocityReady) {
            rebaseVelocityEstimator();
            controller.resetCrossPid();
        }
        stallLastProgressS = 0;
        bestEndError = Double.POSITIVE_INFINITY;
        lastResult = new SegmentResult(SegmentStatus.RUNNING, Double.NaN, Double.NaN);
    }

    private void rebaseVelocityEstimator() {
        robot.odo.update();
        velocityEstimator.rebase(getX(), getY());
        prevLoopTime = System.nanoTime() / 1e9;
        odometryVelocityReady = true;
    }

    /** Copy live dashboard tuning into the hardware-free controller. */
    private SplineController.Parameters currentTuning() {
        tuning.projectionSamples = PROJECTION_SAMPLES;
        tuning.newtonIterations = NEWTON_ITERS;
        tuning.projectionBacktrackIn = PROJECTION_BACKTRACK_IN;
        tuning.projectionForwardIn = PROJECTION_FORWARD_IN;
        tuning.tangentSampleDu = TANGENT_SAMPLE_DU;
        tuning.crossP = K_CROSS_P;
        tuning.crossI = K_CROSS_I;
        tuning.crossD = K_CROSS_D;
        tuning.crossIMaxPower = CROSS_I_MAX_POWER;
        tuning.crossDAlpha = CROSS_D_ALPHA;
        tuning.headingP = K_HEADING;
        tuning.turnDeadArea = TURN_DEAD_AREA;
        tuning.safeBrakeDecel = SAFE_BRAKE_DECEL;
        tuning.brakeReversePower = BRAKE_REVERSE_POWER;
        tuning.brakeFilterTauS = BRAKE_FILTER_TAU_S;
        tuning.speedLimitKp = SPEED_LIMIT_KP;
        tuning.speedLimitFullPowerBelowInS = SPEED_LIMIT_FULL_POWER_BELOW_IN_S;
        tuning.speedLimitMaxReversePower = SPEED_LIMIT_MAX_REVERSE_POWER;
        tuning.stopCaptureDistanceIn = STOP_CAPTURE_DISTANCE_IN;
        tuning.stopCaptureEntrySpeedInS = STOP_CAPTURE_ENTRY_SPEED_IN_S;
        tuning.stopCaptureKp = STOP_CAPTURE_KP;
        tuning.stopCaptureMaxPower = STOP_CAPTURE_MAX_POWER;
        tuning.stopPosThresholdIn = STOP_POS_THRESHOLD_IN;
        tuning.stopHeadingThresholdDeg = STOP_HEADING_THRESHOLD_DEG;
        tuning.stopSpeedThresholdInS = STOP_SPEED_THRESHOLD_IN_S;
        tuning.endUThreshold = END_U_THRESHOLD;
        tuning.passMaxCrossErrorIn = PASS_MAX_CROSS_ERROR_IN;
        tuning.motorVoltage = motorVoltage;
        tuning.voltageCompWeight = VOLTAGE_COMP_WEIGHT;
        tuning.maxTranslationPower = MAX_TRANSLATION_POWER;
        return tuning;
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

        velocityEstimator.update(rx, ry, dt, VELOCITY_FILTER_ALPHA);
        double voltage = robot.getVoltage();
        SplineController.Output output = controller.update(
                new SplineController.Input(rx, ry, curH,
                        velocityEstimator.vx(), velocityEstimator.vy(), voltage, dt),
                currentTuning());
        double closestU = output.closestU;
        double progressS = output.progressS;
        double pathX = output.pathX;
        double pathY = output.pathY;
        double crossError = output.crossError;
        double tangentialVel = output.tangentialVelocity;
        double tangentPower = output.tangentPower;
        double crossPower = output.crossPower;
        double targetH = output.targetHeading;
        double headingError = output.headingError;
        double powerX = output.powerX;
        double powerY = output.powerY;
        double yawPower = output.yawPower;
        double endPosError = output.endPositionError;
        boolean done = output.arrived;

        if (!Globals.DEBUG) {
            robot.odoDrivetrain.driveRobotFieldCentric(powerX, -powerY, -yawPower);
        }
        gotoX = pathX;
        gotoY = pathY;
        gotoH = targetH;

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
        robot.telemetry.addData("progress", String.format("%.1f / %.1f in", progressS, activeGeometry.totalLength()));
        robot.telemetry.addData("crossErr", String.format("%.2f in", crossError));
        robot.telemetry.addData("vTangential", String.format("%.1f in/s", tangentialVel));
        robot.telemetry.addData("brakeZone", output.brakingZone);
        robot.telemetry.addData("endSpeed", segmentEndSpeed);
        robot.telemetry.addData("stopCapture", output.stopCapture);
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
                    + " v=" + String.format("%.1f", tangentialVel)
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
        double error = SplineController.angleDiff(activeUnwrappedHeading, current);
        double absError = Math.abs(error);

        gotoX = getX();
        gotoY = getY();
        gotoH = activeUnwrappedHeading;

        if (absError <= STOP_HEADING_THRESHOLD_DEG) {
            return finishActiveSegment(SegmentStatus.ARRIVED);
        }

        if (absError <= bestEndError - 0.5) {
            bestEndError = absError;
            stallTimer.reset();
        } else if (STALL_TIMEOUT_S > 0 && stallTimer.seconds() >= STALL_TIMEOUT_S) {
            return finishActiveSegment(SegmentStatus.STALLED);
        }

        double yawPower = SplineController.headingPowerFromError(error, K_HEADING, TURN_DEAD_AREA);

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
        // An ordinary waypoint must never force a motor stop. A requested zero-speed
        // endpoint, a heading-only move and any failure do stop the drivetrain.
        if (status != SegmentStatus.ARRIVED || controller.isStopPoint() || activeHeadingOnly) {
            stopMotor();
        }
        lastResult = snapshotResult(status);
        clearActiveSegment();
        return lastResult;
    }

    private SegmentResult snapshotResult(SegmentStatus status) {
        double posError = Math.hypot(this.x - getX(), this.y - getY());
        double headingError = SplineController.angleDiff(this.heading, getHeading());
        return new SegmentResult(status, posError, headingError);
    }

    private void clearActiveSegment() {
        activeGeometry = null;
        controller.clearSegment();
        activeHeadingOnly = false;
    }

    @Override
    public void cancel() {
        if (lastSegmentStatus == SegmentStatus.RUNNING) {
            lastSegmentStatus = SegmentStatus.ABORTED;
            lastResult = snapshotResult(SegmentStatus.ABORTED);
        }
        // Also safe after ARRIVED: used by the loader at waits and route completion.
        stopMotor();
        clearActiveSegment();
    }

    @Override
    public SegmentResult getLastResult() {
        return lastResult;
    }

    public SplineTracker stopMotor() {
        robot.odoDrivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.odoDrivetrain.stopMotor();
        odometryVelocityReady = false;
        return this;
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
