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
import org.firstinspires.ftc.teamcode.common.drive.MixedOdo;
import org.firstinspires.ftc.teamcode.common.drive.OdoDrivetrain;

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
        /** Braking-zone length in inches; NaN automatically uses braking distance + 5%. */
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

    // ---------------------------------------------------------------------
    // Public path state (kept compatible with the previous class)
    // ---------------------------------------------------------------------
    public double[] startPoint;
    public double lx, ly, ldx, ldy;
    public double x, y, dx, dy;
    public double heading, preH;

    // ---------------------------------------------------------------------
    // Tuning
    // ---------------------------------------------------------------------
    // Every path-following, projection, completion, safety and voltage constant now
    // lives in Globals (see the "Spline autonomous path following" section at the end
    // of that file). currentTuning() copies the live values into
    // SplineController.Parameters once per control tick, so editing Globals at runtime
    // takes effect on the next tick.

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
    private final SplineDiagnostics diagnostics;
    private final OdoDrivetrain.DriveTrace driveTrace;
    private final MixedOdo.PinpointSample pinpointSample;

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
        this(robot, new SplineDiagnostics(Globals.Auto.LOGCAT_DIAGNOSTICS));
    }

    public SplineTracker(Robot robot, SplineDiagnostics diagnostics) {
        this.robot = robot;
        this.diagnostics = diagnostics;
        this.driveTrace = diagnostics.enabled() ? new OdoDrivetrain.DriveTrace() : null;
        this.pinpointSample = diagnostics.enabled() ? new MixedOdo.PinpointSample() : null;
        robot.odoDrivetrain.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.odoDrivetrain.setZeroPowerBehavior(Globals.Auto.ZERO_POWER_BEHAVIOR);
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
        if (diagnostics.enabled()) {
            diagnostics.event("BEGIN", System.nanoTime(), "targetInDeg=" + SplineDiagnostics.pose(pose)
                    + " poseInDeg=" + SplineDiagnostics.pose(getPosition())
                    + " rawInDeg=" + SplineDiagnostics.pose(robot.odo.getLastRawFieldInput())
                    + " pinpoint[" + SplineDiagnostics.pinpoint(pinpointSample) + "]");
        }
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
                ? Range.clip(Globals.Auto.DEFAULT_BRAKE_FORWARD_POWER, 0, 1)
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
                this.x, this.y, this.dx, this.dy, Globals.Auto.ARC_SAMPLES);
        activeUnwrappedHeading = this.heading
                + SplineController.angleDiff(target.heading, this.heading);
        controller.startSegment(activeGeometry, this.heading, activeUnwrappedHeading,
                this.x, this.y, maxPower, target.maxSpeed, target.endSpeed,
                target.brakeZoneIn, brakeForwardPower);

        stallTimer.reset();
        lastSegmentStatus = SegmentStatus.RUNNING;
        activeHeadingOnly = activeGeometry.totalLength() < Globals.Auto.DEGENERATE_ARC_THRESHOLD;
        // Passing through a waypoint keeps the prior filtered velocity and PID history.
        if (!odometryVelocityReady) {
            rebaseVelocityEstimator();
            controller.resetCrossPid();
        }
        stallLastProgressS = 0;
        bestEndError = Double.POSITIVE_INFINITY;
        lastResult = new SegmentResult(SegmentStatus.RUNNING, Double.NaN, Double.NaN);
        diagnostics.newSegment();
        if (diagnostics.enabled()) {
            diagnostics.event("SEGMENT_START", System.nanoTime(), "seg=" + segmentIndex
                    + " startInDeg=(" + lx + "," + ly + "," + heading + ")"
                    + " endInDeg=(" + x + "," + y + "," + activeUnwrappedHeading + ")"
                    + " startDXY=(" + ldx + "," + ldy + ") endDXY=(" + dx + "," + dy + ")"
                    + " lengthIn=" + activeGeometry.totalLength() + " headingOnly=" + activeHeadingOnly
                    + " maxPower=" + maxPower + " maxSpeedInS=" + target.maxSpeed
                    + " endSpeedInS=" + target.endSpeed + " brakeZoneIn=" + target.brakeZoneIn
                    + " brakeForwardPower=" + brakeForwardPower
                    + " safeBrakeDecelInS2=" + Globals.Auto.SAFE_BRAKE_DECEL
                    + " brakeReversePower=" + Globals.Auto.BRAKE_REVERSE_POWER
                    + " brakeFilterTauS=" + Globals.Auto.BRAKE_FILTER_TAU_S
                    + " captureDistanceIn=" + Globals.Auto.STOP_CAPTURE_DISTANCE_IN
                    + " captureSpeedInS=" + Globals.Auto.STOP_CAPTURE_ENTRY_SPEED_IN_S
                    + " projectionLimitIn=" + SplineController.MAX_PROJECTION_TANGENT_ERROR_IN
                    + " crossPID=(" + Globals.Auto.K_CROSS_P + "," + Globals.Auto.K_CROSS_I
                    + "," + Globals.Auto.K_CROSS_D + ") debug=" + Globals.DEBUG);
        }
    }

    private void rebaseVelocityEstimator() {
        Pose2D pose = robot.odo.updateAndGetPosition(pinpointSample);
        velocityEstimator.rebase(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH));
        prevLoopTime = System.nanoTime() / 1e9;
        odometryVelocityReady = true;
    }

    /** Copy live Globals tuning into the hardware-free controller. */
    private SplineController.Parameters currentTuning() {
        tuning.tangentSampleDu = Globals.Auto.TANGENT_SAMPLE_DU;
        tuning.crossP = Globals.Auto.K_CROSS_P;
        tuning.crossI = Globals.Auto.K_CROSS_I;
        tuning.crossD = Globals.Auto.K_CROSS_D;
        tuning.crossDAlpha = Globals.Auto.CROSS_D_ALPHA;
        tuning.headingP = Globals.Auto.K_HEADING;
        tuning.turnDeadArea = Globals.Auto.TURN_DEAD_AREA;
        tuning.safeBrakeDecel = Globals.Auto.SAFE_BRAKE_DECEL;
        tuning.brakeReversePower = Globals.Auto.BRAKE_REVERSE_POWER;
        tuning.brakeFilterTauS = Globals.Auto.BRAKE_FILTER_TAU_S;
        tuning.speedLimitKp = Globals.Auto.SPEED_LIMIT_KP;
        tuning.speedLimitFullPowerBelowInS = Globals.Auto.SPEED_LIMIT_FULL_POWER_BELOW_IN_S;
        tuning.speedLimitMaxReversePower = Globals.Auto.SPEED_LIMIT_MAX_REVERSE_POWER;
        tuning.stopCaptureDistanceIn = Globals.Auto.STOP_CAPTURE_DISTANCE_IN;
        tuning.stopCaptureEntrySpeedInS = Globals.Auto.STOP_CAPTURE_ENTRY_SPEED_IN_S;
        tuning.stopCaptureKp = Globals.Auto.STOP_CAPTURE_KP;
        tuning.stopCaptureMaxPower = Globals.Auto.STOP_CAPTURE_MAX_POWER;
        tuning.stopPosThresholdIn = Globals.Auto.STOP_POS_THRESHOLD_IN;
        tuning.stopHeadingThresholdDeg = Globals.Auto.STOP_HEADING_THRESHOLD_DEG;
        tuning.stopSpeedThresholdInS = Globals.Auto.STOP_SPEED_THRESHOLD_IN_S;
        tuning.endUThreshold = Globals.Auto.END_U_THRESHOLD;
        tuning.passMaxCrossErrorIn = Globals.Auto.PASS_MAX_CROSS_ERROR_IN;
        tuning.motorVoltage = Globals.Auto.motorVoltage;
        tuning.voltageCompWeight = Globals.Auto.VOLTAGE_COMP_WEIGHT;
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

        Pose2D odomPose = robot.odo.updateAndGetPosition(pinpointSample);
        if (robot.odo.hasOdometryFault()) {
            // A missing pod pins one axis at zero, so the follower would keep requesting a correction
            // it can never observe and would drift the full segment before tripping a projection
            // guard. Stop here and name the real cause.
            SegmentResult failed = finishActiveSegment(SegmentStatus.ABORTED);
            diagnostics.event("ODOMETRY_FAULT", System.nanoTime(), "seg=" + segmentIndex
                    + " " + SplineDiagnostics.pinpoint(pinpointSample));
            robot.telemetry.addData("odometryFault", "Pinpoint reports a pod or IMU fault");
            Log.e("SplineTracker", "Odometry fault: " + SplineDiagnostics.pinpoint(pinpointSample));
            return failed;
        }

        long nowNanos = System.nanoTime();
        diagnostics.odometry(nowNanos, segmentIndex, pinpointSample);
        double now = nowNanos / 1e9;
        double observedDt = now - prevLoopTime;
        double dt = observedDt;
        if (dt <= 1e-5 || dt > 0.25) dt = 0.02;
        prevLoopTime = now;

        double rx = odomPose.getX(DistanceUnit.INCH);
        double ry = odomPose.getY(DistanceUnit.INCH);
        double curH = odomPose.getHeading(AngleUnit.DEGREES);
        if (!Double.isFinite(rx) || !Double.isFinite(ry) || !Double.isFinite(curH)) {
            SegmentResult failed = finishActiveSegment(SegmentStatus.ABORTED);
            if (diagnostics.enabled()) {
                diagnostics.event("INVALID_POSE", nowNanos, "seg=" + segmentIndex
                        + " poseInDeg=(" + rx + "," + ry + "," + curH + ")");
            }
            return failed;
        }

        velocityEstimator.update(rx, ry, dt, Globals.Auto.VELOCITY_FILTER_ALPHA);
        double voltage = robot.getVoltage();
        SplineController.Input input = new SplineController.Input(rx, ry, curH,
                velocityEstimator.vx(), velocityEstimator.vy(), voltage, dt);
        SplineController.Output output = controller.update(input, currentTuning());
        Pose2D raw = diagnostics.enabled() ? robot.odo.getLastRawFieldInput() : null;
        if (output.projectionFault) {
            // Stop hardware before any telemetry/logging that could block.
            SegmentResult failed = finishActiveSegment(SegmentStatus.ABORTED);
            diagnostics.control(nowNanos, segmentIndex, observedDt, input, output, raw, pinpointSample, null,
                    "PROJECTION_FAULT");
            robot.telemetry.addData("projectionFault", "Tangential projection residual exceeded limit");
            robot.telemetry.addData("projectionTangentErr", "%.2f in", output.projectionTangentError);
            Log.e("SplineTracker", "Projection fault: tangent residual=" + output.projectionTangentError);
            return failed;
        }
        robot.telemetry.addData("projectionTangentErr", "%.2f in", output.projectionTangentError);
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

        OdoDrivetrain.DriveTrace sent = null;
        if (!Globals.DEBUG) {
            robot.odoDrivetrain.driveRobotFieldCentric(powerX, -powerY, -yawPower, driveTrace);
            sent = driveTrace;
        }
        gotoX = pathX;
        gotoY = pathY;
        gotoH = targetH;

        if (Globals.Auto.STALL_TIMEOUT_S > 0) {
            boolean progressed = progressS >= stallLastProgressS + Globals.Auto.STALL_PROGRESS_EPS_IN;
            boolean endImproved = endPosError <= bestEndError - Globals.Auto.STALL_END_ERROR_EPS_IN;
            if (progressed || endImproved) {
                if (progressed) stallLastProgressS = progressS;
                if (endPosError < bestEndError) bestEndError = endPosError;
                stallTimer.reset();
            } else if (stallTimer.seconds() >= Globals.Auto.STALL_TIMEOUT_S && !done) {
                SegmentResult stalled = finishActiveSegment(SegmentStatus.STALLED);
                diagnostics.control(nowNanos, segmentIndex, observedDt, input, output, raw, pinpointSample,
                        sent, "STALLED");
                return stalled;
            }
        }

        if (done) {
            SegmentResult arrived = finishActiveSegment(SegmentStatus.ARRIVED);
            diagnostics.control(nowNanos, segmentIndex, observedDt, input, output, raw, pinpointSample,
                    sent, "ARRIVED");
            return arrived;
        }
        diagnostics.control(nowNanos, segmentIndex, observedDt, input, output, raw, pinpointSample,
                sent, null);

        robot.telemetry.addData("seg", segmentIndex);
        robot.telemetry.addData("poseX", "%.2f in", rx);
        robot.telemetry.addData("poseY", "%.2f in", ry);
        robot.telemetry.addData("poseH", "%.1f deg", curH);
        robot.telemetry.addData("pathX", "%.2f in", pathX);
        robot.telemetry.addData("pathY", "%.2f in", pathY);
        robot.telemetry.addData("cmdX", "%.3f", powerX);
        robot.telemetry.addData("cmdY", "%.3f", powerY);
        robot.telemetry.addData("cmdYaw", "%.3f", yawPower);
        // Format-string overload: the Driver Station shows the same text as before, while the
        // network API also receives the raw number (and the unit) so it can be plotted.
        robot.telemetry.addData("uClosest", "%.4f", closestU);
        robot.telemetry.addData("progress", String.format("%.1f / %.1f in", progressS, activeGeometry.totalLength()));
        robot.telemetry.addData("crossErr", "%.2f in", crossError);
        robot.telemetry.addData("vTangential", "%.1f in/s", tangentialVel);
        robot.telemetry.addData("brakeZone", output.brakingZone);
        robot.telemetry.addData("endSpeed", segmentEndSpeed);
        robot.telemetry.addData("stopCapture", output.stopCapture);
        robot.telemetry.addData("pTangent", "%.3f", tangentPower);
        robot.telemetry.addData("pCross", "%.3f", crossPower);
        robot.telemetry.addData("targetH", "%.1f", targetH);
        robot.telemetry.addData("headingErr", "%.1f", headingError);
        robot.telemetry.addData("endErr", "%.2f in", endPosError);
        robot.telemetry.addData("cmdXY", String.format("%.3f, %.3f", powerX, powerY));
        robot.telemetry.addData("Voltage", voltage);

        if (Globals.Auto.LOG_VERBOSE) {
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
        robot.odo.updateAndGetPosition(pinpointSample);
        double current = getHeading();
        if (!Double.isFinite(current)) {
            return finishActiveSegment(SegmentStatus.ABORTED);
        }
        double error = SplineController.angleDiff(activeUnwrappedHeading, current);
        double absError = Math.abs(error);

        gotoX = getX();
        gotoY = getY();
        gotoH = activeUnwrappedHeading;

        if (absError <= Globals.Auto.STOP_HEADING_THRESHOLD_DEG) {
            return finishActiveSegment(SegmentStatus.ARRIVED);
        }

        if (absError <= bestEndError - 0.5) {
            bestEndError = absError;
            stallTimer.reset();
        } else if (Globals.Auto.STALL_TIMEOUT_S > 0 && stallTimer.seconds() >= Globals.Auto.STALL_TIMEOUT_S) {
            return finishActiveSegment(SegmentStatus.STALLED);
        }

        double yawPower = SplineController.headingPowerFromError(
                error, Globals.Auto.K_HEADING, Globals.Auto.TURN_DEAD_AREA);

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
        if (diagnostics.enabled()) {
            diagnostics.event("SEGMENT_END", System.nanoTime(), "seg=" + segmentIndex
                    + " status=" + status + " poseInDeg=" + SplineDiagnostics.pose(getPosition())
                    + " endErrIn=" + lastResult.finalPositionError
                    + " headingErrDeg=" + lastResult.finalHeadingError);
        }
        clearActiveSegment();
        return lastResult;
    }

    private SegmentResult snapshotResult(SegmentStatus status) {
        double posError = Math.hypot(this.x - getX(), this.y - getY());
        double currentHeading = getHeading();
        double headingError = Double.isFinite(currentHeading)
                ? SplineController.angleDiff(this.heading, currentHeading) : Double.NaN;
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
        DcMotor.ZeroPowerBehavior zeroBehavior = Globals.Auto.ZERO_POWER_BEHAVIOR;
        robot.odoDrivetrain.setZeroPowerBehavior(zeroBehavior);
        robot.odoDrivetrain.stopMotor();
        odometryVelocityReady = false;
        if (diagnostics.enabled()) {
            diagnostics.event("ZERO", System.nanoTime(), "seg=" + segmentIndex
                    + " wheelLFRFLBRB=(0,0,0,0) behavior=" + zeroBehavior);
        }
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
