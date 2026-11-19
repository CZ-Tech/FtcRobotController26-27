package org.firstinspires.ftc.teamcode.common.command.auto.spline;

/**
 * Stateful, hardware-independent path following calculation. One update is one
 * control tick; the caller owns the clock, odometry, motor output and stall timer.
 */
public final class SplineController {
    /** Conservative fail-stop bound, not a tracking gain or an auto-recovery radius. */
    public static final double MAX_PROJECTION_TANGENT_ERROR_IN = 3.0;
    /** Tuning snapshot supplied by the FTC adapter each tick (dashboard values stay live). */
    public static final class Parameters {
        public double tangentSampleDu;
        public double crossP, crossI, crossD, crossDAlpha;
        public double headingP, turnDeadArea;
        public double safeBrakeDecel, brakeReversePower, brakeFilterTauS;
        public double speedLimitKp, speedLimitFullPowerBelowInS, speedLimitMaxReversePower;
        public double stopCaptureDistanceIn, stopCaptureEntrySpeedInS;
        public double stopCaptureKp, stopCaptureMaxPower;
        public double stopPosThresholdIn, stopHeadingThresholdDeg, stopSpeedThresholdInS;
        public double endUThreshold, passMaxCrossErrorIn;
        public double motorVoltage, voltageCompWeight;
    }

    /** Measurement passed by the FTC adapter; field coordinates in inches. */
    public static final class Input {
        public final double x, y, heading, vx, vy, voltage, dt;

        public Input(double x, double y, double heading,
                     double vx, double vy, double voltage, double dt) {
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.vx = vx;
            this.vy = vy;
            this.voltage = voltage;
            this.dt = dt;
        }
    }

    /** Field-centric motor request plus values useful for monitoring and assertions. */
    public static final class Output {
        public final double powerX, powerY, yawPower;
        public final double pathX, pathY, targetHeading, headingError;
        public final double closestU, progressS, crossError, tangentialVelocity;
        public final double tangentPower, crossPower, endPositionError;
        public final boolean brakingZone, stopCapture, arrived;
        public final double projectionTangentError;
        public final boolean projectionFault;
        // Pre-guard data from this exact tick, including when output power is suppressed.
        public final SplineGeometry.Projection projection;
        public final double previousU, remainingArc, brakeTriggerDistance;
        public final boolean brakeReverseRequested;
        public final double candidateTangentPower, candidateCrossPower;

        private Output(double powerX, double powerY, double yawPower,
                       double pathX, double pathY, double targetHeading, double headingError,
                       double closestU, double progressS, double crossError, double tangentialVelocity,
                       double tangentPower, double crossPower, double endPositionError,
                       boolean brakingZone, boolean stopCapture, boolean arrived,
                       double projectionTangentError, boolean projectionFault,
                       SplineGeometry.Projection projection, double previousU, double remainingArc,
                       double brakeTriggerDistance, boolean brakeReverseRequested,
                       double candidateTangentPower, double candidateCrossPower) {
            this.powerX = powerX;
            this.powerY = powerY;
            this.yawPower = yawPower;
            this.pathX = pathX;
            this.pathY = pathY;
            this.targetHeading = targetHeading;
            this.headingError = headingError;
            this.closestU = closestU;
            this.progressS = progressS;
            this.crossError = crossError;
            this.tangentialVelocity = tangentialVelocity;
            this.tangentPower = tangentPower;
            this.crossPower = crossPower;
            this.endPositionError = endPositionError;
            this.brakingZone = brakingZone;
            this.stopCapture = stopCapture;
            this.arrived = arrived;
            this.projectionTangentError = projectionTangentError;
            this.projectionFault = projectionFault;
            this.projection = projection;
            this.previousU = previousU;
            this.remainingArc = remainingArc;
            this.brakeTriggerDistance = brakeTriggerDistance;
            this.brakeReverseRequested = brakeReverseRequested;
            this.candidateTangentPower = candidateTangentPower;
            this.candidateCrossPower = candidateCrossPower;
        }
    }

    private static final class PIDState {
        double integral, lastError, filteredDerivative;
        boolean firstRun = true;

        void reset() {
            integral = 0;
            lastError = 0;
            filteredDerivative = 0;
            firstRun = true;
        }
    }

    private final PIDState crossPID = new PIDState();
    private SplineGeometry geometry;
    private double[] headingSpline;
    private double endX, endY, endHeading;
    private double segmentMaxPower, segmentMaxSpeed, segmentEndSpeed;
    private double segmentBrakeZoneIn, segmentBrakeForwardPower;
    private double brakeFilteredPower;
    private boolean wasInBrakeZone, stopCapture, projectionFault;
    private double closestU, progressS;

    public void resetCrossPid() { crossPID.reset(); }

    public void clearSegment() {
        geometry = null;
        headingSpline = null;
    }

    /** Does not reset the PID; pass-through motion deliberately keeps its history. */
    public void startSegment(SplineGeometry geometry, double startHeading, double unwrappedEndHeading,
                             double endX, double endY, double maxPower, double maxSpeed,
                             double endSpeed, double brakeZoneIn, double brakeForwardPower) {
        this.geometry = geometry;
        this.headingSpline = SplineGeometry.hermite(startHeading, 0, unwrappedEndHeading, 0);
        this.endX = endX;
        this.endY = endY;
        this.endHeading = unwrappedEndHeading;
        segmentMaxPower = maxPower;
        segmentMaxSpeed = maxSpeed;
        segmentEndSpeed = endSpeed;
        segmentBrakeZoneIn = brakeZoneIn;
        segmentBrakeForwardPower = brakeForwardPower;
        wasInBrakeZone = false;
        stopCapture = false;
        projectionFault = false;
        closestU = 0;
        progressS = 0;
    }

    public boolean isStopPoint() {
        return !Double.isNaN(segmentEndSpeed) && segmentEndSpeed <= 1e-6;
    }

    public Output update(Input input, Parameters p) {
        if (!Double.isFinite(input.x) || !Double.isFinite(input.y)
                || !Double.isFinite(input.heading) || !Double.isFinite(input.vx)
                || !Double.isFinite(input.vy)) {
            throw new IllegalArgumentException("Non-finite motion measurement");
        }
        double rx = input.x;
        double ry = input.y;
        double curH = input.heading;
        double dt = input.dt;

        double previousU = closestU;
        SplineGeometry.Projection projection = geometry.project(rx, ry, previousU);
        closestU = projection.u;
        if (projection.s > progressS) progressS = projection.s;
        double progressU = geometry.uAtS(progressS);

        double pathX = projection.x;
        double pathY = projection.y;
        double tx = projection.tx;
        double ty = projection.ty;
        double nx = -ty;
        double ny = tx;

        double crossError = (pathX - rx) * nx + (pathY - ry) * ny;
        double tangentialVel = input.vx * tx + input.vy * ty;
        double remainingArc = Math.max(0, geometry.totalLength() - progressS);
        double speedMag = Math.hypot(input.vx, input.vy);
        boolean autoBrakeZone = Double.isNaN(segmentBrakeZoneIn);
        double forwardSpeed = Math.max(0, tangentialVel);
        double triggerDistance = brakeTriggerDistance(forwardSpeed, autoBrakeZone, p);
        // The automatic zone follows the current speed until entered, then stays
        // active so deceleration does not make the zone shrink out from under us.
        boolean inBrakeZone = !Double.isNaN(segmentEndSpeed)
                && (autoBrakeZone
                    ? wasInBrakeZone || (forwardSpeed > segmentEndSpeed
                            && remainingArc <= triggerDistance)
                    : remainingArc <= segmentBrakeZoneIn);
        boolean reverse = inBrakeZone && forwardSpeed > segmentEndSpeed
                && triggerDistance >= remainingArc;
        double tangentPower = tangentialPower(tangentialVel, dt, p, inBrakeZone, reverse);

        double motorPowerGain = 1.0;
        if (input.voltage > 1e-6) {
            double rawGain = Math.abs(p.motorVoltage / input.voltage);
            motorPowerGain = 1.0 + (rawGain - 1.0) * p.voltageCompWeight;
        }
        double dx = endX - rx, dy = endY - ry;
        double endPosError = Math.hypot(dx, dy);
        if (!stopCapture && isStopPoint() && remainingArc <= p.stopCaptureDistanceIn
                && speedMag <= p.stopCaptureEntrySpeedInS) {
            stopCapture = true;
            resetCrossPid();
        }
        double crossPower;
        if (stopCapture) {
            // Capture targets the endpoint, not the nearest path point. Use one gain
            // in both axes so neither curvature nor saturation can rotate the request.
            double limit = Math.min(segmentMaxPower, clip01(p.stopCaptureMaxPower));
            // Apply the radial limit to the whole translation, including any voltage boost.
            double maxRequest = limit / Math.max(1.0, motorPowerGain);
            double gain = endPosError == 0 ? 0 : Math.min(Math.max(0, p.stopCaptureKp),
                    maxRequest / endPosError);
            tangentPower = gain * (dx * tx + dy * ty);
            crossPower = gain * (dx * nx + dy * ny);
        } else {
            crossPower = computePID(crossError, dt, p, crossPID);
        }
        double candidateTangentPower = tangentPower;
        double candidateCrossPower = crossPower;

        // Preserve the tangent/normal ratio. Only the drivetrain normalizes
        // the combined wheel requests to the physical motor range.
        double targetH = SplineGeometry.eval(headingSpline, progressU);
        double headingError = angleDiff(targetH, curH);
        double yawPower = headingPowerFromError(headingError, p.headingP, p.turnDeadArea);
        double powerX = tangentPower * tx + crossPower * nx;
        double powerY = tangentPower * ty + crossPower * ny;

        powerX *= motorPowerGain;
        powerY *= motorPowerGain;
        yawPower *= motorPowerGain;

        boolean done;
        if (isStopPoint()) {
            done = endPosError <= p.stopPosThresholdIn
                    && speedMag <= p.stopSpeedThresholdInS
                    && Math.abs(angleDiff(endHeading, curH)) <= p.stopHeadingThresholdDeg;
        } else {
            double[] endTangent = geometry.unitTangent(1.0, p.tangentSampleDu);
            double endDx = rx - endX;
            double endDy = ry - endY;
            double crossedNormal = endDx * endTangent[0] + endDy * endTangent[1];
            double crossAtEnd = -endDx * endTangent[1] + endDy * endTangent[0];
            done = closestU >= p.endUThreshold && crossedNormal >= 0
                    && Math.abs(crossAtEnd) <= p.passMaxCrossErrorIn;
        }
        // An interior nearest point has zero tangential residual. A constrained
        // or inaccurate projection need not: normal-only control would ignore
        // that displacement and could keep cruising away from its reference.
        // Allow a valid pass-through handoff, but never silently resume a fault.
        double projectionTangentError = (pathX - rx) * tx + (pathY - ry) * ty;
        if (!done && (!Double.isFinite(projectionTangentError)
                || Math.abs(projectionTangentError) > MAX_PROJECTION_TANGENT_ERROR_IN)) {
            projectionFault = true;
        }
        if (projectionFault) {
            powerX = powerY = yawPower = 0;
            tangentPower = crossPower = 0;
            done = false;
        }
        return new Output(powerX, powerY, yawPower,
                pathX, pathY, targetH, headingError, closestU, progressS,
                crossError, tangentialVel, tangentPower, crossPower, endPosError,
                wasInBrakeZone, stopCapture, done, projectionTangentError, projectionFault,
                projection, previousU, remainingArc, triggerDistance, reverse,
                candidateTangentPower, candidateCrossPower);
    }

    public static double angleDiff(double target, double current) {
        if (!Double.isFinite(target) || !Double.isFinite(current)) {
            throw new IllegalArgumentException("Non-finite heading");
        }
        double diff = (target % 360 - current % 360) % 360;
        if (diff > 180) diff -= 360;
        if (diff <= -180) diff += 360;
        return diff;
    }

    public static double headingPowerFromError(double error, double kp, double deadArea) {
        double yawPower = error * kp;
        if (Math.abs(error) < deadArea) yawPower = 0;
        return clip(yawPower, -1, 1);
    }

    private static double computePID(double error, double dt, Parameters p, PIDState state) {
        if (dt <= 1e-5 || dt > 0.25) dt = 0.02;
        if (state.firstRun) {
            state.firstRun = false;
            state.lastError = error;
            state.filteredDerivative = 0;
        }
        if (p.crossI != 0) {
            state.integral += error * dt;
        } else {
            state.integral = 0;
        }
        double rawDerivative = (error - state.lastError) / dt;
        double alpha = clip01(p.crossDAlpha);
        state.filteredDerivative = alpha * rawDerivative
                + (1 - alpha) * state.filteredDerivative;
        state.lastError = error;
        double iTerm = p.crossI * state.integral;
        return p.crossP * error + iTerm + p.crossD * state.filteredDerivative;
    }

    private double brakeTriggerDistance(double forwardSpeed, boolean autoBrakeZone, Parameters p) {
        double brakeDistance = 0;
        if (!Double.isNaN(segmentEndSpeed)) {
            double excessSpeedSq = Math.max(0,
                    forwardSpeed * forwardSpeed - segmentEndSpeed * segmentEndSpeed);
            brakeDistance = excessSpeedSq / (2 * Math.max(1e-3, p.safeBrakeDecel));
        }
        return autoBrakeZone ? brakeDistance * 1.05 : brakeDistance;
    }

    private double tangentialPower(double tangentialVel, double dt, Parameters p,
                                   boolean inBrakeZone, boolean reverse) {
        double request;
        if (!inBrakeZone) {
            wasInBrakeZone = false;
            request = segmentMaxPower;
        } else {
            request = reverse ? -clip(p.brakeReversePower, 0, 1) : segmentBrakeForwardPower;
            if (!wasInBrakeZone) {
                brakeFilteredPower = request;
                wasInBrakeZone = true;
            } else {
                double tau = Math.max(0, p.brakeFilterTauS);
                double alpha = tau <= 1e-6 ? 1 : dt / (tau + dt);
                brakeFilteredPower += alpha * (request - brakeFilteredPower);
            }
            request = brakeFilteredPower;
        }
        if (!Double.isNaN(segmentMaxSpeed)
                && tangentialVel >= segmentMaxSpeed - p.speedLimitFullPowerBelowInS) {
            double speedLimitedPower = clip(p.speedLimitKp * (segmentMaxSpeed - tangentialVel),
                    -Math.max(0, p.speedLimitMaxReversePower), segmentMaxPower);
            request = Math.min(request, speedLimitedPower);
        }
        return request;
    }

    private static double clip01(double x) { return Math.max(0, Math.min(1, x)); }
    private static double clip(double x, double low, double high) {
        return Math.max(low, Math.min(high, x));
    }
}
