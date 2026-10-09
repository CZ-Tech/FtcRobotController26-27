package org.firstinspires.ftc.teamcode.common.command.auto.spline;

/**
 * Stateful, hardware-independent path following calculation. One update is one
 * control tick; the caller owns the clock, odometry, motor output and stall timer.
 */
public final class SplineController {
    /** Tuning snapshot supplied by the FTC adapter each tick (dashboard values stay live). */
    public static final class Parameters {
        public int projectionSamples, newtonIterations;
        public double projectionBacktrackIn, projectionForwardIn, tangentSampleDu;
        public double crossP, crossI, crossD, crossIMaxPower, crossDAlpha;
        public double headingP, turnDeadArea;
        public double safeBrakeDecel, brakeReversePower, brakeFilterTauS;
        public double speedLimitKp, speedLimitFullPowerBelowInS, speedLimitMaxReversePower;
        public double stopCaptureDistanceIn, stopCaptureEntrySpeedInS;
        public double stopCaptureKp, stopCaptureMaxPower;
        public double stopPosThresholdIn, stopHeadingThresholdDeg, stopSpeedThresholdInS;
        public double endUThreshold, passMaxCrossErrorIn;
        public double motorVoltage, voltageCompWeight, maxTranslationPower;
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

        private Output(double powerX, double powerY, double yawPower,
                       double pathX, double pathY, double targetHeading, double headingError,
                       double closestU, double progressS, double crossError, double tangentialVelocity,
                       double tangentPower, double crossPower, double endPositionError,
                       boolean brakingZone, boolean stopCapture, boolean arrived) {
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
    private boolean wasInBrakeZone, stopCapture;
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
        closestU = 0;
        progressS = 0;
    }

    public boolean isStopPoint() {
        return !Double.isNaN(segmentEndSpeed) && segmentEndSpeed <= 1e-6;
    }

    public Output update(Input input, Parameters p) {
        double rx = input.x;
        double ry = input.y;
        double curH = input.heading;
        double dt = input.dt;

        closestU = geometry.closestU(rx, ry, closestU,
                p.projectionBacktrackIn, p.projectionForwardIn,
                p.projectionSamples, p.newtonIterations);
        double closestS = geometry.sAtU(closestU);
        if (closestS > progressS) progressS = closestS;
        double progressU = geometry.uAtS(progressS);

        double pathX = geometry.xAt(closestU);
        double pathY = geometry.yAt(closestU);
        double[] tangent = geometry.unitTangent(closestU, p.tangentSampleDu);
        double tx = tangent[0];
        double ty = tangent[1];
        double nx = -ty;
        double ny = tx;

        double crossError = (pathX - rx) * nx + (pathY - ry) * ny;
        double crossPower = computePID(crossError, dt, p, crossPID);
        double tangentialVel = input.vx * tx + input.vy * ty;
        double remainingArc = Math.max(0, geometry.totalLength() - progressS);
        double speedMag = Math.hypot(input.vx, input.vy);
        double tangentPower = tangentialPower(tangentialVel, speedMag, remainingArc,
                rx, ry, tx, ty, dt, p);

        tangentPower = clip(tangentPower, -1, 1);
        crossPower = clip(crossPower, -1, 1);
        double targetH = SplineGeometry.eval(headingSpline, progressU);
        double headingError = angleDiff(targetH, curH);
        double yawPower = headingPowerFromError(headingError, p.headingP, p.turnDeadArea);
        double powerX = tangentPower * tx + crossPower * nx;
        double powerY = tangentPower * ty + crossPower * ny;

        double motorPowerGain = 1.0;
        if (input.voltage > 1e-6) {
            double rawGain = Math.abs(p.motorVoltage / input.voltage);
            motorPowerGain = 1.0 + (rawGain - 1.0) * p.voltageCompWeight;
        }
        powerX *= motorPowerGain;
        powerY *= motorPowerGain;
        yawPower *= motorPowerGain;

        double translationMag = Math.hypot(powerX, powerY);
        double maxTranslation = clip(p.maxTranslationPower, 0, 1);
        if (translationMag > maxTranslation && translationMag > 1e-9) {
            double scale = maxTranslation / translationMag;
            powerX *= scale;
            powerY *= scale;
        }
        yawPower = clip(yawPower, -1, 1);

        double endPosError = Math.hypot(endX - rx, endY - ry);
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
        return new Output(powerX, powerY, yawPower,
                pathX, pathY, targetH, headingError, closestU, progressS,
                crossError, tangentialVel, tangentPower, crossPower, endPosError,
                wasInBrakeZone, stopCapture, done);
    }

    public static double angleDiff(double target, double current) {
        double diff = target - current;
        while (diff > 180) diff -= 360;
        while (diff <= -180) diff += 360;
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
            double iTerm = p.crossI * state.integral;
            if (p.crossIMaxPower > 0) {
                double clipped = clip(iTerm, -p.crossIMaxPower, p.crossIMaxPower);
                if (clipped != iTerm) state.integral = clipped / p.crossI;
            }
        } else {
            state.integral = 0;
        }
        double rawDerivative = (error - state.lastError) / dt;
        double alpha = clip01(p.crossDAlpha);
        state.filteredDerivative = alpha * rawDerivative
                + (1 - alpha) * state.filteredDerivative;
        state.lastError = error;
        double iTerm = p.crossI * state.integral;
        if (p.crossIMaxPower > 0) iTerm = clip(iTerm, -p.crossIMaxPower, p.crossIMaxPower);
        return p.crossP * error + iTerm + p.crossD * state.filteredDerivative;
    }

    private double tangentialPower(double tangentialVel, double speedMag,
                                   double remainingArc, double rx, double ry,
                                   double tx, double ty, double dt, Parameters p) {
        boolean inBrakeZone = !Double.isNaN(segmentEndSpeed)
                && remainingArc <= segmentBrakeZoneIn;
        double request;
        if (!inBrakeZone) {
            wasInBrakeZone = false;
            request = segmentMaxPower;
        } else {
            double forwardSpeed = Math.max(0, tangentialVel);
            double excessSpeedSq = Math.max(0,
                    forwardSpeed * forwardSpeed - segmentEndSpeed * segmentEndSpeed);
            double brakeDistance = excessSpeedSq / (2 * Math.max(1e-3, p.safeBrakeDecel));
            boolean brake = forwardSpeed > segmentEndSpeed && brakeDistance >= remainingArc;
            request = brake ? -clip(p.brakeReversePower, 0, 1) : segmentBrakeForwardPower;
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
        if (isStopPoint() && (stopCapture ||
                (remainingArc <= p.stopCaptureDistanceIn
                        && speedMag <= p.stopCaptureEntrySpeedInS))) {
            stopCapture = true;
            double[] endTangent = geometry.unitTangent(1.0, p.tangentSampleDu);
            double alongError = (endX - rx) * endTangent[0]
                    + (endY - ry) * endTangent[1];
            request = clip(p.stopCaptureKp * alongError,
                    -p.stopCaptureMaxPower, p.stopCaptureMaxPower);
        }
        return clip(request, -1, 1);
    }

    private static double clip01(double x) { return Math.max(0, Math.min(1, x)); }
    private static double clip(double x, double low, double high) {
        return Math.max(low, Math.min(high, x));
    }
}
