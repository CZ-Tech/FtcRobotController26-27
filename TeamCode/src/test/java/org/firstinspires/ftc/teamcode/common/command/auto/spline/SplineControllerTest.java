package org.firstinspires.ftc.teamcode.common.command.auto.spline;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Before;
import org.junit.Test;

public class SplineControllerTest {
    private SplineController controller;
    private SplineController.Parameters p;
    private SplineGeometry straight;

    @Before
    public void setUp() {
        controller = new SplineController();
        straight = new SplineGeometry(0, 0, 20, 0,
                20, 0, 20, 0, 240);
        p = new SplineController.Parameters();
        p.tangentSampleDu = 0.01;
        p.crossP = 0.10;
        p.crossI = 0;
        p.crossD = 0.010;
        p.crossDAlpha = 0.25;
        p.headingP = 0.02;
        p.turnDeadArea = 0;
        p.safeBrakeDecel = 30;
        p.brakeReversePower = 0.75;
        p.brakeFilterTauS = 0.07;
        p.speedLimitKp = 0.075;
        p.speedLimitFullPowerBelowInS = 6;
        p.speedLimitMaxReversePower = 0.35;
        p.stopCaptureDistanceIn = 3;
        p.stopCaptureEntrySpeedInS = 6;
        p.stopCaptureKp = 0.12;
        p.stopCaptureMaxPower = 0.25;
        p.stopPosThresholdIn = 1;
        p.stopHeadingThresholdDeg = 3;
        p.stopSpeedThresholdInS = 3;
        p.endUThreshold = 0.94;
        p.passMaxCrossErrorIn = 8;
        p.motorVoltage = 12;
        p.voltageCompWeight = 0;
    }

    private void start(double endSpeed, double brakeZone) {
        controller.startSegment(straight, 0, 0, 20, 0,
                0.8, Double.NaN, endSpeed, brakeZone, 0.2);
    }

    private SplineController.Output at(double x, double y, double vx, double vy) {
        return controller.update(new SplineController.Input(x, y, 0, vx, vy, 12, 0.02), p);
    }

    @Test
    public void straightSegmentPreservesTangentialAndCrossTrackCommands() {
        start(Double.NaN, 0);
        SplineController.Output output = at(5, 2, 0, 0);
        assertEquals(0.8, output.tangentPower, 1e-12);
        assertEquals(-2, output.crossError, 1e-9);
        assertEquals(-0.2, output.crossPower, 1e-9);
        assertEquals(0.8, output.powerX, 1e-9);
        assertEquals(-0.2, output.powerY, 1e-9);
        assertFalse(output.arrived);
    }

    @Test
    public void midSegmentCrossCorrectionReversesAfterCrossingPath() {
        // Stay well before the endpoint. Crossing the path must reverse correction
        // even when the previous command saturated at full lateral power.
        straight = new SplineGeometry(0, 0, 200, 0,
                200, 0, 200, 0, 240);
        controller.startSegment(straight, 0, 0, 200, 0,
                0.8, Double.NaN, Double.NaN, 0, 0.2);
        SplineController.Output left = at(10, 20, 0, 0);
        assertEquals(-2, left.crossPower, 1e-9);
        assertTrue(left.powerY < 0);
        SplineController.Output right = at(10, -2, 0, 0);
        assertTrue(right.powerY > 0);
        assertTrue(right.closestU < 0.1);
        assertFalse(right.arrived);
    }

    @Test
    public void midSegmentLateralPathCorrectsBothSidesWithoutLatching() {
        SplineGeometry lateral = new SplineGeometry(0, 0, 0, 200,
                0, 200, 0, 200, 240);
        controller.startSegment(lateral, 0, 0, 0, 200,
                0.8, Double.NaN, Double.NaN, 0, 0.2);
        SplineController.Output first = at(20, 10, 0, 0);
        assertTrue(first.powerX < 0);
        assertTrue(first.powerY > 0);
        SplineController.Output crossed = at(-2, 10, 0, 0);
        assertTrue(crossed.powerX > 0);
        assertTrue(crossed.powerY > 0);
        assertTrue(crossed.closestU < 0.1);
        assertFalse(crossed.arrived);
    }

    @Test
    public void twentyFourInchNormalErrorPreservesRawCorrectionRatio() {
        controller.startSegment(new SplineGeometry(0, 0, 200, 0,
                        200, 0, 200, 0, 240), 0, 0, 200, 0,
                1, Double.NaN, Double.NaN, 0, 0.2);
        SplineController.Output output = at(10, -24, 0, 0);
        assertEquals(1, output.tangentPower, 1e-9);
        assertEquals(2.4, output.crossPower, 1e-9);
        assertEquals(1, output.powerX, 1e-9);
        assertEquals(2.4, output.powerY, 1e-9);
    }

    @Test
    public void globalProjectionFindsPositionBeyondOldForwardWindow() {
        controller.startSegment(new SplineGeometry(0, 0, 0, 200,
                        0, 200, 0, 200, 1000), 0, 0, 0, 200,
                1, Double.NaN, Double.NaN, 0, 0.2);
        SplineController.Output output = at(0, 24, 0, 0);
        assertEquals(24, output.pathY, 1e-9);
        assertEquals(24, output.progressS, 1e-9);
        assertEquals(0, output.crossError, 1e-9);
        assertEquals(0, output.powerX, 1e-9);
        assertEquals(1, output.powerY, 1e-9);
        assertEquals(0, output.projectionTangentError, 1e-9);
        assertFalse(output.projectionFault);
    }

    @Test
    public void cubicInflectionNoLongerSticksAtOldWindowBoundary() {
        // r(u) = (100u, 80(u - 0.5)^3). Previously a window ending
        // at u=0.5 hid the normal error for a robot at (60, 0).
        SplineGeometry cubic = new SplineGeometry(0, -10, 100, 60,
                100, 10, 100, 60, 1000);
        controller.startSegment(cubic, 0, 0, 100, 10,
                1, Double.NaN, Double.NaN, 0, 0.2);
        SplineController.Output output = at(60, 0, 0, 0);
        assertEquals(0.6, output.closestU, 1e-4);
        assertEquals(60, output.pathX, 0.01);
        assertEquals(0.08, output.crossError, 0.001);
        assertEquals(0, output.projectionTangentError, 1e-9);
        assertFalse(output.projectionFault);
        assertTrue(output.tangentPower > 0);
        assertFalse(output.arrived);
    }

    @Test
    public void trueEndpointFaultStillStopsAndLatches() {
        start(0, 12);
        SplineController.Output output = at(30, 0, 0, 0);
        assertTrue(output.projectionFault);
        assertEquals(0, output.powerX, 0);
        assertEquals(0, output.powerY, 0);
        assertEquals(0, output.yawPower, 0);
        output = at(10, 0, 0, 0);
        assertTrue(output.projectionFault);
        assertEquals(0, output.powerX, 0);
        start(Double.NaN, 0);
        assertFalse(at(5, 0, 0, 0).projectionFault);
    }

    @Test
    public void validPassThroughIsNotMistakenForProjectionFault() {
        start(Double.NaN, 0);
        at(15, 0, 10, 0);
        SplineController.Output output = at(25, 0, 10, 0);
        assertEquals(-5, output.projectionTangentError, 1e-9);
        assertTrue(output.arrived);
        assertFalse(output.projectionFault);
    }

    @Test
    public void startBoundaryAndLargeStopOvershootCannotCruiseBlindly() {
        start(Double.NaN, 0);
        assertTrue(at(-5, 0, 0, 0).projectionFault);
        start(0, 12);
        at(15, 0, 10, 0);
        SplineController.Output overshot = at(25, 0, 10, 0);
        assertTrue(overshot.projectionFault);
        assertFalse(overshot.arrived);
        assertEquals(0, overshot.powerX, 0);
    }

    @Test
    public void derivativeCanCancelProportionalTermWhileReturningToPath() {
        start(Double.NaN, 0);
        assertEquals(0.36, at(5, -3.6, 0, 0).crossPower, 1e-9);
        // e: 3.6 -> 2 in over 0.02 s. Filtered derivative is
        // 0.25 * (2 - 3.6) / 0.02 = -20 in/s.
        // P + D = 0.10 * 2 + 0.010 * (-20) = 0.
        SplineController.Output returning = at(5, -2, 0, 0);
        assertEquals(2, returning.crossError, 1e-9);
        assertEquals(0, returning.crossPower, 1e-9);
        // With the position now held, that cancellation decays rather than latches.
        assertEquals(0.05, at(5, -2, 0, 0).crossPower, 1e-9);
        assertEquals(0.0875, at(5, -2, 0, 0).crossPower, 1e-9);
    }

    @Test
    public void steadilyGrowingNormalErrorIncreasesCorrectionRatherThanCancellingIt() {
        start(Double.NaN, 0);
        assertEquals(0.05, at(5, -0.5, 0, 0).crossPower, 1e-9);
        assertEquals(0.1625, at(5, -1, 0, 0).crossPower, 1e-9);
        assertEquals(0.259375, at(5, -1.5, 0, 0).crossPower, 1e-9);
        assertEquals(0.34453125, at(5, -2, 0, 0).crossPower, 1e-9);
    }

    @Test
    public void pidHistoryPersistsUntilExplicitReset() {
        start(Double.NaN, 0);
        assertEquals(0.1, at(5, -1, 0, 0).crossPower, 1e-8);
        assertEquals(0.325, at(5, -2, 0, 0).crossPower, 1e-8);
        controller.startSegment(straight, 0, 0, 20, 0,
                0.8, Double.NaN, Double.NaN, 0, 0.2);
        // startSegment must retain the filtered derivative across pass-through segments.
        assertEquals(0.29375, at(5, -2, 0, 0).crossPower, 1e-8);
        controller.resetCrossPid();
        assertEquals(0.2, at(5, -2, 0, 0).crossPower, 1e-8);
    }

    @Test
    public void explicitBrakeUsesOriginalStoppingDistanceBangBangRule() {
        start(0, 12);
        SplineController.Output output = at(15, 0, 20, 0);
        assertTrue(output.brakingZone);
        assertEquals(-0.75, output.tangentPower, 1e-9);
        assertFalse(output.arrived);
        // At the same position, releasing the brake retains the original EMA history.
        output = at(15, 0, 0, 0);
        assertEquals(-0.75 + (0.02 / 0.09) * (0.2 + 0.75),
                output.tangentPower, 1e-9);
    }

    @Test
    public void automaticBrakeUsesFivePercentMarginAndStaysInZone() {
        start(0, Double.NaN);
        // At 20 in/s and 30 in/s² the distance is 6.667 in; with margin it is 7 in.
        SplineController.Output outside = at(12.9, 0, 20, 0);
        assertFalse(outside.brakingZone);
        assertFalse(outside.brakeReverseRequested);
        assertEquals(7, outside.brakeTriggerDistance, 1e-9);
        assertEquals(0.8, outside.tangentPower, 1e-9);
        SplineController.Output inside = at(13.1, 0, 20, 0);
        assertTrue(inside.brakingZone);
        assertTrue(inside.brakeReverseRequested);
        assertEquals(-0.75, inside.tangentPower, 1e-9);
        // Once braking slows the robot, retain the zone and its filter history.
        SplineController.Output slower = at(13.1, 0, 10, 0);
        assertTrue(slower.brakingZone);
        assertFalse(slower.brakeReverseRequested);
        assertEquals(-0.75 + (0.02 / 0.09) * (0.2 + 0.75),
                slower.tangentPower, 1e-9);
    }

    @Test
    public void faultFrameRetainsTheProjectionJumpAndSuppressedRequest() {
        SplineGeometry hooked = new SplineGeometry(46.15, 45.67, -4.76, -19.41,
                -7.61, 50.83, -2.39, -15.47, 1000);
        controller.startSegment(hooked, -179.97, -180, -7.61, 50.83,
                1, Double.NaN, 0, Double.NaN, 0.30);
        SplineController.Output before = controller.update(
                new SplineController.Input(-4.30, 46.30, 180, -8, 0, 12, 0.02), p);
        assertFalse(before.projectionFault);
        assertEquals(0.85, before.closestU, 0.02);
        assertTrue(before.remainingArc > 3);

        SplineController.Output fault = controller.update(
                new SplineController.Input(-4.65, 46.30, 180, -8, 0, 12, 0.02), p);
        assertTrue(fault.projectionFault);
        assertEquals(before.closestU, fault.previousU, 0);
        assertEquals(1, fault.closestU, 0);
        assertEquals(0, fault.remainingArc, 1e-9);
        assertEquals(-4, fault.projectionTangentError, 0.3);
        assertEquals(-0.75, fault.candidateTangentPower, 1e-9);
        assertEquals(0, fault.powerX, 0);
        assertEquals(0, fault.powerY, 0);
        assertFalse(fault.arrived);
    }

    @Test
    public void automaticBrakeUsesEndSpeedAndResetsForNextSegment() {
        start(10, Double.NaN);
        // (20² - 10²) / (2*30) * 1.05 = 5.25 in.
        assertFalse(at(14.7, 0, 20, 0).brakingZone);
        assertTrue(at(14.8, 0, 20, 0).brakingZone);
        start(10, Double.NaN);
        SplineController.Output reverse = at(5, 0, -20, 0);
        assertFalse(reverse.brakingZone);
        assertEquals(0.8, reverse.tangentPower, 1e-9);
        start(Double.NaN, Double.NaN);
        assertFalse(at(15, 0, 20, 0).brakingZone);
    }

    @Test
    public void stopPointRequiresBothPositionAndSpeed() {
        start(0, 12);
        assertFalse(at(20, 0, 4, 0).arrived);
        assertTrue(at(20, 0, 0, 0).arrived);
    }

    @Test
    public void ordinaryWaypointPassesThroughOnEndpointNormal() {
        start(Double.NaN, 0);
        at(10, 0, 10, 0);
        SplineController.Output output = at(21, 0, 10, 0);
        assertTrue(output.arrived);
    }

    @Test(timeout = 1000)
    public void hugeHeadingsNormalizeInBoundedTime() {
        assertEquals(2, SplineController.angleDiff(360000000002.0, 0), 0);
        assertTrue(Double.isFinite(SplineController.angleDiff(Double.MAX_VALUE, -Double.MAX_VALUE)));
        assertEquals(180, SplineController.angleDiff(-180, 0), 0);
    }

    @Test(expected = IllegalArgumentException.class, timeout = 1000)
    public void infiniteHeadingIsRejectedRatherThanHanging() {
        SplineController.angleDiff(Double.POSITIVE_INFINITY, 0);
    }

    @Test(expected = IllegalArgumentException.class)
    public void invalidMeasurementIsRejected() {
        start(Double.NaN, 0);
        at(5, Double.NaN, 0, 0);
    }

    @Test
    public void headingUsesShortestAngleAndOutputIsVoltageClipped() {
        assertEquals(2, SplineController.angleDiff(-179, 179), 1e-12);
        assertEquals(-2, SplineController.angleDiff(179, -179), 1e-12);
        assertEquals(1, SplineController.headingPowerFromError(100, 0.02, 0), 0);
        start(Double.NaN, 0);
        p.voltageCompWeight = 1;
        SplineController.Output output = controller.update(
                new SplineController.Input(5, 0, -90, 0, 0, 6, 0.02), p);
        assertEquals(1.6, output.powerX, 1e-9);
        assertEquals(2, output.yawPower, 1e-9);
    }

    @Test
    public void headingInterpolatesByGeometricProgress() {
        controller.startSegment(straight, 170, 190, 20, 0,
                0.8, Double.NaN, Double.NaN, 0, 0.2);
        SplineController.Output output = controller.update(
                new SplineController.Input(10, 0, 170, 0, 0, 12, 0.02), p);
        assertEquals(180, output.targetHeading, 1e-7);
        assertEquals(0.2, output.yawPower, 1e-9);
    }

    @Test
    public void stopCaptureTracksHookEndpointOnEveryTick() {
        double endX = -7.61, endY = 50.83;
        SplineGeometry hooked = new SplineGeometry(46.15, 45.67, -4.76, -19.41,
                endX, endY, -2.39, -15.47, 1000);
        controller.startSegment(hooked, -179.97, -180, endX, endY,
                1, Double.NaN, 0, Double.NaN, 0.30);
        // Approach the hook fast enough to brake, but not to enter capture yet.
        for (int i = 1; i <= 8; i++) {
            double u = i / 10.0;
            SplineController.Output approach = controller.update(new SplineController.Input(
                    hooked.xAt(u), hooked.yAt(u), 180, -23, 0, 12, 0.02), p);
            assertFalse(approach.stopCapture);
        }
        // At heading 180 the endpoint is ahead, slightly to the LEFT. The nearby
        // path is to the right; its normal correction must not steer capture.
        for (int i = 0; i <= 250; i++) {
            double x = -5.30 - 0.002 * i, y = 50.90 + 0.0014 * i;
            SplineController.Output out = controller.update(new SplineController.Input(
                    x, y, 180, -0.10, 0.07, 12, 0.02), p);
            assertTrue(out.brakingZone);
            assertTrue(out.stopCapture);
            assertFalse(out.arrived);
            double dx = endX - x, dy = endY - y, distance = Math.hypot(dx, dy);
            double magnitude = Math.min(p.stopCaptureKp * distance, p.stopCaptureMaxPower);
            assertEquals(magnitude * dx / distance, out.powerX, 1e-9);
            assertEquals(magnitude * dy / distance, out.powerY, 1e-9);
        }
    }

    @Test
    public void stopCaptureCapsCombinedTranslationEvenWithVoltageCompensation() {
        p.voltageCompWeight = 1;
        // Check both the capture limit and a lower per-segment limit.
        for (double segmentLimit : new double[]{0.8, 0.10, 0}) {
            controller.startSegment(straight, 0, 0, 20, 0,
                    segmentLimit, Double.NaN, 0, 12, 0.2);
            SplineController.Output out = controller.update(new SplineController.Input(
                    18.4, 1.2, -10, 0, 0, 9, 0.02), p);
            assertTrue(out.stopCapture);
            double limit = Math.min(p.stopCaptureMaxPower, segmentLimit);
            assertEquals(0.8 * limit, out.powerX, 1e-9);
            assertEquals(-0.6 * limit, out.powerY, 1e-9);
            assertEquals(0.2 * (12.0 / 9.0), out.yawPower, 1e-9);
        }
    }

    @Test
    public void stopCaptureReturnsFromOvershootAndCanTurnInPlaceAtEndpoint() {
        start(0, 12);
        at(16, 0, 8, 0); // Bring the projection within reach of the end.
        SplineController.Output overshot = at(21.4, 0.8, 0, 0);
        assertTrue(overshot.stopCapture);
        assertFalse(overshot.arrived);
        assertEquals(-1.4 * p.stopCaptureKp, overshot.powerX, 1e-9);
        assertEquals(-0.8 * p.stopCaptureKp, overshot.powerY, 1e-9);
        SplineController.Output turning = controller.update(new SplineController.Input(
                20, 0, -10, 0, 0, 12, 0.02), p);
        assertEquals(0, turning.powerX, 0);
        assertEquals(0, turning.powerY, 0);
        assertTrue(turning.yawPower > 0);
        assertFalse(turning.arrived);
        assertTrue(at(20, 0, 0, 0).arrived);
    }

    @Test
    public void speedLimitAndStopCaptureStillOverrideCruisePower() {
        controller.startSegment(straight, 0, 0, 20, 0,
                0.8, 10, Double.NaN, 0, 0.2);
        assertEquals(-0.35, at(5, 0, 20, 0).tangentPower, 1e-9);

        start(0, 12);
        SplineController.Output nearEnd = at(19, 0, 0, 0);
        assertTrue(nearEnd.stopCapture);
        assertEquals(0.12, nearEnd.tangentPower, 1e-9);
    }
}
