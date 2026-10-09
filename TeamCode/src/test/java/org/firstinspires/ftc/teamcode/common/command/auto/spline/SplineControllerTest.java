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
        p.projectionSamples = 14;
        p.newtonIterations = 4;
        p.projectionBacktrackIn = 3;
        p.projectionForwardIn = 18;
        p.tangentSampleDu = 0.01;
        p.crossP = 0.10;
        p.crossI = 0;
        p.crossD = 0.010;
        p.crossIMaxPower = 0.20;
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
        p.maxTranslationPower = 1;
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

    @Test
    public void headingUsesShortestAngleAndOutputIsVoltageClipped() {
        assertEquals(2, SplineController.angleDiff(-179, 179), 1e-12);
        assertEquals(-2, SplineController.angleDiff(179, -179), 1e-12);
        assertEquals(1, SplineController.headingPowerFromError(100, 0.02, 0), 0);
        start(Double.NaN, 0);
        p.voltageCompWeight = 1;
        SplineController.Output output = controller.update(
                new SplineController.Input(5, 0, -90, 0, 0, 6, 0.02), p);
        assertEquals(1, output.powerX, 1e-9);
        assertEquals(1, output.yawPower, 1e-9);
    }

    @Test
    public void headingInterpolatesByGeometricProgress() {
        controller.startSegment(straight, 170, 190, 20, 0,
                0.8, Double.NaN, Double.NaN, 0, 0.2);
        SplineController.Output output = controller.update(
                new SplineController.Input(10, 0, 170, 0, 0, 12, 0.02), p);
        assertEquals(180, output.targetHeading, 1e-9);
        assertEquals(0.2, output.yawPower, 1e-9);
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
