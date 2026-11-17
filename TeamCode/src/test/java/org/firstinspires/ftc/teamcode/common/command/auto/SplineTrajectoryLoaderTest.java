package org.firstinspires.ftc.teamcode.common.command.auto;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Constructor;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Queue;

public class SplineTrajectoryLoaderTest {

    private boolean oldHonorDelay;
    private boolean oldWarnDuration;

    @Before
    public void setUp() {
        oldHonorDelay = SplineTrajectoryLoader.HONOR_DELAY_AFTER_ARRIVE;
        oldWarnDuration = SplineTrajectoryLoader.WARN_UNUSED_DURATION;
        SplineTrajectoryLoader.HONOR_DELAY_AFTER_ARRIVE = false;
        SplineTrajectoryLoader.WARN_UNUSED_DURATION = false;
    }

    @After
    public void tearDown() {
        SplineTrajectoryLoader.HONOR_DELAY_AFTER_ARRIVE = oldHonorDelay;
        SplineTrajectoryLoader.WARN_UNUSED_DURATION = oldWarnDuration;
    }

    @Test
    public void emptyJsonReturnsEmptyWithoutTouchingDriver() {
        FakeMotionDriver driver = new FakeMotionDriver();
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(driver);

        SplineTrajectoryLoader.ExecutionResult result = loader.start("[]");

        assertEquals(SplineTrajectoryLoader.ExecutionStatus.EMPTY, result.status);
        assertFalse(loader.isRunning());
        assertEquals(0, driver.totalCalls());
    }

    @Test
    public void malformedJsonReturnsParseErrorWithoutTouchingDriver() {
        FakeMotionDriver driver = new FakeMotionDriver();
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(driver);

        SplineTrajectoryLoader.ExecutionResult result = loader.start("[{bad json]");

        assertEquals(SplineTrajectoryLoader.ExecutionStatus.PARSE_ERROR, result.status);
        assertFalse(loader.isRunning());
        assertEquals(0, driver.totalCalls());
    }

    @Test
    public void startOnlyArmsStateMachineAndDoesNotMove() {
        FakeMotionDriver driver = new FakeMotionDriver();
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(driver);

        SplineTrajectoryLoader.ExecutionResult result = loader.start(twoMovePath());

        assertEquals(SplineTrajectoryLoader.ExecutionStatus.RUNNING, result.status);
        assertTrue(loader.isRunning());
        assertEquals(0, driver.totalCalls());
        assertEquals(0, loader.getStepIndex());
    }

    @Test
    public void versionTwoEnvelopeArmsTheSameMovementPlan() {
        FakeMotionDriver driver = new FakeMotionDriver();
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(driver);
        assertEquals(SplineTrajectoryLoader.ExecutionStatus.RUNNING,
                loader.start("{\"version\":2,\"frames\":" + twoMovePath() + "}").status);
        assertEquals(0, driver.totalCalls());
        loader.update();
        loader.update();
        assertEquals(1, driver.beginCalls);
        assertEquals(1, driver.startMoveCalls);
    }

    @Test
    public void versionTwoEnvelopeRetainsOrderedWaypointCommands() {
        FakeMotionDriver driver = new FakeMotionDriver();
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(driver);
        loader.start("{\"version\":2,\"frames\":[{\"x\":0,\"y\":0,\"commands\":["
                + "{\"name\":\"auto.waitMillis\",\"params\":[\"500\"]},"
                + "{\"name\":\"intaker.intakerArmStop\",\"params\":[]}]}]}");
        loader.update();
        SplineTrajectoryLoader.TrajectoryEvent event = loader.pollEvent();
        assertNotNull(event);
        assertEquals(2, event.commands.size());
        assertEquals("auto.waitMillis", event.commands.get(0).name);
        assertEquals(Collections.singletonList("500"), event.commands.get(0).params);
        assertEquals("intaker.intakerArmStop", event.commands.get(1).name);
        loader.update();
        assertEquals(SplineTrajectoryLoader.ExecutionStatus.COMPLETED, loader.getStatus());
    }

    @Test
    public void unknownAndMalformedVersionsFailBeforeTouchingMotion() {
        String[] invalid = {
                "{\"version\":1,\"frames\":[]}", "{\"version\":3,\"frames\":[]}",
                "{\"version\":\"2\",\"frames\":[]}", "{\"version\":2,\"frames\":{}}",
                "{\"frames\":[]}", "{\"version\":2}", "[] []"
        };
        for (String json : invalid) {
            FakeMotionDriver driver = new FakeMotionDriver();
            SplineTrajectoryLoader loader = new SplineTrajectoryLoader(driver);
            assertEquals(json, SplineTrajectoryLoader.ExecutionStatus.PARSE_ERROR,
                    loader.start(json).status);
            assertEquals(0, driver.totalCalls());
        }
    }

    @Test
    public void versionTwoEmptyFramesRemainEmptyWithoutMotion() {
        FakeMotionDriver driver = new FakeMotionDriver();
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(driver);
        assertEquals(SplineTrajectoryLoader.ExecutionStatus.EMPTY,
                loader.start("{\"version\":2,\"frames\":[]}").status);
        assertEquals(0, driver.totalCalls());
    }

    @Test
    public void firstMoveTickOnlyInitializesPath() {
        FakeMotionDriver driver = new FakeMotionDriver();
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(driver);
        loader.start(twoMovePath());

        loader.update();

        assertEquals(1, driver.beginCalls);
        assertEquals(0, driver.startMoveCalls);
        assertEquals(0, driver.updateCalls);
        assertEquals(1, loader.getStepIndex());
        assertTrue(loader.isRunning());
        assertNotNull(driver.beginPose);
        assertEquals(0.0, driver.beginPoint.x, 0.0);
        assertEquals(0.0, driver.beginPoint.y, 0.0);
    }

    @Test
    public void movementUsesAtMostOneDriverUpdatePerLoaderTick() {
        FakeMotionDriver driver = new FakeMotionDriver();
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(driver);
        loader.start(twoMovePath());

        loader.update(); // begin first point
        loader.update(); // start second move

        assertEquals(1, driver.startMoveCalls);
        assertEquals(0, driver.updateCalls);

        for (int i = 1; i <= 8; i++) {
            int before = driver.updateCalls;
            loader.update();
            assertEquals(before + 1, driver.updateCalls);
            assertEquals(i, driver.updateCalls);
            assertTrue(loader.isRunning());
        }
    }

    @Test
    public void arrivedMoveCompletesTrajectory() throws Exception {
        FakeMotionDriver driver = new FakeMotionDriver();
        driver.results.add(segment(SplineTracker.SegmentStatus.ARRIVED));
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(driver);
        loader.start(twoMovePath());

        loader.update(); // begin
        loader.update(); // start move
        SplineTrajectoryLoader.ExecutionResult result = loader.update(); // arrived

        assertEquals(SplineTrajectoryLoader.ExecutionStatus.COMPLETED, result.status);
        assertTrue(loader.isFinished());
        assertFalse(loader.isRunning());
        assertEquals(1, driver.updateCalls);
        assertEquals(1, driver.cancelCalls); // route end stops the drivetrain once
    }

    @Test
    public void stopPolicyTurnsStallIntoTerminalFailureAndCancelsDriver() throws Exception {
        FakeMotionDriver driver = new FakeMotionDriver();
        driver.results.add(segment(SplineTracker.SegmentStatus.STALLED));
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(driver);
        loader.setFailurePolicy(SplineTrajectoryLoader.FailurePolicy.STOP_PATH);
        loader.start(twoMovePath());

        loader.update();
        loader.update();
        SplineTrajectoryLoader.ExecutionResult result = loader.update();

        assertEquals(SplineTrajectoryLoader.ExecutionStatus.TRACKER_FAILURE, result.status);
        assertEquals(SplineTracker.SegmentStatus.STALLED, result.segmentResult.status);
        assertEquals(1, driver.cancelCalls);
        assertFalse(loader.isRunning());
    }

    @Test
    public void continuePolicySkipsFailedMoveAndAdvancesToNextMove() throws Exception {
        FakeMotionDriver driver = new FakeMotionDriver();
        driver.results.add(segment(SplineTracker.SegmentStatus.STALLED));
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(driver);
        loader.setFailurePolicy(SplineTrajectoryLoader.FailurePolicy.CONTINUE);
        loader.start(threeMovePath());

        loader.update(); // begin point 0
        loader.update(); // start move to point 1
        loader.update(); // point 1 stalled -> skip

        assertTrue(loader.isRunning());
        assertEquals(2, loader.getStepIndex());

        loader.update(); // start move to point 2
        assertEquals(2, driver.startMoveCalls);
        assertEquals(2.0, driver.lastStartedPoint.x, 0.0);
        assertEquals(1, driver.cancelCalls); // failed move must be stopped before continuing
    }

    @Test
    public void intermediateWaypointDoesNotStopMotor() throws Exception {
        FakeMotionDriver driver = new FakeMotionDriver();
        driver.results.add(segment(SplineTracker.SegmentStatus.ARRIVED));
        driver.results.add(segment(SplineTracker.SegmentStatus.ARRIVED));
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(driver);
        loader.start(threeMovePath());
        loader.update(); // begin
        loader.update(); // move 1 start
        loader.update(); // move 1 arrives
        assertEquals(0, driver.cancelCalls);
        loader.update(); // move 2 starts without a stop
        assertEquals(0, driver.cancelCalls);
        loader.update(); // route completes
        assertEquals(1, driver.cancelCalls);
    }

    @Test
    public void waitAfterMoveStopsExactlyOnceBeforeWaiting() throws Exception {
        FakeMotionDriver driver = new FakeMotionDriver();
        driver.results.add(segment(SplineTracker.SegmentStatus.ARRIVED));
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(driver);
        loader.start("[{\"x\":0,\"y\":0},{\"x\":5,\"y\":0},{\"wait\":0}]");
        loader.update();
        loader.update();
        loader.update(); // move arrives; next step is wait
        assertEquals(1, driver.cancelCalls);
        loader.update(); // wait starts
        loader.update(); // wait ends
        assertEquals(1, driver.cancelCalls);
    }

    @Test
    public void brakingFieldsAreForwardedAndInvalidZonesRejected() throws Exception {
        FakeMotionDriver driver = new FakeMotionDriver();
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(driver);
        String path = "[{\"x\":0,\"y\":0},"
                + "{\"x\":20,\"y\":0,\"maxPower\":0.7,\"maxSpeed\":24,"
                + "\"endSpeed\":0,\"brakeZoneIn\":12,\"brakeForwardPower\":0.2}]";
        assertEquals(SplineTrajectoryLoader.ExecutionStatus.RUNNING, loader.start(path).status);
        loader.update();
        loader.update();
        assertEquals(0.7, driver.lastStartedPoint.maxPower, 0.0);
        assertEquals(24, driver.lastStartedPoint.maxSpeed, 0.0);
        assertEquals(0, driver.lastStartedPoint.endSpeed, 0.0);
        assertEquals(12, driver.lastStartedPoint.brakeZoneIn, 0.0);
        assertEquals(0.2, driver.lastStartedPoint.brakeForwardPower, 0.0);

        FakeMotionDriver automaticDriver = new FakeMotionDriver();
        SplineTrajectoryLoader automatic = new SplineTrajectoryLoader(automaticDriver);
        assertEquals(SplineTrajectoryLoader.ExecutionStatus.RUNNING,
                automatic.start("[{\"x\":0,\"y\":0},{\"x\":20,\"y\":0,\"endSpeed\":0}]").status);
        automatic.update();
        automatic.update();
        assertTrue(Double.isNaN(automaticDriver.lastStartedPoint.brakeZoneIn));

        SplineTrajectoryLoader invalid = new SplineTrajectoryLoader(new FakeMotionDriver());
        assertEquals(SplineTrajectoryLoader.ExecutionStatus.PARSE_ERROR,
                invalid.start("[{\"x\":0,\"y\":0,\"endSpeed\":0,\"brakeZoneIn\":0}]").status);
        assertEquals(SplineTrajectoryLoader.ExecutionStatus.PARSE_ERROR,
                invalid.start("[{\"x\":0,\"y\":0,\"endSpeed\":0,\"brakeZoneIn\":-1}]").status);
    }

    @Test
    public void longWaitRemainsRunningWithoutCallingMotionDriver() {
        FakeMotionDriver driver = new FakeMotionDriver();
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(driver);
        loader.start("[{\"wait\":3600}]");

        loader.update(); // initialize wait timestamp
        for (int i = 0; i < 100; i++) {
            loader.update();
        }

        assertTrue(loader.isRunning());
        assertEquals(0, driver.totalCalls());
        assertEquals(0, loader.getStepIndex());
    }

    @Test
    public void zeroWaitCompletesAcrossTicksWithoutBlockingDriver() {
        FakeMotionDriver driver = new FakeMotionDriver();
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(driver);
        loader.start("[{\"wait\":0}]");

        assertEquals(SplineTrajectoryLoader.ExecutionStatus.RUNNING, loader.update().status);
        assertEquals(SplineTrajectoryLoader.ExecutionStatus.COMPLETED, loader.update().status);
        assertEquals(0, driver.totalCalls());
    }

    @Test
    public void waypointEventBlocksFurtherProgressUntilConsumed() {
        FakeMotionDriver driver = new FakeMotionDriver();
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(driver);
        loader.start("["
                + "{\"x\":0,\"y\":0,\"heading\":0,\"marker\":\"ready\"},"
                + "{\"wait\":0}"
                + "]");

        loader.update(); // first point -> event, advance logical step index
        assertEquals(1, driver.beginCalls);
        assertEquals(1, loader.getStepIndex());

        loader.update();
        loader.update();
        assertEquals(1, loader.getStepIndex()); // wait must not start while event is pending

        SplineTrajectoryLoader.TrajectoryEvent event = loader.pollEvent();
        assertNotNull(event);
        assertEquals("ready", event.marker);

        loader.update(); // now wait starts
        assertEquals(1, loader.getStepIndex());
        loader.update(); // zero wait finishes
        assertEquals(SplineTrajectoryLoader.ExecutionStatus.COMPLETED, loader.getStatus());
    }

    @Test
    public void eventPreservesMarkerCommandAndParametersAsData() {
        FakeMotionDriver driver = new FakeMotionDriver();
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(driver);
        loader.start("[{"
                + "\"x\":1,\"y\":2,\"heading\":30,"
                + "\"marker\":\"shoot\","
                + "\"command\":\"setMode\","
                + "\"commandParams\":[\"fast\",\"3\"]"
                + "}]");

        loader.update();
        SplineTrajectoryLoader.TrajectoryEvent event = loader.pollEvent();

        assertNotNull(event);
        assertEquals(0, event.frameIndex);
        assertEquals("shoot", event.marker);
        assertEquals("setMode", event.command);
        assertEquals(Arrays.asList("fast", "3"), event.commandParams);
        assertTrue(event.hasCommand());
        assertEquals(1, event.commands.size());
        assertEquals("setMode", event.commands.get(0).name);
        assertEquals(Arrays.asList("fast", "3"), event.commands.get(0).params);
        assertNull(loader.pollEvent());

        loader.update();
        assertEquals(SplineTrajectoryLoader.ExecutionStatus.COMPLETED, loader.getStatus());
    }

    @Test
    public void cancelClearsRunAndCancelsMotionExactlyOnce() {
        FakeMotionDriver driver = new FakeMotionDriver();
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(driver);
        loader.start(twoMovePath());
        loader.update();
        loader.update();

        loader.cancel();

        assertEquals(SplineTrajectoryLoader.ExecutionStatus.CANCELLED, loader.getStatus());
        assertFalse(loader.isRunning());
        assertEquals(1, driver.cancelCalls);

        loader.cancel();
        assertEquals(1, driver.cancelCalls);
    }

    @Test
    public void startingNewTrajectoryCancelsPreviousRunAndResetsState() {
        FakeMotionDriver driver = new FakeMotionDriver();
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(driver);
        loader.start(twoMovePath());
        loader.update(); // begin path; does not start the motors

        SplineTrajectoryLoader.ExecutionResult replacement =
                loader.start("[{\"wait\":0}]");

        assertEquals(0, driver.cancelCalls); // no motor output needs cancellation
        assertEquals(SplineTrajectoryLoader.ExecutionStatus.RUNNING, replacement.status);
        assertEquals(0, loader.getStepIndex());
        assertNull(loader.pollEvent());
    }

    @Test
    public void replacingActiveMoveStopsItExactlyOnce() {
        FakeMotionDriver driver = new FakeMotionDriver();
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(driver);
        loader.start(twoMovePath());
        loader.update(); // begin
        loader.update(); // actively moving
        loader.start("[{\"wait\":0}]");
        assertEquals(1, driver.cancelCalls);
    }

    @Test
    public void driverResultObjectIsPropagatedWithoutReplacement() throws Exception {
        FakeMotionDriver driver = new FakeMotionDriver();
        SplineTracker.SegmentResult arrived = segment(SplineTracker.SegmentStatus.ARRIVED);
        driver.results.add(arrived);
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(driver);
        loader.start(twoMovePath());

        loader.update();
        loader.update();
        SplineTrajectoryLoader.ExecutionResult result = loader.update();

        assertSame(arrived, result.segmentResult);
    }

    @Test
    public void commandsPreserveOrderAndExposeImmutableData() {
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(new FakeMotionDriver());
        assertEquals(SplineTrajectoryLoader.ExecutionStatus.RUNNING, loader.start(
                "[{\"x\":0,\"y\":0,\"commands\":["
                        + "{\"name\":\"auto.prepare\",\"params\":[\"fast\",\"3\"]},"
                        + "{\"name\":\"auto.shoot\"}]}]").status);
        loader.update();
        SplineTrajectoryLoader.TrajectoryEvent event = loader.pollEvent();
        assertNotNull(event);
        assertTrue(event.hasCommand());
        assertNull(event.command);
        assertTrue(event.commandParams.isEmpty());
        assertEquals(2, event.commands.size());
        assertEquals("auto.prepare", event.commands.get(0).name);
        assertEquals(Arrays.asList("fast", "3"), event.commands.get(0).params);
        assertEquals("auto.shoot", event.commands.get(1).name);
        assertTrue(event.commands.get(1).params.isEmpty());
        try {
            event.commands.clear();
            fail("event commands must be immutable");
        } catch (UnsupportedOperationException expected) {
            // Expected.
        }
        try {
            event.commands.get(0).params.add("extra");
            fail("command parameters must be immutable");
        } catch (UnsupportedOperationException expected) {
            // Expected.
        }
        loader.update();
        assertEquals(SplineTrajectoryLoader.ExecutionStatus.COMPLETED, loader.getStatus());
    }

    @Test
    public void trajectoryCommandSnapshotsCallerParameters() {
        List<String> params = new ArrayList<>(Collections.singletonList("original"));
        SplineTrajectoryLoader.TrajectoryCommand command =
                new SplineTrajectoryLoader.TrajectoryCommand("auto.example", params);
        params.set(0, "changed");
        assertEquals(Collections.singletonList("original"), command.params);
    }

    @Test
    public void emptyCommandsAllowLegacyCommandAndEmptyLegacyAllowsCommands() {
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(new FakeMotionDriver());
        loader.start("[{\"x\":0,\"y\":0,\"command\":\"auto.old\","
                + "\"commandParams\":[\"1\"],\"commands\":[]}]");
        loader.update();
        SplineTrajectoryLoader.TrajectoryEvent legacy = loader.pollEvent();
        assertEquals("auto.old", legacy.command);
        assertEquals("auto.old", legacy.commands.get(0).name);
        assertEquals(Collections.singletonList("1"), legacy.commands.get(0).params);

        assertEquals(SplineTrajectoryLoader.ExecutionStatus.RUNNING, loader.start(
                "[{\"x\":0,\"y\":0,\"command\":\"  \","
                        + "\"commands\":[{\"name\":\"auto.new\",\"params\":[]}]}]").status);
        loader.update();
        assertEquals("auto.new", loader.pollEvent().commands.get(0).name);
    }

    @Test
    public void emptyCommandsDoNotCreateAnEvent() {
        FakeMotionDriver driver = new FakeMotionDriver();
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(driver);
        loader.start("[{\"x\":0,\"y\":0,\"commands\":[]}]");
        loader.update();
        assertNull(loader.pollEvent());
        assertEquals(SplineTrajectoryLoader.ExecutionStatus.COMPLETED, loader.getStatus());
    }

    @Test
    public void simultaneousNonEmptyOldAndNewCommandsAreParseError() {
        assertCommandParseError("\"command\":\"auto.old\","
                + "\"commands\":[{\"name\":\"auto.new\"}]");
    }

    @Test
    public void malformedCommandsAreStrictParseErrorsBeforeAnyMotion() {
        String[] malformed = {
                "null", "{}", "\"auto.shoot\"", "1", "true",
                "[null]", "[1]", "[\"auto.shoot\"]", "[[]]", "[{}]",
                "[{\"name\":null}]", "[{\"name\":3}]", "[{\"name\":true}]",
                "[{\"name\":{}}]", "[{\"name\":[]}]", "[{\"name\":\"\"}]",
                "[{\"name\":\"  \"}]", "[{\"name\":\"auto.shoot\",\"params\":null}]",
                "[{\"name\":\"auto.shoot\",\"params\":{}}]",
                "[{\"name\":\"auto.shoot\",\"params\":\"1\"}]",
                "[{\"name\":\"auto.shoot\",\"params\":[null]}]",
                "[{\"name\":\"auto.shoot\",\"params\":[1]}]",
                "[{\"name\":\"auto.shoot\",\"params\":[true]}]",
                "[{\"name\":\"auto.shoot\",\"params\":[{}]}]",
                "[{\"name\":\"auto.shoot\",\"params\":[[]]}]",
                "[{\"name\":\"auto.valid\"},{\"name\":\"auto.bad\",\"params\":[1]}]"
        };
        for (String commands : malformed) {
            assertCommandParseError("\"commands\":" + commands);
        }
    }

    @Test
    public void versionedWrapperIsNotAccepted() {
        FakeMotionDriver driver = new FakeMotionDriver();
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(driver);
        assertEquals(SplineTrajectoryLoader.ExecutionStatus.PARSE_ERROR, loader.start(
                "{\"schemaVersion\":1,\"waypoints\":[{\"x\":0,\"y\":0}]}").status);
        assertEquals(0, driver.totalCalls());
    }

    @Test
    public void commandsOnIntermediateMoveOnlyEmitEventAndDoNotStopPath() throws Exception {
        FakeMotionDriver driver = new FakeMotionDriver();
        driver.results.add(segment(SplineTracker.SegmentStatus.ARRIVED));
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(driver);
        loader.start("[{\"x\":0,\"y\":0},{\"x\":1,\"y\":0,"
                + "\"commands\":[{\"name\":\"auto.first\"},{\"name\":\"auto.second\"}]},"
                + "{\"x\":2,\"y\":0}]");
        loader.update(); // begin
        assertNull(loader.pollEvent());
        loader.update(); // start move
        assertNull(loader.pollEvent());
        loader.update(); // arrival emits the entire waypoint
        assertEquals(0, driver.cancelCalls);
        SplineTrajectoryLoader.TrajectoryEvent event = loader.pollEvent();
        assertEquals(1, event.frameIndex);
        assertEquals(2, event.commands.size());
        loader.update(); // no command-completion handshake is required
        assertEquals(2, driver.startMoveCalls);
        assertEquals(0, driver.cancelCalls);
    }

    @Test
    public void waitFrameCommandsKeepOriginalCompletionTrigger() {
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(new FakeMotionDriver());
        loader.start("[{\"wait\":0,\"commands\":[{\"name\":\"auto.afterWait\"}]}]");
        loader.update();
        assertNull(loader.pollEvent());
        loader.update();
        assertEquals("auto.afterWait", loader.pollEvent().commands.get(0).name);
    }

    private static void assertCommandParseError(String fields) {
        FakeMotionDriver driver = new FakeMotionDriver();
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(driver);
        SplineTrajectoryLoader.ExecutionResult result = loader.start(
                "[{\"x\":0,\"y\":0},{\"x\":1,\"y\":0," + fields + "}]");
        assertEquals(fields, SplineTrajectoryLoader.ExecutionStatus.PARSE_ERROR, result.status);
        assertEquals(1, result.frameIndex);
        assertFalse(loader.isRunning());
        assertEquals(0, driver.totalCalls());
    }

    private static String twoMovePath() {
        return "["
                + "{\"x\":0,\"y\":0,\"dx\":1,\"dy\":0,\"heading\":0},"
                + "{\"x\":1,\"y\":0,\"dx\":1,\"dy\":0,\"heading\":0}"
                + "]";
    }

    private static String threeMovePath() {
        return "["
                + "{\"x\":0,\"y\":0,\"heading\":0},"
                + "{\"x\":1,\"y\":0,\"heading\":0},"
                + "{\"x\":2,\"y\":0,\"heading\":0}"
                + "]";
    }

    private static SplineTracker.SegmentResult segment(
            SplineTracker.SegmentStatus status) throws Exception {
        Constructor<SplineTracker.SegmentResult> constructor =
                SplineTracker.SegmentResult.class.getDeclaredConstructor(
                        SplineTracker.SegmentStatus.class,
                        double.class,
                        double.class);
        constructor.setAccessible(true);
        return constructor.newInstance(status, 0.25, 0.5);
    }

    private static final class FakeMotionDriver implements SplineTrajectoryLoader.MotionDriver {
        int beginCalls;
        int startMoveCalls;
        int updateCalls;
        int cancelCalls;
        Pose2D beginPose;
        SplineTracker.PathPoint beginPoint;
        SplineTracker.PathPoint lastStartedPoint;
        SplineTracker.SegmentResult lastResult;
        final Queue<SplineTracker.SegmentResult> results = new ArrayDeque<>();

        @Override
        public void begin(Pose2D pose, SplineTracker.PathPoint start) {
            beginCalls++;
            beginPose = pose;
            beginPoint = start;
        }

        @Override
        public void startMove(SplineTracker.PathPoint target) {
            startMoveCalls++;
            lastStartedPoint = target;
        }

        @Override
        public SplineTracker.SegmentResult update() {
            updateCalls++;
            if (!results.isEmpty()) lastResult = results.remove();
            return lastResult;
        }

        @Override
        public void cancel() {
            cancelCalls++;
        }

        @Override
        public SplineTracker.SegmentResult getLastResult() {
            return lastResult;
        }

        int totalCalls() {
            return beginCalls + startMoveCalls + updateCalls + cancelCalls;
        }
    }
}
