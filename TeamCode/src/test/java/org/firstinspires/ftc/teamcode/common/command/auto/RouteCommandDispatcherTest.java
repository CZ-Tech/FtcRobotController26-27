package org.firstinspires.ftc.teamcode.common.command.auto;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.common.network.CommandCatalog;
import org.junit.Test;

import java.lang.reflect.Constructor;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class RouteCommandDispatcherTest {
    private static final String INTaker_IN = "intaker.intakerIn";
    private static final String SHOOTER_STOP = "shooter.stopShooter";

    private final RecordingSink sink = new RecordingSink();
    private final CommandCatalog catalog = catalog(
            descriptor(INTaker_IN, Arrays.asList("power"), Arrays.asList("double"),
                    Collections.singletonList(Collections.singletonList("double"))),
            descriptor(SHOOTER_STOP, Collections.<String>emptyList(),
                    Collections.<String>emptyList(),
                    Collections.singletonList(Collections.<String>emptyList())));

    private RouteCommandDispatcher dispatcher() {
        return new RouteCommandDispatcher(sink, catalog);
    }

    @Test
    public void convertsParametersUsingCatalogTypes() {
        RouteCommandDispatcher.Result result =
                dispatcher().dispatch(7, INTaker_IN, Collections.singletonList("0.75"));

        assertEquals(RouteCommandDispatcher.Result.DISPATCHED, result);
        assertEquals(Collections.singletonList(INTaker_IN), sink.names);
        assertEquals(1, sink.args.get(0).length);
        assertEquals(Double.valueOf(0.75), sink.args.get(0)[0]);
    }

    @Test
    public void dispatchesZeroArgumentCommand() {
        RouteCommandDispatcher.Result result =
                dispatcher().dispatch(2, SHOOTER_STOP, null);

        assertEquals(RouteCommandDispatcher.Result.DISPATCHED, result);
        assertEquals(0, sink.args.get(0).length);
    }

    @Test
    public void emptyCommandIsIgnoredNotFatal() {
        RouteCommandDispatcher.Result result =
                dispatcher().dispatch(0, "  ", Collections.<String>emptyList());

        assertEquals(RouteCommandDispatcher.Result.IGNORED, result);
        assertFalse(RouteCommandDispatcher.fatal(result));
        assertTrue(sink.names.isEmpty());
    }

    @Test
    public void nullEventAndNullCommandAreIgnored() {
        assertSame(RouteCommandDispatcher.Result.IGNORED, dispatcher().dispatch(null));
        assertSame(RouteCommandDispatcher.Result.IGNORED,
                dispatcher().dispatch(0, null, null));
        assertTrue(sink.names.isEmpty());
    }

    @Test
    public void unknownCommandIsFatal() {
        RouteCommandDispatcher.Result result =
                dispatcher().dispatch(3, "nope.nothing", Collections.<String>emptyList());

        assertEquals(RouteCommandDispatcher.Result.UNKNOWN_COMMAND, result);
        assertTrue(RouteCommandDispatcher.fatal(result));
        assertTrue(sink.names.isEmpty());
    }

    @Test
    public void wrongArgumentCountIsFatal() {
        RouteCommandDispatcher.Result result =
                dispatcher().dispatch(4, INTaker_IN, Arrays.asList("1", "2"));

        assertEquals(RouteCommandDispatcher.Result.ARG_MISMATCH, result);
        assertTrue(sink.names.isEmpty());
    }

    @Test
    public void unconvertibleArgumentIsFatal() {
        RouteCommandDispatcher.Result result =
                dispatcher().dispatch(5, INTaker_IN, Collections.singletonList("fast"));

        assertEquals(RouteCommandDispatcher.Result.BAD_ARGS, result);
        assertTrue(sink.names.isEmpty());
    }

    @Test
    public void rejectionIsFatal() {
        sink.accept = false;

        RouteCommandDispatcher.Result result =
                dispatcher().dispatch(6, SHOOTER_STOP, Collections.<String>emptyList());

        assertEquals(RouteCommandDispatcher.Result.REJECTED, result);
        assertTrue(RouteCommandDispatcher.fatal(result));
    }

    @Test
    public void sinkExceptionIsReportedNotPropagated() {
        sink.failure = new IllegalStateException("boom");

        RouteCommandDispatcher.Result result =
                dispatcher().dispatch(8, SHOOTER_STOP, Collections.<String>emptyList());

        assertEquals(RouteCommandDispatcher.Result.THREW, result);
        assertTrue(RouteCommandDispatcher.fatal(result));
    }

    @Test
    public void dispatchesTrajectoryEvent() throws Exception {
        RouteCommandDispatcher.Result result = dispatcher().dispatch(
                event(9, INTaker_IN, Collections.singletonList("1")));

        assertEquals(RouteCommandDispatcher.Result.DISPATCHED, result);
        assertEquals(Double.valueOf(1), sink.args.get(0)[0]);
    }

    private static CommandCatalog catalog(CommandCatalog.Descriptor... descriptors) {
        CommandCatalog result = new CommandCatalog();
        result.publish(Arrays.asList(descriptors));
        return result;
    }

    private static CommandCatalog.Descriptor descriptor(
            String name,
            List<String> names,
            List<String> types,
            List<List<String>> accepted) {
        return new CommandCatalog.Descriptor(name, names, types,
                Collections.<String>nCopies(names.size(), null), accepted);
    }

    @SuppressWarnings("unchecked")
    private static SplineTrajectoryLoader.TrajectoryEvent event(
            int frame,
            String command,
            List<String> params) throws Exception {
        Constructor<SplineTrajectoryLoader.TrajectoryEvent> constructor =
                (Constructor<SplineTrajectoryLoader.TrajectoryEvent>) SplineTrajectoryLoader
                        .TrajectoryEvent.class
                        .getDeclaredConstructor(int.class, String.class, String.class, List.class);
        constructor.setAccessible(true);
        return constructor.newInstance(frame, null, command, params);
    }

    private static final class RecordingSink implements RouteCommandDispatcher.Sink {
        final List<String> names = new ArrayList<>();
        final List<Object[]> args = new ArrayList<>();
        boolean accept = true;
        RuntimeException failure;

        @Override
        public boolean submit(String name, Object[] values) {
            names.add(name);
            args.add(values);
            if (failure != null) throw failure;
            return accept;
        }
    }
}
