package org.firstinspires.ftc.mockrobot;

import org.firstinspires.ftc.teamcode.common.network.ControlGate;
import org.firstinspires.ftc.teamcode.common.network.ControlRequest;
import org.firstinspires.ftc.teamcode.common.network.ExecutionStateStore;
import org.firstinspires.ftc.teamcode.common.network.RobotRuntimeStore;
import org.firstinspires.ftc.teamcode.common.network.RouteRepository;
import org.firstinspires.ftc.teamcode.common.opmode.OpModeLifecycleService;

import java.util.Random;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

/** 60 Hz robot/trajectory simulation. Network threads only observe its published snapshots. */
public final class MockSimulationEngine implements AutoCloseable {
    public static final class Snapshot {
        public final double x;
        public final double y;
        public final double heading;
        public final Double idealX;
        public final Double idealY;
        public final Double idealHeading;
        public final double trackingError;
        public final String routeName;
        public final boolean routeRunning;
        public final double routeSeconds;
        public final double routeTotalSeconds;

        Snapshot(
                double x,
                double y,
                double heading,
                Double idealX,
                Double idealY,
                Double idealHeading,
                double trackingError,
                String routeName,
                boolean routeRunning,
                double routeSeconds,
                double routeTotalSeconds) {
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.idealX = idealX;
            this.idealY = idealY;
            this.idealHeading = idealHeading;
            this.trackingError = trackingError;
            this.routeName = routeName;
            this.routeRunning = routeRunning;
            this.routeSeconds = routeSeconds;
            this.routeTotalSeconds = routeTotalSeconds;
        }
    }

    private final MockRouteStore routes;
    private final MockSettings settings;
    private final ControlGate control;
    private final ExecutionStateStore execution;
    private final RobotRuntimeStore runtime;
    private final OpModeLifecycleService opModes;
    private final ScheduledExecutorService executor =
            Executors.newSingleThreadScheduledExecutor(r -> {
                Thread thread = new Thread(r, "MockRobot-Simulation");
                thread.setDaemon(true);
                return thread;
            });
    private final AtomicReference<Snapshot> snapshot = new AtomicReference<>(
            new Snapshot(0, 0, 0, null, null, null, 0, null, false, 0, 0));

    private Random random;
    private long lastTickNanos;
    private long lastPublishNanos;
    private MockRoutePlan plan;
    private String routeName;
    private long planStartedNanos;
    private long activeRequestId;
    private boolean planRunning;
    private boolean autoStopAtEnd;
    private Runnable autoStopCallback;
    private boolean dragging;
    private double x;
    private double y;
    private double heading;
    private double idealX;
    private double idealY;
    private double idealHeading;
    private boolean hasIdeal;
    private double manualOffsetX;
    private double manualOffsetY;
    private double manualOffsetHeading;
    private double noiseX;
    private double noiseY;
    private double noiseHeading;

    public MockSimulationEngine(
            MockRouteStore routes,
            MockSettings settings,
            ControlGate control,
            ExecutionStateStore execution,
            RobotRuntimeStore runtime,
            OpModeLifecycleService opModes) {
        this.routes = routes;
        this.settings = settings;
        this.control = control;
        this.execution = execution;
        this.runtime = runtime;
        this.opModes = opModes;
        random = new Random(settings.randomSeed());
    }

    public void start() {
        lastTickNanos = System.nanoTime();
        executor.scheduleAtFixedRate(this::tickSafely, 0, 16, TimeUnit.MILLISECONDS);
    }

    public void setAutoStopCallback(Runnable callback) {
        autoStopCallback = callback;
    }

    public Snapshot snapshot() {
        return snapshot.get();
    }

    public synchronized MockRoutePlan activePlan() {
        return plan;
    }

    public synchronized void onInit(MockOpModeProfile profile) {
        cancelPlan();
        execution.publish(ExecutionStateStore.State.NOT_READY, 0, null);
        if (profile.routeName() != null) {
            RouteRepository.Entry entry = routes.get(profile.routeName());
            if (entry != null) {
                try {
                    MockRoutePlan preview = MockRoutePlan.parse(entry.json);
                    MockRoutePlan.Pose start = preview.startPose();
                    idealX = start.x;
                    idealY = start.y;
                    idealHeading = start.heading;
                    hasIdeal = true;
                } catch (Exception ignored) {}
            }
        }
    }

    public synchronized void onStart(MockOpModeProfile profile) {
        control.activate();
        execution.publish(ExecutionStateStore.State.IDLE, 0, null);
        String selected = profile.routeName();
        if (selected != null && !selected.isBlank()) {
            startSavedRoute(selected, 0, profile.autoStop());
        }
    }

    public synchronized void onStop() {
        cancelPlan();
        control.deactivate();
        execution.publish(ExecutionStateStore.State.NOT_READY, 0, null);
        hasIdeal = false;
    }

    public synchronized void snapToRouteStart(String name) {
        RouteRepository.Entry entry = routes.get(name);
        if (entry == null) return;
        try {
            MockRoutePlan route = MockRoutePlan.parse(entry.json);
            MockRoutePlan.Pose pose = route.startPose();
            setPoseInternal(pose.x, pose.y, pose.heading);
            idealX = pose.x;
            idealY = pose.y;
            idealHeading = pose.heading;
            hasIdeal = true;
            resetErrors();
        } catch (Exception e) {
            MockLog.error("Simulation", "Invalid route " + name, e);
        }
    }

    public synchronized void beginDrag() {
        dragging = true;
    }

    public synchronized void dragTo(double newX, double newY) {
        if (!dragging) return;
        x = clampField(newX);
        y = clampField(newY);
        if (hasIdeal) {
            manualOffsetX = x - idealX - noiseX;
            manualOffsetY = y - idealY - noiseY;
        }
        publishSnapshot(0, plan == null ? 0 : plan.totalTime());
    }

    public synchronized void endDrag() {
        dragging = false;
    }

    public synchronized void setPose(double newX, double newY, double newHeading) {
        x = clampField(newX);
        y = clampField(newY);
        heading = MockRoutePlan.normalizeDegrees(newHeading);
        if (hasIdeal) {
            manualOffsetX = x - idealX - noiseX;
            manualOffsetY = y - idealY - noiseY;
            manualOffsetHeading =
                    MockRoutePlan.normalizeDegrees(heading - idealHeading - noiseHeading);
        }
        publishSnapshot(0, plan == null ? 0 : plan.totalTime());
    }

    public synchronized void addRandomKick() {
        manualOffsetX += random.nextGaussian() * 6;
        manualOffsetY += random.nextGaussian() * 6;
        manualOffsetHeading += random.nextGaussian() * 8;
    }

    public synchronized void resetTrackingError() {
        resetErrors();
        if (hasIdeal) setPoseInternal(idealX, idealY, idealHeading);
    }

    public synchronized void reseedNoise() {
        random = new Random(settings.randomSeed());
        noiseX = noiseY = noiseHeading = 0;
    }

    private void tickSafely() {
        try {
            tick();
        } catch (Throwable t) {
            MockLog.error("Simulation", "simulation tick failed", t);
        }
    }

    private synchronized void tick() {
        long now = System.nanoTime();
        double dt = Math.min(0.1, Math.max(0.001, (now - lastTickNanos) / 1e9));
        lastTickNanos = now;

        ControlRequest request = control.pollLatest();
        if (request != null) {
            if (request.type == ControlRequest.Type.EXECUTE_SAVED_PATH) {
                startSavedRoute(request.pathName, request.id, false);
            } else if (request.type == ControlRequest.Type.EXECUTE_INLINE_PATH) {
                startInlineRoute(request.inlineJson, request.id, "inline", false);
            }
        }

        double routeSeconds = 0;
        double routeTotal = plan == null ? 0 : plan.totalTime();
        boolean finished = false;
        if (planRunning && plan != null) {
            routeSeconds = (now - planStartedNanos) / 1e9 * settings.speedScale();
            if (routeSeconds >= routeTotal && routeTotal > 0) {
                if (settings.loopRoute()) {
                    planStartedNanos = now;
                    routeSeconds = 0;
                } else {
                    routeSeconds = routeTotal;
                    finished = true;
                }
            }
            MockRoutePlan.Pose ideal = plan.sample(routeSeconds);
            idealX = ideal.x;
            idealY = ideal.y;
            idealHeading = ideal.heading;
            hasIdeal = true;

            updateNoise(dt);
            if (!dragging) {
                double decay = Math.exp(-dt / settings.correctionSeconds());
                manualOffsetX *= decay;
                manualOffsetY *= decay;
                manualOffsetHeading *= decay;
                x = clampField(idealX + manualOffsetX + noiseX);
                y = clampField(idealY + manualOffsetY + noiseY);
                heading = MockRoutePlan.normalizeDegrees(
                        idealHeading + manualOffsetHeading + noiseHeading);
            }
        }

        if (finished) {
            planRunning = false;
            execution.publish(ExecutionStateStore.State.IDLE, activeRequestId, routeName);
            if (autoStopAtEnd && autoStopCallback != null) {
                executor.execute(autoStopCallback);
            }
        }

        int hz = Math.max(1, settings.poseHz());
        if (now - lastPublishNanos >= 1_000_000_000L / hz) {
            OpModeLifecycleService.Snapshot mode = opModes.snapshot();
            boolean active = mode.phase != OpModeLifecycleService.Phase.STOPPED;
            runtime.publish(active, active ? mode.activeName : null, x, y, heading);
            lastPublishNanos = now;
        }
        publishSnapshot(routeSeconds, routeTotal);
    }

    private void updateNoise(double dt) {
        if (!settings.noiseEnabled()) {
            noiseX = noiseY = noiseHeading = 0;
            return;
        }
        double tau = 0.75;
        double decay = Math.exp(-dt / tau);
        double drive = Math.sqrt(Math.max(0, 1 - decay * decay));
        double sigma = settings.positionNoiseSigma();
        noiseX = noiseX * decay + random.nextGaussian() * sigma * drive;
        noiseY = noiseY * decay + random.nextGaussian() * sigma * drive;
        double headingSigma = settings.headingNoiseSigma();
        noiseHeading = noiseHeading * decay
                + random.nextGaussian() * headingSigma * drive;
    }

    private void startSavedRoute(String name, long requestId, boolean autoStop) {
        RouteRepository.Entry entry = routes.get(name);
        if (entry == null) {
            execution.publish(ExecutionStateStore.State.IDLE, requestId, name);
            return;
        }
        startInlineRoute(entry.json, requestId, name, autoStop);
    }

    private void startInlineRoute(
            String json,
            long requestId,
            String subject,
            boolean autoStop) {
        try {
            MockRoutePlan next = MockRoutePlan.parse(json);
            plan = next;
            routeName = subject;
            activeRequestId = requestId;
            autoStopAtEnd = autoStop;
            planStartedNanos = System.nanoTime();
            planRunning = next.totalTime() > 0;
            MockRoutePlan.Pose start = next.startPose();
            idealX = start.x;
            idealY = start.y;
            idealHeading = start.heading;
            hasIdeal = true;
            if (!planRunning) setPoseInternal(start.x, start.y, start.heading);
            execution.publish(
                    planRunning
                            ? ExecutionStateStore.State.RUNNING
                            : ExecutionStateStore.State.IDLE,
                    requestId,
                    subject);
            MockLog.info("Simulation", "Started route " + subject);
        } catch (Exception e) {
            execution.publish(ExecutionStateStore.State.IDLE, requestId, subject);
            MockLog.error("Simulation", "Invalid route " + subject, e);
        }
    }

    private void cancelPlan() {
        planRunning = false;
        plan = null;
        routeName = null;
        activeRequestId = 0;
        autoStopAtEnd = false;
    }

    private void resetErrors() {
        manualOffsetX = manualOffsetY = manualOffsetHeading = 0;
        noiseX = noiseY = noiseHeading = 0;
    }

    private void setPoseInternal(double newX, double newY, double newHeading) {
        x = clampField(newX);
        y = clampField(newY);
        heading = MockRoutePlan.normalizeDegrees(newHeading);
    }

    private void publishSnapshot(double routeSeconds, double routeTotal) {
        double error = hasIdeal ? Math.hypot(x - idealX, y - idealY) : 0;
        snapshot.set(new Snapshot(
                x,
                y,
                heading,
                hasIdeal ? idealX : null,
                hasIdeal ? idealY : null,
                hasIdeal ? idealHeading : null,
                error,
                routeName,
                planRunning,
                routeSeconds,
                routeTotal));
    }

    private static double clampField(double value) {
        return Math.max(-72, Math.min(72, value));
    }

    @Override
    public void close() {
        executor.shutdownNow();
    }
}
