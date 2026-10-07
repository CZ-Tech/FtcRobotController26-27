package org.firstinspires.ftc.teamcode.common.subsystem;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.LongSupplier;

/**
 * Base class for hardware-owning subsystems with a latest-execute-wins scheduler.
 *
 * <p>{@link #schedule()} creates a completely independent mutable draft. The framework
 * does not register, order, supersede or otherwise care about drafts. A draft may be
 * stored, passed around and completed later.</p>
 *
 * <p>Only {@link ScheduleApi#execute()} crosses the scheduling boundary: it atomically
 * seals that draft into an immutable built schedule, replaces the capacity-1 incoming
 * mailbox, and logically cancels the currently active schedule. Hardware remains owned
 * by the OpMode/control thread and is only touched from {@link #update()}.</p>
 */
public abstract class ScheduledSubsystem<A> {
    public static int MAX_STEPS_PER_UPDATE = 32;

    private final Class<A> apiType;
    private final LongSupplier nanoTime;
    private final AtomicReference<BuiltSchedule> incoming = new AtomicReference<>();
    private final Object executionLock = new Object();

    private volatile BuiltSchedule active;
    private final A cancelledProxy;

    protected ScheduledSubsystem(Class<A> apiType) {
        this(apiType, System::nanoTime);
    }

    ScheduledSubsystem(Class<A> apiType, LongSupplier nanoTime) {
        if (apiType == null || !apiType.isInterface()) {
            throw new IllegalArgumentException("schedule API must be an interface");
        }
        if (!ScheduleApi.class.isAssignableFrom(apiType)) {
            throw new IllegalArgumentException(
                    "schedule API must extend ScheduleApi: " + apiType.getName());
        }

        this.apiType = apiType;
        this.nanoTime = nanoTime;
        validateScheduleApi();
        this.cancelledProxy = createCancelledProxy();
    }

    /** Creates a new independent draft. No running or pending work is affected. */
    public final A schedule() {
        return createDraftProxy(new DraftSchedule());
    }

    /**
     * One bounded-time scheduler tick. Must run on the OpMode/control thread that owns
     * this subsystem's hardware.
     */
    public final void update() {
        boolean cancelledActive = false;

        synchronized (executionLock) {
            if (active != null && active.cancelled) {
                active = null;
                cancelledActive = true;
            }

            BuiltSchedule next = incoming.getAndSet(null);
            if (next != null) {
                if (active != null) {
                    active.cancelled = true;
                    active = null;
                    cancelledActive = true;
                }
                if (!next.cancelled) active = next;
            }
        }

        if (cancelledActive) onScheduleCancelled();

        BuiltSchedule running = active;
        if (running != null) {
            running.update(nanoTime.getAsLong());
            boolean wasCancelled = running.cancelled;
            if (running.finished || wasCancelled) {
                synchronized (executionLock) {
                    if (active == running) active = null;
                }
            }
            if (wasCancelled) onScheduleCancelled();
        }

        periodic();
    }

    /**
     * Cancels pending and active executed schedules.
     *
     * <p>Independent drafts are intentionally unaffected because the subsystem does not
     * own or track them before execute().</p>
     */
    public final void cancelSchedule() {
        synchronized (executionLock) {
            BuiltSchedule dropped = incoming.getAndSet(null);
            if (dropped != null) dropped.cancelled = true;
            if (active != null) active.cancelled = true;
        }
    }

    public final boolean hasActiveSchedule() {
        BuiltSchedule running = active;
        return running != null && !running.cancelled && !running.finished;
    }

    /** Optional normal per-loop work, always called from {@link #update()}. */
    protected void periodic() {}

    /**
     * Optional hardware-safe reaction when an executed schedule is preempted/cancelled.
     * Called only from {@link #update()}, never from a producer thread.
     */
    protected void onScheduleCancelled() {}

    private A createDraftProxy(DraftSchedule draft) {
        InvocationHandler handler = (proxy, method, args) ->
                invokeDraft(proxy, draft, method, args);
        return apiType.cast(Proxy.newProxyInstance(
                apiType.getClassLoader(),
                new Class<?>[]{apiType},
                handler));
    }

    private A createCancelledProxy() {
        InvocationHandler handler = (proxy, method, args) -> {
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, method, args, "CancelledSchedule");
            }
            if ("isCancelled".equals(method.getName()) && method.getParameterCount() == 0) {
                return true;
            }
            if ("isExecuted".equals(method.getName()) && method.getParameterCount() == 0) {
                return false;
            }
            return returnProxyOrDefault(proxy, method);
        };
        return apiType.cast(Proxy.newProxyInstance(
                apiType.getClassLoader(),
                new Class<?>[]{apiType},
                handler));
    }

    private Object invokeDraft(
            Object proxy,
            DraftSchedule draft,
            Method method,
            Object[] args) throws Exception {
        if (method.getDeclaringClass() == Object.class) {
            return objectMethod(proxy, method, args, "ScheduleDraft");
        }

        String name = method.getName();

        if ("isCancelled".equals(name) && method.getParameterCount() == 0) {
            return draft.isCancelled();
        }
        if ("isExecuted".equals(name) && method.getParameterCount() == 0) {
            return draft.isExecuted();
        }

        synchronized (draft) {
            if (draft.state == DraftState.CANCELLED || draft.executedScheduleCancelled()) {
                return invokeCancelled(method, args);
            }

            if ("waitMillis".equals(name) && method.getParameterCount() == 1) {
                if (draft.state == DraftState.EXECUTED) return proxy;
                long millis = (Long) args[0];
                if (millis < 0) throw new IllegalArgumentException("millis < 0");
                draft.steps.add(new WaitMillisStep(millis));
                return proxy;
            }

            if ("waitUntil".equals(name) && method.getParameterCount() == 1) {
                if (draft.state == DraftState.EXECUTED) return proxy;
                BooleanSupplier condition = (BooleanSupplier) args[0];
                if (condition == null) throw new IllegalArgumentException("condition == null");
                draft.steps.add(new WaitUntilStep(condition));
                return proxy;
            }

            if ("execute".equals(name) && method.getParameterCount() == 0) {
                if (draft.state == DraftState.EXECUTED) return proxy;
                BuiltSchedule built = new BuiltSchedule(new ArrayList<>(draft.steps));
                draft.state = DraftState.EXECUTED;
                draft.built = built;
                publish(built);
                return proxy;
            }

            if ("cancel".equals(name) && method.getParameterCount() == 0) {
                draft.state = DraftState.CANCELLED;
                if (draft.built != null) draft.built.cancelled = true;
                return cancelledProxy;
            }

            if (draft.state == DraftState.EXECUTED) return proxy;

            Method target = resolveTargetMethod(method);
            draft.steps.add(new InvokeStep(target, copyArgs(args)));
            return proxy;
        }
    }

    /**
     * Submission boundary. Last execute wins; draft creation order is irrelevant.
     */
    private void publish(BuiltSchedule built) {
        synchronized (executionLock) {
            BuiltSchedule dropped = incoming.getAndSet(built);
            if (dropped != null) dropped.cancelled = true;

            // Logical cancellation may happen from any producer thread. Hardware cleanup
            // is deferred to update()/onScheduleCancelled() on the control thread.
            if (active != null) active.cancelled = true;
        }
    }

    private Object invokeCancelled(Method method, Object[] args) {
        try {
            return method.invoke(cancelledProxy, args);
        } catch (IllegalAccessException | InvocationTargetException e) {
            throw new IllegalStateException("cancelled schedule proxy failure", e);
        }
    }

    private Method resolveTargetMethod(Method scheduleMethod) {
        try {
            Method target = getClass().getMethod(
                    scheduleMethod.getName(),
                    scheduleMethod.getParameterTypes());
            target.setAccessible(true);
            return target;
        } catch (NoSuchMethodException e) {
            throw new IllegalStateException(
                    "Schedule method has no matching subsystem method: "
                            + getClass().getSimpleName() + "."
                            + scheduleMethod.getName()
                            + Arrays.toString(scheduleMethod.getParameterTypes()),
                    e);
        }
    }

    private void validateScheduleApi() {
        for (Method method : apiType.getMethods()) {
            if (method.getDeclaringClass() == ScheduleApi.class
                    || method.getDeclaringClass() == Object.class) {
                continue;
            }
            resolveTargetMethod(method);
        }
    }

    private static Object[] copyArgs(Object[] args) {
        return args == null ? new Object[0] : args.clone();
    }

    private static Object objectMethod(
            Object proxy,
            Method method,
            Object[] args,
            String text) {
        switch (method.getName()) {
            case "toString":
                return text;
            case "hashCode":
                return System.identityHashCode(proxy);
            case "equals":
                return proxy == args[0];
            default:
                throw new UnsupportedOperationException(method.toString());
        }
    }

    private static Object returnProxyOrDefault(Object proxy, Method method) {
        Class<?> type = method.getReturnType();
        if (type.isInstance(proxy)) return proxy;
        if (!type.isPrimitive()) return null;
        if (type == boolean.class) return false;
        if (type == char.class) return '\0';
        if (type == byte.class) return (byte) 0;
        if (type == short.class) return (short) 0;
        if (type == int.class) return 0;
        if (type == long.class) return 0L;
        if (type == float.class) return 0f;
        if (type == double.class) return 0d;
        return null;
    }

    private enum DraftState {
        OPEN,
        EXECUTED,
        CANCELLED
    }

    private final class DraftSchedule {
        final List<Step> steps = new ArrayList<>();
        DraftState state = DraftState.OPEN;
        BuiltSchedule built;

        synchronized boolean isCancelled() {
            return state == DraftState.CANCELLED || executedScheduleCancelled();
        }

        synchronized boolean isExecuted() {
            return state == DraftState.EXECUTED;
        }

        boolean executedScheduleCancelled() {
            return built != null && built.cancelled;
        }
    }

    private interface Step {
        /** @return true when this step is complete. */
        boolean update(long nowNanos);
    }

    private final class InvokeStep implements Step {
        private final Method method;
        private final Object[] args;
        private boolean executed;

        InvokeStep(Method method, Object[] args) {
            this.method = method;
            this.args = args;
        }

        @Override
        public boolean update(long nowNanos) {
            if (executed) return true;
            try {
                method.invoke(ScheduledSubsystem.this, args);
                executed = true;
                return true;
            } catch (IllegalAccessException e) {
                throw new IllegalStateException("Cannot invoke scheduled method " + method, e);
            } catch (InvocationTargetException e) {
                Throwable cause = e.getCause();
                if (cause instanceof RuntimeException) throw (RuntimeException) cause;
                if (cause instanceof Error) throw (Error) cause;
                throw new IllegalStateException("Scheduled method failed: " + method, cause);
            }
        }
    }

    private static final class WaitMillisStep implements Step {
        private final long durationNanos;
        private long deadlineNanos = Long.MIN_VALUE;

        WaitMillisStep(long millis) {
            durationNanos = millis * 1_000_000L;
        }

        @Override
        public boolean update(long nowNanos) {
            if (deadlineNanos == Long.MIN_VALUE) {
                deadlineNanos = nowNanos + durationNanos;
            }
            return nowNanos >= deadlineNanos;
        }
    }

    private static final class WaitUntilStep implements Step {
        private final BooleanSupplier condition;

        WaitUntilStep(BooleanSupplier condition) {
            this.condition = condition;
        }

        @Override
        public boolean update(long nowNanos) {
            return condition.getAsBoolean();
        }
    }

    private final class BuiltSchedule {
        final List<Step> steps;
        int index;
        boolean finished;
        volatile boolean cancelled;

        BuiltSchedule(List<Step> steps) {
            this.steps = steps;
            this.finished = steps.isEmpty();
        }

        void update(long nowNanos) {
            int budget = Math.max(1, MAX_STEPS_PER_UPDATE);
            while (!cancelled && !finished && budget-- > 0) {
                Step step = steps.get(index);
                if (!step.update(nowNanos)) return;
                index++;
                if (index >= steps.size()) finished = true;
            }
        }
    }
}
