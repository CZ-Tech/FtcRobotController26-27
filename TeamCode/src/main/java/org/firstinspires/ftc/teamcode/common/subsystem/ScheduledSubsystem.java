package org.firstinspires.ftc.teamcode.common.subsystem;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.LongSupplier;

/**
 * Base class for hardware-owning subsystems with a latest-wins fluent scheduler.
 *
 * <p>Each subsystem owns exactly one mutable draft, one capacity-1 thread-safe
 * incoming mailbox, and at most one active built schedule. Calling
 * {@link #schedule()} immediately invalidates every older draft, pending schedule,
 * and active schedule. The old proxy then behaves as a cancelled null object.</p>
 *
 * <p>Schedule proxies never touch hardware. Subsystem methods captured by the
 * proxy are invoked only from {@link #update()}, so callers may safely construct
 * schedules from another thread without moving hardware access off the OpMode
 * thread.</p>
 */
public abstract class ScheduledSubsystem<A> {
    public static int MAX_STEPS_PER_UPDATE = 32;

    private final Class<A> apiType;
    private final LongSupplier nanoTime;
    private final AtomicLong generation = new AtomicLong(0);
    private final AtomicReference<BuiltSchedule> incoming = new AtomicReference<>();
    private final Object draftLock = new Object();

    private DraftSchedule currentDraft;
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

    /**
     * Starts a new draft and immediately supersedes all previous work for this subsystem.
     */
    public final A schedule() {
        final DraftSchedule draft;
        synchronized (draftLock) {
            final long nextGeneration = generation.incrementAndGet();
            if (currentDraft != null) currentDraft.cancelled = true;
            draft = new DraftSchedule(nextGeneration);
            currentDraft = draft;

            BuiltSchedule dropped = incoming.getAndSet(null);
            if (dropped != null) dropped.cancelled = true;
        }

        return createDraftProxy(draft);
    }

    /**
     * One bounded-time scheduler tick. This method must be called from the OpMode/control
     * thread that owns the subsystem hardware.
     */
    public final void update() {
        final long currentGeneration = generation.get();
        if (active != null && active.generation != currentGeneration) {
            active.cancelled = true;
            active = null;
            onScheduleCancelled();
        }

        BuiltSchedule next = incoming.getAndSet(null);
        if (next != null) {
            if (next.generation != generation.get() || next.cancelled) {
                next.cancelled = true;
            } else {
                if (active != null) {
                    active.cancelled = true;
                    onScheduleCancelled();
                }
                active = next;
            }
        }

        if (active != null) {
            active.update(nanoTime.getAsLong());
            if (active.finished || active.cancelled) active = null;
        }

        periodic();
    }

    /**
     * Immediately invalidates draft, pending and active work.
     *
     * <p>Hardware cleanup remains on the control thread: {@link #onScheduleCancelled()}
     * runs from the next {@link #update()} when an active chain is observed stale.</p>
     */
    public final void cancelSchedule() {
        synchronized (draftLock) {
            generation.incrementAndGet();
            if (currentDraft != null) currentDraft.cancelled = true;
            currentDraft = null;

            BuiltSchedule dropped = incoming.getAndSet(null);
            if (dropped != null) dropped.cancelled = true;
        }
    }

    public final boolean hasActiveSchedule() {
        return active != null && !active.cancelled && !active.finished;
    }

    /**
     * Optional normal per-loop work for the subsystem. It is always called from update().
     */
    protected void periodic() {}

    /**
     * Optional hardware-safe reaction when an active schedule is preempted.
     * Called only from update(), never from a producer/network thread.
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
            if ("isBuilt".equals(method.getName()) && method.getParameterCount() == 0) {
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
            return objectMethod(proxy, method, args, "Schedule#" + draft.generation);
        }

        String name = method.getName();
        boolean stale = draft.cancelled || draft.generation != generation.get();

        if ("isCancelled".equals(name) && method.getParameterCount() == 0) {
            return stale;
        }

        if ("isBuilt".equals(name) && method.getParameterCount() == 0) {
            return draft.sealed && !stale;
        }

        if (stale) {
            return invokeCancelled(method, args);
        }

        // build() seals this handle. Further fluent mutations are harmless no-ops;
        // the handle still reports built rather than cancelled until superseded.
        if (draft.sealed) {
            return proxy;
        }

        if ("waitMillis".equals(name) && method.getParameterCount() == 1) {
            long millis = (Long) args[0];
            if (millis < 0) throw new IllegalArgumentException("millis < 0");
            append(draft, new WaitMillisStep(millis));
            return proxy;
        }

        if ("waitUntil".equals(name) && method.getParameterCount() == 1) {
            BooleanSupplier condition = (BooleanSupplier) args[0];
            if (condition == null) throw new IllegalArgumentException("condition == null");
            append(draft, new WaitUntilStep(condition));
            return proxy;
        }

        if ("build".equals(name) && method.getParameterCount() == 0) {
            buildNow(draft);
            return proxy;
        }

        if ("cancel".equals(name) && method.getParameterCount() == 0) {
            cancelDraft(draft);
            return cancelledProxy;
        }

        Method target = resolveTargetMethod(method);
        append(draft, new InvokeStep(target, copyArgs(args)));
        return proxy;
    }

    private Object invokeCancelled(Method method, Object[] args) {
        try {
            return method.invoke(cancelledProxy, args);
        } catch (IllegalAccessException | InvocationTargetException e) {
            throw new IllegalStateException("cancelled schedule proxy failure", e);
        }
    }

    private boolean isLive(DraftSchedule draft) {
        return !draft.cancelled
                && !draft.sealed
                && draft.generation == generation.get();
    }

    private void append(DraftSchedule draft, Step step) {
        synchronized (draftLock) {
            if (!isLive(draft) || currentDraft != draft) {
                draft.cancelled = true;
                return;
            }
            draft.steps.add(step);
        }
    }

    private void buildNow(DraftSchedule draft) {
        BuiltSchedule built;
        synchronized (draftLock) {
            if (!isLive(draft) || currentDraft != draft) {
                draft.cancelled = true;
                return;
            }
            built = sealLocked(draft);
        }
        publish(built);
    }

    private BuiltSchedule sealLocked(DraftSchedule draft) {
        draft.sealed = true;
        if (currentDraft == draft) currentDraft = null;
        return new BuiltSchedule(
                draft.generation,
                new ArrayList<>(draft.steps));
    }

    private void cancelDraft(DraftSchedule draft) {
        synchronized (draftLock) {
            draft.cancelled = true;
            if (currentDraft == draft) currentDraft = null;
        }

        // Only the current generation is allowed to invalidate active/pending work.
        if (draft.generation == generation.get()) {
            cancelSchedule();
        }
    }

    private void publish(BuiltSchedule built) {
        if (built.generation != generation.get()) {
            built.cancelled = true;
            return;
        }
        BuiltSchedule dropped = incoming.getAndSet(built);
        if (dropped != null) dropped.cancelled = true;
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

    private final class DraftSchedule {
        final long generation;
        final List<Step> steps = new ArrayList<>();
        boolean sealed;
        boolean cancelled;

        DraftSchedule(long generation) {
            this.generation = generation;
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
        final long generation;
        final List<Step> steps;
        int index;
        boolean finished;
        volatile boolean cancelled;

        BuiltSchedule(long generation, List<Step> steps) {
            this.generation = generation;
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
