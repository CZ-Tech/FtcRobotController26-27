package org.firstinspires.ftc.teamcode.common.network;

import android.util.Log;

import org.firstinspires.ftc.teamcode.common.subsystem.ScheduleApi;
import org.firstinspires.ftc.teamcode.common.subsystem.Scheduler;
import org.firstinspires.ftc.teamcode.common.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.common.util.Param;

import java.lang.annotation.Annotation;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.lang.reflect.ParameterizedType;
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Exposes subsystem schedule methods as remote commands.
 *
 * <p>Every public {@link Scheduler} field of the host (for {@link Subsystem}: each
 * subsystem) contributes one command per method of its generated schedule interface,
 * named {@code <field>.<method>}. Overloads of one method become one command that accepts
 * any of the overload argument counts; the published descriptor lists the longest one.
 * Trailing parameters with a {@link Param#defaultValue()} may be omitted by the caller
 * and are filled in at {@link #submit}; a method with {@code d} defaults is therefore
 * callable with {@code n-d} to {@code n} arguments, and that range must not overlap
 * another overload.</p>
 *
 * <p>The catalog is built from class metadata only, so it can be published before any
 * OpMode (and any {@code Robot}) exists. A binding instance is the {@link
 * CommandDispatcher.Sink} for one host: {@link #submit} builds a one-step schedule and
 * {@code execute()}s it, which only publishes it to the scheduler mailbox. Hardware is
 * touched later by {@code Subsystem.update()} on the OpMode thread.</p>
 *
 * <p>Latest-execute-wins applies per subsystem: a command replaces the schedule that
 * subsystem is currently running or has pending.</p>
 */
public final class SubsystemCommandBinding implements CommandDispatcher.Sink {
    private static final String TAG = "SubsystemCommands";
    private static final Set<Class<?>> SUPPORTED = new HashSet<>();

    static {
        Collections.addAll(SUPPORTED,
                int.class, Integer.class, long.class, Long.class,
                short.class, Short.class, byte.class, Byte.class,
                float.class, Float.class, double.class, Double.class,
                boolean.class, Boolean.class, char.class, Character.class,
                String.class);
    }

    /** Class-level command metadata for one host type. */
    public static final class Commands {
        private final Map<String, Command> byName;
        private final List<CommandCatalog.Descriptor> descriptors;

        private Commands(Map<String, Command> byName) {
            this.byName = byName;
            List<CommandCatalog.Descriptor> list = new ArrayList<>();
            for (Command command : byName.values()) list.add(command.descriptor());
            Collections.sort(list, new Comparator<CommandCatalog.Descriptor>() {
                @Override
                public int compare(CommandCatalog.Descriptor a, CommandCatalog.Descriptor b) {
                    return a.name.compareTo(b.name);
                }
            });
            this.descriptors = Collections.unmodifiableList(list);
        }

        public List<CommandCatalog.Descriptor> descriptors() {
            return descriptors;
        }
    }

    /**
     * One schedule-interface method together with its trailing default values. It is
     * reachable under every argument count from {@code parameterCount - defaults.length}
     * up to {@code parameterCount}.
     */
    private static final class Overload {
        final Method method;
        /** Raw default text per parameter; null entries are required parameters. */
        final List<String> defaultTexts;
        /** Converted values of the trailing defaulted parameters, in order. */
        final Object[] defaults;

        Overload(Method method, List<String> defaultTexts, Object[] defaults) {
            this.method = method;
            this.defaultTexts = defaultTexts;
            this.defaults = defaults;
        }

        int parameterCount() {
            return method.getParameterTypes().length;
        }

        /** Appends the defaults for the parameters {@code values} does not cover. */
        Object[] complete(Object[] values) {
            Object[] full = new Object[parameterCount()];
            System.arraycopy(values, 0, full, 0, values.length);
            int firstDefaulted = parameterCount() - defaults.length;
            for (int i = values.length; i < full.length; i++) {
                full[i] = defaults[i - firstDefaulted];
            }
            return full;
        }
    }

    private static final class Command {
        final String name;
        final Field field;
        /** The overload serving each accepted argument count. */
        final Map<Integer, Overload> byArity = new LinkedHashMap<>();

        Command(String name, Field field) {
            this.name = name;
            this.field = field;
        }

        CommandCatalog.Descriptor descriptor() {
            int longest = -1;
            for (int arity : byArity.keySet()) longest = Math.max(longest, arity);
            Overload primary = byArity.get(longest);

            List<List<String>> accepted = new ArrayList<>();
            for (Map.Entry<Integer, Overload> entry : byArity.entrySet()) {
                List<String> types = parameterTypes(entry.getValue().method);
                accepted.add(types.subList(0, entry.getKey()));
            }
            return new CommandCatalog.Descriptor(
                    name,
                    parameterNames(primary.method),
                    parameterTypes(primary.method),
                    primary.defaultTexts,
                    accepted);
        }
    }

    private static volatile Commands subsystemCommands;

    /** Commands of {@link Subsystem}, scanned once per process. */
    public static Commands subsystemCommands() {
        Commands value = subsystemCommands;
        if (value == null) {
            synchronized (SubsystemCommandBinding.class) {
                value = subsystemCommands;
                if (value == null) {
                    value = scan(Subsystem.class);
                    subsystemCommands = value;
                }
            }
        }
        return value;
    }

    /**
     * Scans {@code hostType}'s public {@link Scheduler} fields. Methods with unsupported
     * parameter types and commands with two overloads of the same argument count are
     * skipped with a warning rather than published.
     */
    public static Commands scan(Class<?> hostType) {
        Map<String, Command> commands = new LinkedHashMap<>();
        Set<String> conflicted = new HashSet<>();

        for (Field field : hostType.getFields()) {
            if (Modifier.isStatic(field.getModifiers())) continue;
            if (!Scheduler.class.isAssignableFrom(field.getType())) continue;

            Class<?> api = scheduleApiType(field.getGenericType());
            if (api == null || !api.isInterface()) {
                Log.w(TAG, "Cannot resolve schedule API of field " + field.getName());
                continue;
            }

            for (Method method : api.getMethods()) {
                if (method.getDeclaringClass() == ScheduleApi.class
                        || method.getDeclaringClass() == Object.class
                        || Modifier.isStatic(method.getModifiers())) {
                    continue;
                }
                String name = field.getName() + "." + method.getName();
                if (!allParametersSupported(method)) {
                    Log.w(TAG, "Skipping " + name + ": unsupported parameter type");
                    continue;
                }
                method.setAccessible(true);

                Overload overload;
                try {
                    overload = overloadOf(method);
                } catch (IllegalArgumentException e) {
                    Log.e(TAG, "Skipping " + name + ": " + e.getMessage());
                    continue;
                }

                Command command = commands.get(name);
                if (command == null) {
                    command = new Command(name, field);
                    commands.put(name, command);
                }
                int count = overload.parameterCount();
                for (int arity = count - overload.defaults.length; arity <= count; arity++) {
                    if (command.byArity.put(arity, overload) != null) conflicted.add(name);
                }
            }
        }

        for (String name : conflicted) {
            Log.e(TAG, "Skipping " + name
                    + ": two overloads can be called with the same number of arguments");
            commands.remove(name);
        }
        return new Commands(commands);
    }

    /**
     * Reads the {@link Param} defaults of {@code method}. Defaulted parameters must be the
     * trailing ones and every default must convert to its parameter type.
     */
    private static Overload overloadOf(Method method) {
        Class<?>[] types = method.getParameterTypes();
        List<String> texts = new ArrayList<>();
        List<Object> converted = new ArrayList<>();
        for (int i = 0; i < types.length; i++) {
            Param param = paramAnnotation(method, i);
            String text = param == null || Param.NO_DEFAULT.equals(param.defaultValue())
                    ? null
                    : param.defaultValue();
            texts.add(text);
            if (text == null) {
                if (!converted.isEmpty()) {
                    throw new IllegalArgumentException(
                            "a parameter without a default follows one with a default");
                }
            } else {
                try {
                    converted.add(CommandArgs.convert(text, types[i].getName()));
                } catch (IllegalArgumentException e) {
                    throw new IllegalArgumentException(
                            "default of parameter " + i + ": " + e.getMessage(), e);
                }
            }
        }
        return new Overload(method, texts, converted.toArray());
    }

    private final Object host;
    private final Commands commands;

    public SubsystemCommandBinding(Subsystem subsystem) {
        this(subsystem, subsystemCommands());
    }

    public SubsystemCommandBinding(Object host, Commands commands) {
        if (host == null) throw new IllegalArgumentException("host == null");
        if (commands == null) throw new IllegalArgumentException("commands == null");
        this.host = host;
        this.commands = commands;
    }

    @Override
    public boolean submit(String name, Object[] args) {
        Command command = commands.byName.get(name);
        if (command == null) return false;
        Object[] values = args == null ? new Object[0] : args;
        Overload overload = command.byArity.get(values.length);
        if (overload == null) return false;

        try {
            Scheduler<?> scheduler = (Scheduler<?>) command.field.get(host);
            Object draft = scheduler.schedule();
            overload.method.invoke(draft, overload.complete(values));
            ((ScheduleApi<?>) draft).execute();
            return true;
        } catch (InvocationTargetException e) {
            Throwable cause = e.getCause();
            if (cause instanceof RuntimeException) throw (RuntimeException) cause;
            throw new IllegalStateException("Command failed: " + name, cause);
        } catch (IllegalAccessException e) {
            throw new IllegalStateException("Cannot run command: " + name, e);
        }
    }

    private static boolean allParametersSupported(Method method) {
        for (Class<?> type : method.getParameterTypes()) {
            if (!SUPPORTED.contains(type)) return false;
        }
        return true;
    }

    private static List<String> parameterTypes(Method method) {
        List<String> types = new ArrayList<>();
        for (Class<?> type : method.getParameterTypes()) types.add(type.getName());
        return types;
    }

    private static List<String> parameterNames(Method method) {
        List<String> names = new ArrayList<>();
        for (int i = 0; i < method.getParameterTypes().length; i++) {
            Param param = paramAnnotation(method, i);
            String name = param == null ? null : param.value();
            names.add(name == null || name.isEmpty() ? "arg" + i : name);
        }
        return names;
    }

    private static Param paramAnnotation(Method method, int index) {
        for (Annotation annotation : method.getParameterAnnotations()[index]) {
            if (annotation instanceof Param) return (Param) annotation;
        }
        return null;
    }

    /** Resolves {@code A} of a {@code Scheduler<A>} field type, including subclasses. */
    private static Class<?> scheduleApiType(Type type) {
        Map<Type, Type> seen = new HashMap<>();
        Type current = type;
        while (current != null && seen.put(current, current) == null) {
            if (current instanceof ParameterizedType) {
                ParameterizedType parameterized = (ParameterizedType) current;
                if (parameterized.getRawType() == Scheduler.class) {
                    Type arg = parameterized.getActualTypeArguments()[0];
                    return arg instanceof Class ? (Class<?>) arg : null;
                }
                current = parameterized.getRawType();
            } else if (current instanceof Class) {
                current = ((Class<?>) current).getGenericSuperclass();
            } else {
                return null;
            }
        }
        return null;
    }
}
