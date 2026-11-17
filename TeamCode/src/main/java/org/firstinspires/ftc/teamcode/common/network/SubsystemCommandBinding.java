package org.firstinspires.ftc.teamcode.common.network;

import android.util.Log;

import org.firstinspires.ftc.teamcode.common.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.CommandTasks;
import java.util.concurrent.CancellationException;
import org.firstinspires.ftc.teamcode.common.util.Param;

import java.lang.annotation.Annotation;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;

/**
 * Exposes public instance {@link RemoteCommand} methods on a host's public instance
 * fields, named {@code <field>.<method>}. Metadata is scanned without constructing a host.
 * Only void methods with supported scalar parameters are published. Overloads must have
 * disjoint accepted argument counts, including omitted trailing {@link Param} defaults.
 *
 * <p>{@link #submit} snapshots arguments and starts one managed task by default.
 * {@link #submitAll} executes a whole waypoint list sequentially in one task; separate
 * submissions may overlap. Direct Java calls inside a command remain ordinary calls.
 * A stopped task owner may reject with {@link IllegalStateException}, in which case
 * submission returns false. Accepted does not mean completed.</p>
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
     * One annotated method together with its trailing default values. It is
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
            List<CommandCatalog.Signature> accepted = new ArrayList<>();
            for (Map.Entry<Integer, Overload> entry : byArity.entrySet()) {
                Overload overload = entry.getValue();
                int count = entry.getKey();
                accepted.add(new CommandCatalog.Signature(
                        parameterNames(overload.method).subList(0, count),
                        parameterTypes(overload.method).subList(0, count),
                        overload.defaultTexts.subList(0, count)));
            }
            return CommandCatalog.Descriptor.fromSignatures(name, accepted);
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

    /** Scans public instance fields for annotated command methods. */
    public static Commands scan(Class<?> hostType) {
        Map<String, Command> commands = new LinkedHashMap<>();
        Set<String> conflicted = new HashSet<>();

        for (Field field : hostType.getFields()) {
            if (Modifier.isStatic(field.getModifiers())) continue;
            for (Method method : field.getType().getMethods()) {
                if (!method.isAnnotationPresent(RemoteCommand.class)
                        || Modifier.isStatic(method.getModifiers())
                        || method.getReturnType() != void.class) continue;
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
    private final Consumer<Runnable> execute;

    public SubsystemCommandBinding(Subsystem subsystem) {
        this(subsystem, subsystemCommands(), subsystem::execute);
    }

    public SubsystemCommandBinding(Object host, Commands commands, Consumer<Runnable> execute) {
        if (host == null) throw new IllegalArgumentException("host == null");
        if (commands == null) throw new IllegalArgumentException("commands == null");
        if (execute == null) throw new IllegalArgumentException("execute == null");
        this.execute = execute;
        this.host = host;
        this.commands = commands;
    }

    @Override
    public boolean submit(String name, Object[] args) {
        if (name == null) return false;
        return submitAll(Collections.singletonList(new CommandDispatcher.Call(name, args == null ? new Object[0] : args)));
    }

    /** Validate/capture every call first, then run the whole list on one managed thread. */
    public boolean submitAll(List<CommandDispatcher.Call> calls) {
        if (calls == null) return false;
        List<Runnable> prepared = new ArrayList<>();
        for (CommandDispatcher.Call call : calls) {
            if (call == null) return false;
            Command command = commands.byName.get(call.name);
            if (command == null) return false;
            Object[] values = call.arguments();
            Overload overload = command.byArity.get(values.length);
            if (overload == null) return false;
            final Object[] captured = overload.complete(values);
            final Object receiver;
            try {
                receiver = command.field.get(host);
            } catch (IllegalAccessException e) {
                throw new IllegalStateException("Cannot read command target: " + call.name, e);
            }
            if (receiver == null) return false;
            prepared.add(() -> invoke(call.name, overload.method, receiver, captured));
        }
        if (prepared.isEmpty()) return true;
        try {
            execute.accept(() -> {
                for (Runnable invocation : prepared) {
                    CommandTasks.checkCancelled();
                    invocation.run();
                }
            });
            return true;
        } catch (IllegalStateException rejected) {
            return false;
        }
    }

    private static void invoke(String name, Method method, Object receiver, Object[] args) {
        try {
            method.invoke(receiver, args);
        } catch (InvocationTargetException e) {
            Throwable cause = e.getCause();
            if (cause instanceof InterruptedException) {
                Thread.currentThread().interrupt();
                throw new CancellationException("Command interrupted: " + name);
            }
            if (cause instanceof RuntimeException) throw (RuntimeException) cause;
            if (cause instanceof Error) throw (Error) cause;
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

}
