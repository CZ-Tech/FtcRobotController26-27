package org.firstinspires.ftc.teamcode.common.util;

import android.util.Log;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/**
 * Legacy local-only registry used by hard-coded autonomous JSON.
 *
 * <p>This class has no HTTP/network role. It exists only so the old
 * {@code TrajectoryLoader + PinpointTrajectory} autonomous code can keep resolving
 * {@code "command"} fields inside JSON literals during the current season.</p>
 *
 * @deprecated The legacy command system will be replaced next season.
 */
@Deprecated
public final class LegacyAutoTaskRegistry {
    private static final String TAG = "LegacyAutoTask";

    private static final class Entry {
        final Object instance;
        final Method method;
        final Class<?>[] parameterTypes;

        Entry(Object instance, Method method) {
            this.instance = instance;
            this.method = method;
            this.parameterTypes = method.getParameterTypes();
        }
    }

    private static final Map<String, Entry> entries = new HashMap<>();

    private LegacyAutoTaskRegistry() {}

    /**
     * Rebuild the registry from the current Robot object graph.
     *
     * <p>Rebuilding each time avoids stale OpMode/Robot references without requiring
     * any process-wide HTTP lifecycle.</p>
     */
    public static synchronized void scanObjectTree(Object root) {
        entries.clear();
        if (root == null) return;

        Deque<Object> stack = new ArrayDeque<>();
        Set<Integer> visited = new HashSet<>();
        stack.push(root);

        while (!stack.isEmpty()) {
            Object instance = stack.pop();
            if (instance == null) continue;

            int identity = System.identityHashCode(instance);
            if (!visited.add(identity)) continue;

            Class<?> type = instance.getClass();
            registerMethods(type, instance);

            if (!isTeamCodeClass(type)) continue;

            for (Field field : allFields(type)) {
                if (Modifier.isStatic(field.getModifiers())) continue;
                try {
                    field.setAccessible(true);
                    Object value = field.get(instance);
                    if (value == null || isLeaf(value.getClass())) continue;
                    stack.push(value);
                } catch (Exception ignored) {
                    // Legacy scanner is best-effort by design.
                }
            }
        }
    }

    public static synchronized Runnable createCommandRunnable(
            String commandName,
            String[] commandParams) {
        final Entry entry = entries.get(commandName);
        if (entry == null) {
            Log.w(TAG, "Unknown legacy command: " + commandName);
            return null;
        }

        final String[] params = commandParams == null
                ? new String[0]
                : commandParams.clone();

        return () -> {
            try {
                if (params.length != entry.parameterTypes.length) {
                    Log.w(TAG, "Command '" + commandName + "' expects "
                            + entry.parameterTypes.length + " arg(s), got "
                            + params.length);
                    return;
                }

                Object[] converted = new Object[params.length];
                for (int i = 0; i < params.length; i++) {
                    converted[i] = convert(params[i], entry.parameterTypes[i]);
                }
                entry.method.invoke(entry.instance, converted);
            } catch (Exception e) {
                Log.e(TAG, "Legacy command failed: " + commandName, e);
            }
        };
    }

    private static void registerMethods(Class<?> type, Object instance) {
        boolean classAnnotated = type.isAnnotationPresent(AutoTask.class);

        for (Method method : type.getDeclaredMethods()) {
            AutoTask annotation = method.getAnnotation(AutoTask.class);
            if (!classAnnotated && annotation == null) continue;
            if (!Modifier.isPublic(method.getModifiers())) continue;

            String name = method.getName();
            if (annotation != null && !annotation.value().trim().isEmpty()) {
                name = annotation.value().trim();
            }

            method.setAccessible(true);
            if (entries.containsKey(name)) {
                Log.w(TAG, "Duplicate legacy command name '" + name
                        + "'; keeping first registration");
                continue;
            }
            entries.put(name, new Entry(instance, method));
        }
    }

    private static Field[] allFields(Class<?> type) {
        java.util.ArrayList<Field> fields = new java.util.ArrayList<>();
        Class<?> current = type;
        while (current != null && current != Object.class) {
            for (Field field : current.getDeclaredFields()) fields.add(field);
            current = current.getSuperclass();
        }
        return fields.toArray(new Field[0]);
    }

    private static boolean isTeamCodeClass(Class<?> type) {
        return type.getName().startsWith("org.firstinspires.ftc.teamcode");
    }

    private static boolean isLeaf(Class<?> type) {
        return type.isPrimitive()
                || type.isEnum()
                || type == String.class
                || Number.class.isAssignableFrom(type)
                || type == Boolean.class
                || type == Character.class;
    }

    private static Object convert(String value, Class<?> targetType) {
        if (targetType == String.class) return value;
        if (targetType == int.class || targetType == Integer.class) return Integer.parseInt(value);
        if (targetType == long.class || targetType == Long.class) return Long.parseLong(value);
        if (targetType == float.class || targetType == Float.class) return Float.parseFloat(value);
        if (targetType == double.class || targetType == Double.class) return Double.parseDouble(value);
        if (targetType == byte.class || targetType == Byte.class) return Byte.parseByte(value);
        if (targetType == short.class || targetType == Short.class) return Short.parseShort(value);
        if (targetType == boolean.class || targetType == Boolean.class) {
            return Boolean.parseBoolean(value);
        }
        if (targetType == char.class || targetType == Character.class) {
            if (value == null || value.isEmpty()) {
                throw new IllegalArgumentException("Empty value for char parameter");
            }
            return value.charAt(0);
        }
        throw new IllegalArgumentException(
                "Unsupported legacy command parameter type: " + targetType.getName());
    }
}
