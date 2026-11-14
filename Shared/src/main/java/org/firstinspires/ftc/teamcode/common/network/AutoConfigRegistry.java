package org.firstinspires.ftc.teamcode.common.network;

import org.json.JSONArray;
import org.json.JSONObject;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Runtime-only configuration editor. Fields themselves hold the effective values;
 * defaults are captured once on process initialization, never persisted.
 */
public final class AutoConfigRegistry {
    private static final class Entry {
        final String key;
        final String group;
        final Field field;
        final Object initial;

        Entry(String key, String group, Field field) throws IllegalAccessException {
            this.key = key;
            this.group = group;
            this.field = field;
            this.initial = field.get(null);
        }
    }

    private final Map<String, Entry> entries = new LinkedHashMap<>();
    private long revision = 1;

    /** Searches nested classes recursively. The annotation is the only opt-in. */
    public AutoConfigRegistry(Class<?> root) {
        discover(root);
    }

    private void discover(Class<?> type) {
        AutoConfig annotation = type.getAnnotation(AutoConfig.class);
        if (annotation != null) {
            String group = annotation.value().isEmpty() ? type.getSimpleName() : annotation.value();
            for (Field field : type.getDeclaredFields()) {
                int m = field.getModifiers();
                if (!Modifier.isPublic(m) || !Modifier.isStatic(m) || Modifier.isFinal(m)
                        || field.isSynthetic()) continue;
                if (!supported(field.getType())) continue;
                String key = type.getSimpleName() + "." + field.getName();
                try {
                    if (entries.put(key, new Entry(key, group, field)) != null) {
                        throw new IllegalStateException("Duplicate tuning key: " + key);
                    }
                } catch (IllegalAccessException e) {
                    throw new IllegalStateException("Inaccessible tuning field " + key, e);
                }
            }
        }
        for (Class<?> child : type.getDeclaredClasses()) discover(child);
    }

    private static boolean supported(Class<?> type) {
        return type == double.class || type == float.class || type == int.class
                || type == long.class || type == boolean.class || type == String.class
                || type.isEnum();
    }

    public synchronized long revision() { return revision; }

    public synchronized String snapshotJson() {
        JSONObject root = new JSONObject();
        JSONArray fields = new JSONArray();
        try {
            root.put("revision", revision);
            for (Entry entry : entries.values()) {
                JSONObject item = new JSONObject();
                item.put("name", entry.key);
                item.put("group", entry.group);
                item.put("type", typeName(entry.field.getType()));
                item.put("value", jsonValue(entry.field.get(null)));
                item.put("defaultValue", jsonValue(entry.initial));
                if (entry.field.getType().isEnum()) {
                    JSONArray options = new JSONArray();
                    for (Object constant : entry.field.getType().getEnumConstants()) {
                        options.put(((Enum<?>) constant).name());
                    }
                    item.put("options", options);
                }
                fields.put(item);
            }
            root.put("fields", fields);
        } catch (Exception e) {
            throw new IllegalStateException("Could not read tuning", e);
        }
        return root.toString();
    }

    private static Object jsonValue(Object value) {
        return value instanceof Enum ? ((Enum<?>) value).name() : value;
    }

    private static String typeName(Class<?> type) {
        return type.isEnum() ? "enum" : type.getSimpleName();
    }

    /** Validate the whole request before changing any field. */
    public synchronized void patch(long expectedRevision, JSONObject updates) {
        checkRevision(expectedRevision);
        if (updates == null || updates.length() == 0) {
            throw new IllegalArgumentException("No tuning values supplied");
        }
        Map<Entry, Object> resolved = new LinkedHashMap<>();
        for (java.util.Iterator<String> it = updates.keys(); it.hasNext();) {
            String key = it.next();
            Entry entry = entries.get(key);
            if (entry == null) throw new IllegalArgumentException("Unknown tuning field: " + key);
            resolved.put(entry, convert(entry.field.getType(), updates.opt(key), key));
        }
        try {
            for (Map.Entry<Entry, Object> update : resolved.entrySet()) {
                update.getKey().field.set(null, update.getValue());
            }
        } catch (IllegalAccessException e) {
            throw new IllegalStateException("Tuning write failed", e);
        }
        revision++;
    }

    public synchronized void reset(long expectedRevision) {
        checkRevision(expectedRevision);
        try {
            for (Entry entry : entries.values()) entry.field.set(null, entry.initial);
        } catch (IllegalAccessException e) {
            throw new IllegalStateException("Tuning reset failed", e);
        }
        revision++;
    }

    private void checkRevision(long expected) {
        if (revision != expected) throw new RevisionConflict(revision);
    }

    public static final class RevisionConflict extends RuntimeException {
        public final long actual;
        RevisionConflict(long actual) { this.actual = actual; }
    }

    private static Object convert(Class<?> type, Object raw, String name) {
        if (raw == null || raw == JSONObject.NULL) {
            throw new IllegalArgumentException(name + " cannot be null");
        }
        if (type == boolean.class) {
            if (raw instanceof Boolean) return raw;
        } else if (type == String.class) {
            if (raw instanceof String) return raw;
        } else if (type.isEnum()) {
            if (raw instanceof String) {
                for (Object c : type.getEnumConstants()) {
                    if (((Enum<?>) c).name().equals(raw)) return c;
                }
            }
        } else if (raw instanceof Number) {
            double value = ((Number) raw).doubleValue();
            if (!Double.isFinite(value)) throw new IllegalArgumentException(name + " must be finite");
            if (type == double.class) return value;
            if (type == float.class && Math.abs(value) <= Float.MAX_VALUE) return (float) value;
            if ((type == int.class || type == long.class) && Math.rint(value) == value) {
                if (type == int.class && value >= Integer.MIN_VALUE && value <= Integer.MAX_VALUE) return (int) value;
                if (type == long.class && value >= Long.MIN_VALUE && value <= Long.MAX_VALUE) return (long) value;
            }
        }
        throw new IllegalArgumentException("Invalid value/type for " + name);
    }
}
