package org.firstinspires.ftc.teamcode.common.network;

import org.json.JSONArray;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Hardware-free command metadata catalog.
 *
 * <p>Invocation is deliberately absent: network code describes commands and validates
 * argument data here, then hands typed arguments to a {@link CommandDispatcher}.</p>
 */
public final class CommandCatalog {
    public static final class Descriptor {
        public final String name;
        /** Names of the longest accepted signature; this is what {@code GET /commands} lists. */
        public final List<String> parameterNames;
        /** Types of the longest accepted signature. */
        public final List<String> parameterTypes;
        /**
         * Default value text per parameter of the longest signature, {@code null} for a
         * required parameter. Defaulted parameters are always the trailing ones.
         */
        public final List<String> parameterDefaults;
        private final List<List<String>> acceptedTypes;

        public Descriptor(String name, List<String> parameterNames, List<String> parameterTypes) {
            this(name, parameterNames, parameterTypes,
                    Collections.singletonList(parameterTypes));
        }

        public Descriptor(String name,
                          List<String> parameterNames,
                          List<String> parameterTypes,
                          List<List<String>> acceptedTypes) {
            this(name, parameterNames, parameterTypes, null, acceptedTypes);
        }

        /**
         * @param parameterDefaults default text per parameter ({@code null} entries are
         *                          required parameters); {@code null} means no defaults
         * @param acceptedTypes     every signature the command accepts, one type list per
         *                          distinct argument count (overloads and omitted trailing
         *                          defaults). The published {@code parameterTypes} should be
         *                          the longest of them.
         */
        public Descriptor(String name,
                          List<String> parameterNames,
                          List<String> parameterTypes,
                          List<String> parameterDefaults,
                          List<List<String>> acceptedTypes) {
            this.name = name;
            this.parameterNames = Collections.unmodifiableList(new ArrayList<>(parameterNames));
            this.parameterTypes = Collections.unmodifiableList(new ArrayList<>(parameterTypes));
            List<String> defaults = new ArrayList<>();
            for (int i = 0; i < parameterNames.size(); i++) {
                defaults.add(parameterDefaults == null || i >= parameterDefaults.size()
                        ? null
                        : parameterDefaults.get(i));
            }
            this.parameterDefaults = Collections.unmodifiableList(defaults);
            List<List<String>> copy = new ArrayList<>();
            for (List<String> types : acceptedTypes) {
                copy.add(Collections.unmodifiableList(new ArrayList<>(types)));
            }
            this.acceptedTypes = Collections.unmodifiableList(copy);
        }

        /** Parameter types of the signature taking exactly {@code count} args, or null. */
        public List<String> typesForArity(int count) {
            for (List<String> types : acceptedTypes) {
                if (types.size() == count) return types;
            }
            return null;
        }
    }

    private final AtomicReference<List<Descriptor>> descriptors =
            new AtomicReference<>(Collections.emptyList());
    private final AtomicLong revision = new AtomicLong(1);

    public void publish(List<Descriptor> value) {
        descriptors.set(Collections.unmodifiableList(new ArrayList<>(value)));
        revision.incrementAndGet();
    }

    public List<Descriptor> list() {
        return descriptors.get();
    }

    /** Exact-name lookup in the currently published catalog; null when absent. */
    public Descriptor find(String name) {
        if (name == null) return null;
        for (Descriptor descriptor : descriptors.get()) {
            if (descriptor.name.equals(name)) return descriptor;
        }
        return null;
    }

    public long revision() {
        return revision.get();
    }

    public String toJson() {
        JSONArray commands = new JSONArray();
        for (Descriptor descriptor : descriptors.get()) {
            JSONObject item = new JSONObject();
            try {
                item.put("name", descriptor.name);
                item.put("paramNames", new JSONArray(descriptor.parameterNames));
                item.put("paramTypes", new JSONArray(descriptor.parameterTypes));
                JSONArray defaults = new JSONArray();
                for (String value : descriptor.parameterDefaults) {
                    defaults.put(value == null ? JSONObject.NULL : value);
                }
                item.put("paramDefaults", defaults);
                commands.put(item);
            } catch (Exception ignored) {}
        }
        JSONObject root = new JSONObject();
        try {
            root.put("revision", revision.get());
            root.put("commands", commands);
        } catch (Exception ignored) {}
        return root.toString();
    }
}
