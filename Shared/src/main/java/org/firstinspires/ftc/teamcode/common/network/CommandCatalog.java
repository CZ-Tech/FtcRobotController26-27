package org.firstinspires.ftc.teamcode.common.network;

import org.json.JSONArray;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
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
    /** One accepted invocation, including its own overload's parameter names. */
    public static final class Signature {
        public final List<String> parameterNames;
        public final List<String> parameterTypes;
        /** Explicit defaults only; null never means an inferred overload default. */
        public final List<String> parameterDefaults;

        public Signature(List<String> names, List<String> types, List<String> defaults) {
            if (names.size() != types.size()) {
                throw new IllegalArgumentException("Parameter names/types must have equal lengths");
            }
            parameterNames = Collections.unmodifiableList(new ArrayList<>(names));
            parameterTypes = Collections.unmodifiableList(new ArrayList<>(types));
            List<String> values = new ArrayList<>();
            for (int i = 0; i < names.size(); i++) {
                values.add(defaults == null || i >= defaults.size() ? null : defaults.get(i));
            }
            parameterDefaults = Collections.unmodifiableList(values);
        }
    }

    public static final class Descriptor {
        public final String name;
        /** Legacy fields describing the longest accepted signature. */
        public final List<String> parameterNames;
        public final List<String> parameterTypes;
        /** Explicit defaults of the longest signature; null means no known default. */
        public final List<String> parameterDefaults;
        /** Every accepted invocation, sorted by argument count. */
        public final List<Signature> signatures;
        public final List<Integer> acceptedArgCounts;
        /** True only for continuous arities with identical name/type prefixes. */
        public final boolean hasUnifiedSignature;
        /** Null when signatures must be presented separately. */
        public final Integer requiredParamCount;
        /** Required flags for the longest signature, or null when it cannot be unified. */
        public final List<Boolean> parameterRequired;

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
         * Compatibility constructor: shorter signatures use the longest signature's
         * name prefixes. Use {@link #fromSignatures} for real overloads with distinct names.
         * Defaults are explicit metadata, never inferred from an overload's method body.
         */
        public Descriptor(String name,
                          List<String> parameterNames,
                          List<String> parameterTypes,
                          List<String> parameterDefaults,
                          List<List<String>> acceptedTypes) {
            this(name, legacySignatures(parameterNames, parameterTypes,
                    parameterDefaults, acceptedTypes));
        }

        public static Descriptor fromSignatures(String name, List<Signature> signatures) {
            return new Descriptor(name, signatures);
        }

        private Descriptor(String name, List<Signature> accepted) {
            if (accepted.isEmpty()) throw new IllegalArgumentException("No accepted signatures");
            this.name = name;
            List<Signature> sorted = new ArrayList<>(accepted);
            Collections.sort(sorted, new Comparator<Signature>() {
                @Override public int compare(Signature a, Signature b) {
                    return Integer.compare(a.parameterTypes.size(), b.parameterTypes.size());
                }
            });
            signatures = Collections.unmodifiableList(sorted);
            Signature longest = sorted.get(sorted.size() - 1);
            parameterNames = longest.parameterNames;
            parameterTypes = longest.parameterTypes;
            parameterDefaults = longest.parameterDefaults;

            List<Integer> counts = new ArrayList<>();
            boolean unified = true;
            int first = sorted.get(0).parameterTypes.size();
            for (Signature signature : sorted) {
                int count = signature.parameterTypes.size();
                if (!counts.isEmpty() && count == counts.get(counts.size() - 1)) {
                    throw new IllegalArgumentException("Duplicate accepted argument count: " + count);
                }
                if (count != first + counts.size()
                        || !parameterNames.subList(0, count).equals(signature.parameterNames)
                        || !parameterTypes.subList(0, count).equals(signature.parameterTypes)) {
                    unified = false;
                }
                counts.add(count);
            }
            acceptedArgCounts = Collections.unmodifiableList(counts);
            hasUnifiedSignature = unified;
            requiredParamCount = unified ? first : null;
            List<Boolean> required = new ArrayList<>();
            for (int i = 0; i < parameterNames.size(); i++) required.add(i < first);
            parameterRequired = unified ? Collections.unmodifiableList(required) : null;
        }

        private static List<Signature> legacySignatures(List<String> names, List<String> types,
                                                        List<String> defaults,
                                                        List<List<String>> acceptedTypes) {
            Signature longest = new Signature(names, types, defaults);
            List<Signature> result = new ArrayList<>();
            for (List<String> accepted : acceptedTypes) {
                int count = accepted.size();
                if (count > types.size()) {
                    throw new IllegalArgumentException("Accepted signature exceeds longest signature");
                }
                result.add(new Signature(names.subList(0, count), accepted,
                        longest.parameterDefaults.subList(0, count)));
            }
            return result;
        }

        /** Parameter types of the signature taking exactly {@code count} args, or null. */
        public List<String> typesForArity(int count) {
            for (Signature signature : signatures) {
                if (signature.parameterTypes.size() == count) return signature.parameterTypes;
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

    private static JSONArray defaultsJson(List<String> values) {
        JSONArray result = new JSONArray();
        for (String value : values) result.put(value == null ? JSONObject.NULL : value);
        return result;
    }

    public String toJson() {
        JSONArray commands = new JSONArray();
        for (Descriptor descriptor : descriptors.get()) {
            JSONObject item = new JSONObject();
            try {
                item.put("name", descriptor.name);
                item.put("paramNames", new JSONArray(descriptor.parameterNames));
                item.put("paramTypes", new JSONArray(descriptor.parameterTypes));
                item.put("paramDefaults", defaultsJson(descriptor.parameterDefaults));
                item.put("acceptedArgCounts", new JSONArray(descriptor.acceptedArgCounts));
                item.put("hasUnifiedSignature", descriptor.hasUnifiedSignature);
                item.put("requiredParamCount", descriptor.requiredParamCount == null
                        ? JSONObject.NULL : descriptor.requiredParamCount);
                item.put("paramRequired", descriptor.parameterRequired == null
                        ? JSONObject.NULL : new JSONArray(descriptor.parameterRequired));
                JSONArray signatures = new JSONArray();
                for (Signature signature : descriptor.signatures) {
                    JSONObject accepted = new JSONObject();
                    accepted.put("paramNames", new JSONArray(signature.parameterNames));
                    accepted.put("paramTypes", new JSONArray(signature.parameterTypes));
                    accepted.put("paramDefaults", defaultsJson(signature.parameterDefaults));
                    signatures.put(accepted);
                }
                item.put("signatures", signatures);
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
