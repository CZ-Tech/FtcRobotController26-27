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
 * <p>Binding/invocation remains deliberately absent until the main-thread integration
 * phase. Network code can describe commands and enqueue typed argument data only.</p>
 */
public final class CommandCatalog {
    public static final class Descriptor {
        public final String name;
        public final List<String> parameterNames;
        public final List<String> parameterTypes;

        public Descriptor(String name, List<String> parameterNames, List<String> parameterTypes) {
            this.name = name;
            this.parameterNames = Collections.unmodifiableList(new ArrayList<>(parameterNames));
            this.parameterTypes = Collections.unmodifiableList(new ArrayList<>(parameterTypes));
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
