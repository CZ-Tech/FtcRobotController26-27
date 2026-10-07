package org.firstinspires.ftc.mockrobot;

import org.firstinspires.ftc.teamcode.common.network.RouteRepository;
import org.json.JSONArray;
import org.json.JSONObject;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicLong;

/** Desktop route repository with the same revision semantics as the Android RouteStore. */
public final class MockRouteStore implements RouteRepository {
    private final TreeMap<String, Entry> routes = new TreeMap<>();
    private final AtomicLong changeRevision = new AtomicLong(1);
    private final CopyOnWriteArrayList<Runnable> listeners = new CopyOnWriteArrayList<>();
    private final Path file;

    public MockRouteStore(Path file) {
        this.file = file;
        load();
    }

    public void addListener(Runnable listener) {
        if (listener != null) listeners.add(listener);
    }

    @Override
    public synchronized List<Entry> list() {
        return Collections.unmodifiableList(new ArrayList<>(routes.values()));
    }

    @Override
    public synchronized Entry get(String name) {
        return routes.get(name);
    }

    @Override
    public synchronized PutResult put(String name, String json, Long expectedRevision) {
        validateName(name);
        if (json == null) throw new IllegalArgumentException("json == null");
        Entry old = routes.get(name);
        long oldRevision = old == null ? 0 : old.revision;
        if (expectedRevision != null && expectedRevision != oldRevision) {
            return new PutResult(false, true, old);
        }
        Entry next = new Entry(name, json, oldRevision + 1);
        routes.put(name, next);
        changed();
        return new PutResult(true, false, next);
    }

    @Override
    public synchronized boolean delete(String name, Long expectedRevision) {
        Entry old = routes.get(name);
        if (old == null) return false;
        if (expectedRevision != null && expectedRevision != old.revision) return false;
        routes.remove(name);
        changed();
        return true;
    }

    @Override
    public long changeRevision() {
        return changeRevision.get();
    }

    @Override
    public synchronized String manifestJson() {
        JSONArray array = new JSONArray();
        for (Entry entry : routes.values()) {
            array.put(new JSONObject()
                    .put("name", entry.name)
                    .put("revision", entry.revision));
        }
        return new JSONObject()
                .put("routes", array)
                .put("changeRevision", changeRevision.get())
                .toString();
    }

    public synchronized void replaceWithDemoRoutes() {
        routes.clear();
        putDirect("Mock Straight", DemoRoutes.straight());
        putDirect("Mock Spline", DemoRoutes.spline());
        putDirect("Mock Figure Eight", DemoRoutes.figureEight());
        changed();
    }

    public synchronized void seedIfEmpty() {
        if (routes.isEmpty()) replaceWithDemoRoutes();
    }

    private void putDirect(String name, String json) {
        Entry old = routes.get(name);
        long next = old == null ? 1 : old.revision + 1;
        routes.put(name, new Entry(name, json, next));
    }

    private void changed() {
        changeRevision.incrementAndGet();
        save();
        for (Runnable listener : listeners) listener.run();
    }

    private synchronized void save() {
        try {
            Files.createDirectories(file.getParent());
            JSONObject root = new JSONObject();
            for (Map.Entry<String, Entry> item : routes.entrySet()) {
                root.put(item.getKey(), new JSONObject()
                        .put("revision", item.getValue().revision)
                        .put("json", item.getValue().json));
            }
            Files.writeString(file, root.toString(2), StandardCharsets.UTF_8);
        } catch (IOException e) {
            MockLog.error("Routes", "Failed to persist routes", e);
        }
    }

    private synchronized void load() {
        if (!Files.isRegularFile(file)) return;
        try {
            JSONObject root = new JSONObject(Files.readString(file, StandardCharsets.UTF_8));
            for (String name : root.keySet()) {
                JSONObject item = root.getJSONObject(name);
                routes.put(name, new Entry(
                        name,
                        item.getString("json"),
                        Math.max(1, item.optLong("revision", 1))));
            }
        } catch (Exception e) {
            MockLog.error("Routes", "Ignoring invalid persisted route database", e);
            routes.clear();
        }
    }

    private static void validateName(String name) {
        if (name == null || name.trim().isEmpty()) {
            throw new IllegalArgumentException("route name is empty");
        }
        if (name.length() > 128 || name.indexOf('\0') >= 0) {
            throw new IllegalArgumentException("invalid route name");
        }
    }
}
