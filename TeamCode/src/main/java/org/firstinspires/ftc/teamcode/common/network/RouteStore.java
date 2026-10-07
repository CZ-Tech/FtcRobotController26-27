package org.firstinspires.ftc.teamcode.common.network;

import android.content.Context;
import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import org.json.JSONArray;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.concurrent.atomic.AtomicLong;

/**
 * Atomic named-route persistence for Network V2.
 *
 * <p>Each PUT writes the complete route body and revision together from the caller's
 * perspective. There is deliberately no shared "current JSON" staging slot.</p>
 */
public final class RouteStore implements RouteRepository {
    private static final String JSON_PREFIX = "az_v2_route_json_";
    private static final String REV_PREFIX = "az_v2_route_rev_";

    private final SharedPreferences preferences;
    private final TreeMap<String, RouteRepository.Entry> routes = new TreeMap<>();
    private final AtomicLong changeRevision = new AtomicLong(1);

    public RouteStore(Context context) {
        if (context == null) throw new IllegalArgumentException("context == null");
        preferences = PreferenceManager.getDefaultSharedPreferences(
                context.getApplicationContext());
        load();
    }

    @Override
    public synchronized List<RouteRepository.Entry> list() {
        return Collections.unmodifiableList(new ArrayList<>(routes.values()));
    }

    @Override
    public synchronized RouteRepository.Entry get(String name) {
        return routes.get(name);
    }

    /**
     * @param expectedRevision null means unconditional write. 0 means "create only".
     */
    @Override
    public synchronized RouteRepository.PutResult put(
            String name,
            String json,
            Long expectedRevision) {
        validateName(name);
        if (json == null) throw new IllegalArgumentException("json == null");

        RouteRepository.Entry old = routes.get(name);
        long oldRevision = old == null ? 0 : old.revision;
        if (expectedRevision != null && expectedRevision != oldRevision) {
            return new RouteRepository.PutResult(false, true, old);
        }

        long nextRevision = oldRevision + 1;
        RouteRepository.Entry next = new RouteRepository.Entry(name, json, nextRevision);
        preferences.edit()
                .putString(JSON_PREFIX + name, json)
                .putLong(REV_PREFIX + name, nextRevision)
                .commit();
        routes.put(name, next);
        changeRevision.incrementAndGet();
        return new RouteRepository.PutResult(true, false, next);
    }

    @Override
    public synchronized boolean delete(String name, Long expectedRevision) {
        RouteRepository.Entry old = routes.get(name);
        if (old == null) return false;
        if (expectedRevision != null && expectedRevision != old.revision) return false;

        preferences.edit()
                .remove(JSON_PREFIX + name)
                .remove(REV_PREFIX + name)
                .commit();
        routes.remove(name);
        changeRevision.incrementAndGet();
        return true;
    }

    /** Monotonic process-local signal used by SSE to announce manifest changes. */
    @Override
    public long changeRevision() {
        return changeRevision.get();
    }

    @Override
    public synchronized String manifestJson() {
        JSONArray array = new JSONArray();
        for (RouteRepository.Entry entry : routes.values()) {
            JSONObject item = new JSONObject();
            try {
                item.put("name", entry.name);
                item.put("revision", entry.revision);
                array.put(item);
            } catch (Exception ignored) {}
        }
        JSONObject result = new JSONObject();
        try {
            result.put("routes", array);
            result.put("changeRevision", changeRevision.get());
        } catch (Exception ignored) {}
        return result.toString();
    }

    private void load() {
        Map<String, ?> all = preferences.getAll();
        for (Map.Entry<String, ?> pref : all.entrySet()) {
            String key = pref.getKey();
            if (!key.startsWith(JSON_PREFIX) || !(pref.getValue() instanceof String)) continue;
            String name = key.substring(JSON_PREFIX.length());
            String json = (String) pref.getValue();
            long revision = preferences.getLong(REV_PREFIX + name, 1);
            routes.put(
                    name,
                    new RouteRepository.Entry(name, json, Math.max(1, revision)));
        }
    }

    private static void validateName(String name) {
        if (name == null || name.trim().isEmpty()) {
            throw new IllegalArgumentException("route name is empty");
        }
        if (name.length() > 128) throw new IllegalArgumentException("route name too long");
        if (name.indexOf('\0') >= 0) throw new IllegalArgumentException("invalid route name");
    }
}
