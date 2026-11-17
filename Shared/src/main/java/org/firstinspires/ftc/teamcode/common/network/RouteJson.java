package org.firstinspires.ftc.teamcode.common.network;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import org.json.JSONTokener;

/** Versioned route envelope shared by HTTP, the robot loader and desktop preview. */
public final class RouteJson {
    public static final int CURRENT_VERSION = 2;

    private RouteJson() {}

    /**
     * A bare frame array is legacy v1. New routes use {"version":2,"frames":[...]}.
     * Never reinterpret an unknown object/version as a legacy route.
     */
    public static JSONArray frames(String json) throws JSONException {
        if (json == null) throw new JSONException("route JSON is null");
        JSONTokener tokener = new JSONTokener(json);
        Object root = tokener.nextValue();
        if (tokener.nextClean() != 0) throw new JSONException("trailing route JSON content");
        if (root instanceof JSONArray) return (JSONArray) root;
        if (!(root instanceof JSONObject)) {
            throw new JSONException("route must be a legacy array or a versioned object");
        }
        JSONObject envelope = (JSONObject) root;
        Object version = envelope.opt("version");
        if (!(version instanceof Number)
                || ((Number) version).doubleValue() != CURRENT_VERSION) {
            throw new JSONException("unsupported route version (expected numeric 2)");
        }
        Object frames = envelope.opt("frames");
        if (!(frames instanceof JSONArray)) {
            throw new JSONException("version 2 route requires a frames array");
        }
        return (JSONArray) frames;
    }
}
