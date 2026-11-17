package org.firstinspires.ftc.teamcode.common.network;

import org.json.JSONArray;
import org.json.JSONException;
import org.junit.Test;

import static org.junit.Assert.*;

public class RouteJsonTest {
    @Test public void bareArrayRemainsLegacyVersionOne() {
        JSONArray frames = RouteJson.frames("[{\"x\":1,\"y\":2},{\"wait\":0.5}]");
        assertEquals(2, frames.length());
        assertEquals(1, frames.getJSONObject(0).getInt("x"));
        assertEquals(0.5, frames.getJSONObject(1).getDouble("wait"), 0);
    }

    @Test public void versionTwoPreservesOrderedFramesAndCommands() {
        JSONArray frames = RouteJson.frames("{\"version\":2,\"frames\":["
                + "{\"x\":0,\"y\":0,\"commands\":["
                + "{\"name\":\"auto.waitMillis\",\"params\":[\"500\"]},"
                + "{\"name\":\"intaker.intakerArmStop\",\"params\":[]}]},"
                + "{\"wait\":1}]}");
        assertEquals(2, frames.length());
        JSONArray commands = frames.getJSONObject(0).getJSONArray("commands");
        assertEquals("auto.waitMillis", commands.getJSONObject(0).getString("name"));
        assertEquals("intaker.intakerArmStop", commands.getJSONObject(1).getString("name"));
    }

    @Test public void emptyArraysAreUnwrappedForTheCallerToValidate() {
        assertEquals(0, RouteJson.frames("[]").length());
        assertEquals(0, RouteJson.frames("{\"version\":2,\"frames\":[]}").length());
    }

    @Test public void rejectsUnknownVersionsAndInvalidEnvelopes() {
        String[] invalid = {
                "{}", "{\"frames\":[]}", "{\"version\":1,\"frames\":[]}",
                "{\"version\":3,\"frames\":[]}", "{\"version\":2.5,\"frames\":[]}",
                "{\"version\":\"2\",\"frames\":[]}", "{\"version\":true,\"frames\":[]}",
                "{\"version\":null,\"frames\":[]}", "{\"version\":2}",
                "{\"version\":2,\"frames\":null}", "{\"version\":2,\"frames\":{}}",
                "{\"version\":2,\"frames\":\"[]\"}", "null", "true", "2", "\"[]\""
        };
        for (String json : invalid) {
            assertThrows(json, JSONException.class, () -> RouteJson.frames(json));
        }
    }

    @Test public void rejectsTrailingJsonAndNullInput() {
        assertThrows(JSONException.class, () -> RouteJson.frames("[] []"));
        assertThrows(JSONException.class, () -> RouteJson.frames("{\"version\":2,\"frames\":[]} true"));
        assertThrows(JSONException.class, () -> RouteJson.frames(null));
        assertThrows(JSONException.class, () -> RouteJson.frames(""));
    }
}
