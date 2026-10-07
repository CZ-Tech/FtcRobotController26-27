package org.firstinspires.ftc.mockrobot;

import org.json.JSONArray;
import org.json.JSONObject;

/** Built-in routes used when the simulator is started for the first time. */
public final class DemoRoutes {
    private DemoRoutes() {}

    public static String straight() {
        return new JSONArray()
                .put(node(-55, -24, 45, 0, 90, 0, 1.0, "Start"))
                .put(node(0, -24, 55, 0, 90, 0, 2.5, "Middle"))
                .put(node(55, -24, 45, 0, 90, 0, 2.5, "Finish"))
                .toString();
    }

    public static String spline() {
        return new JSONArray()
                .put(node(-58, -50, 40, 10, 90, 0, 1.0, "Start"))
                .put(node(-15, -5, 35, 45, 45, -20, 2.3, "Curve 1"))
                .put(node(20, 35, 40, 10, 5, -15, 2.2, "Curve 2"))
                .put(node(58, 48, 35, 0, 0, 0, 2.0, "Finish"))
                .toString();
    }

    public static String figureEight() {
        return new JSONArray()
                .put(node(-45, 0, 35, 45, 45, 0, 1.0, "Start"))
                .put(node(0, 42, 45, 0, 90, 0, 2.0, "Top"))
                .put(node(45, 0, 35, -45, 135, 0, 2.0, "Right"))
                .put(node(0, -42, -45, 0, -90, 0, 2.0, "Bottom"))
                .put(node(-45, 0, -35, 45, -135, 0, 2.0, "Finish"))
                .toString();
    }

    private static JSONObject node(
            double x,
            double y,
            double dx,
            double dy,
            double heading,
            double dHeading,
            double duration,
            String marker) {
        return new JSONObject()
                .put("x", x)
                .put("dx", dx)
                .put("y", y)
                .put("dy", dy)
                .put("heading", heading)
                .put("dHeading", dHeading)
                .put("duration", duration)
                .put("marker", marker)
                .put("command", "")
                .put("commandParams", new JSONArray())
                .put("delayAfterArrive", 0.0);
    }
}
