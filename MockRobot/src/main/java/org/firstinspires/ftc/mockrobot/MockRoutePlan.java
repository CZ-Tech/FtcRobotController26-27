package org.firstinspires.ftc.mockrobot;

import org.json.JSONArray;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** Minimal parser/evaluator matching AzConductor's cubic Hermite route semantics. */
public final class MockRoutePlan {
    public static final class Pose {
        public final double x;
        public final double y;
        public final double heading;

        public Pose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    private static final class Node {
        final double x;
        final double y;
        final double dx;
        final double dy;
        final double heading;
        final double dHeading;
        final double duration;
        final double delay;

        Node(JSONObject json) {
            x = json.optDouble("x", 0);
            y = json.optDouble("y", 0);
            dx = json.optDouble("dx", 0);
            dy = json.optDouble("dy", 0);
            heading = json.optDouble("heading", 0);
            dHeading = json.optDouble("dHeading", 0);
            duration = Math.max(0, json.optDouble("duration", 1));
            delay = Math.max(0, json.optDouble("delayAfterArrive", 0));
        }
    }

    private final List<Node> nodes;
    private final double totalTime;

    private MockRoutePlan(List<Node> nodes) {
        this.nodes = nodes;
        double time = 0;
        for (int i = 1; i < nodes.size(); i++) {
            time += nodes.get(i).duration + nodes.get(i).delay;
        }
        totalTime = time;
    }

    public static MockRoutePlan parse(String json) {
        JSONArray array = new JSONArray(json);
        List<Node> nodes = new ArrayList<>();
        for (int i = 0; i < array.length(); i++) {
            nodes.add(new Node(array.getJSONObject(i)));
        }
        if (nodes.isEmpty()) throw new IllegalArgumentException("route has no nodes");
        return new MockRoutePlan(Collections.unmodifiableList(nodes));
    }

    public double totalTime() {
        return totalTime;
    }

    public Pose startPose() {
        Node first = nodes.get(0);
        return new Pose(first.x, first.y, first.heading);
    }

    public Pose endPose() {
        Node last = nodes.get(nodes.size() - 1);
        return new Pose(last.x, last.y, last.heading);
    }

    public Pose sample(double seconds) {
        if (nodes.size() == 1 || totalTime <= 0) return startPose();
        double t = Math.max(0, Math.min(totalTime, seconds));
        double accumulated = 0;
        for (int i = 1; i < nodes.size(); i++) {
            Node start = nodes.get(i - 1);
            Node end = nodes.get(i);
            double segmentEnd = accumulated + end.duration;
            if (t <= segmentEnd) {
                double u = end.duration <= 0 ? 1 : (t - accumulated) / end.duration;
                return interpolate(start, end, u);
            }
            double delayEnd = segmentEnd + end.delay;
            if (t <= delayEnd) return interpolate(start, end, 1);
            accumulated = delayEnd;
        }
        return endPose();
    }

    public List<Pose> polyline(int samplesPerSegment) {
        if (nodes.size() == 1) return List.of(startPose());
        List<Pose> result = new ArrayList<>();
        int samples = Math.max(4, samplesPerSegment);
        for (int i = 1; i < nodes.size(); i++) {
            Node start = nodes.get(i - 1);
            Node end = nodes.get(i);
            for (int j = 0; j <= samples; j++) {
                if (i > 1 && j == 0) continue;
                result.add(interpolate(start, end, j / (double) samples));
            }
        }
        return result;
    }

    private static Pose interpolate(Node start, Node end, double u) {
        double x = cubic(start.x, start.dx, end.x, end.dx, u);
        double y = cubic(start.y, start.dy, end.y, end.dy, u);
        double relativeHeading = normalizeRelative(start.heading, end.heading);
        double heading = cubic(
                start.heading,
                start.dHeading,
                relativeHeading,
                end.dHeading,
                u);
        return new Pose(x, y, normalizeDegrees(heading));
    }

    private static double cubic(double p0, double d0, double p1, double d1, double u) {
        double a = 2 * p0 + d0 - 2 * p1 + d1;
        double b = -3 * p0 - 2 * d0 + 3 * p1 - d1;
        return a * u * u * u + b * u * u + d0 * u + p0;
    }

    private static double normalizeRelative(double start, double end) {
        double diff = (end - start) % 360.0;
        if (diff > 180) diff -= 360;
        if (diff < -180) diff += 360;
        return start + diff;
    }

    public static double normalizeDegrees(double value) {
        double result = value % 360;
        if (result <= -180) result += 360;
        if (result > 180) result -= 360;
        return result;
    }
}
