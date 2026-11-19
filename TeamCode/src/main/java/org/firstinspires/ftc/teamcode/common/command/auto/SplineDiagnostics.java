package org.firstinspires.ftc.teamcode.common.command.auto;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.command.auto.spline.SplineController;
import org.firstinspires.ftc.teamcode.common.drive.OdoDrivetrain;
import org.firstinspires.ftc.teamcode.common.network.RobotRuntimeStore;

import java.util.Locale;

/** One route's Logcat trace. All timestamps use the same monotonic clock. */
public final class SplineDiagnostics {
    private static final String TAG = "SplineDiag";
    private static final long SAMPLE_INTERVAL_NS = 50_000_000L;

    private final boolean enabled;
    private final long runId = System.nanoTime();
    private long lastControlNs, lastPoseNs;
    private int tick;
    private boolean controlLogged, poseLogged;
    private boolean brakeZone, reverse, capture;

    public SplineDiagnostics(boolean enabled) { this.enabled = enabled; }

    public boolean enabled() { return enabled; }

    public void event(String kind, long timeNs, String detail) {
        if (enabled) Log.i(TAG, kind + " run=" + runId + " tNs=" + timeNs + " " + detail);
    }

    void newSegment() {
        tick = 0;
        controlLogged = false;
    }

    void control(long timeNs, int segment, double observedDt, SplineController.Input in,
                 SplineController.Output out, Pose2D raw, OdoDrivetrain.DriveTrace drive,
                 String terminalReason) {
        if (!enabled) return;
        tick++;
        if (controlLogged && terminalReason == null && timeNs - lastControlNs < SAMPLE_INTERVAL_NS
                && brakeZone == out.brakingZone && reverse == out.brakeReverseRequested
                && capture == out.stopCapture) {
            return;
        }
        controlLogged = true;
        lastControlNs = timeNs;
        brakeZone = out.brakingZone;
        reverse = out.brakeReverseRequested;
        capture = out.stopCapture;
        double tx = out.projection.tx;
        double ty = out.projection.ty;
        double vNormal = -in.vx * ty + in.vy * tx;
        String applied = drive == null ? "drive=NOT_SENT" : String.format(Locale.ROOT,
                "mixHeadingRad=%.6f bodyFRYaw=(%.4f,%.4f,%.4f) norm=%.4f wheelLFRFLBRB=(%.4f,%.4f,%.4f,%.4f)",
                drive.headingRad, drive.forward, drive.right, drive.yaw, drive.denominator,
                drive.lf, drive.rf, drive.lb, drive.rb);
        event("CTRL", timeNs, String.format(Locale.ROOT,
                "seg=%d tick=%d result=%s poseInDeg=(%.4f,%.4f,%.4f) rawInDeg=%s"
                        + " dtObservedS=%.6f dtUsedS=%.6f vXYInS=(%.3f,%.3f) vTNInS=(%.3f,%.3f)"
                        + " uPrev=%.6f u=%.6f pathXYIn=(%.4f,%.4f) tangentXY=(%.5f,%.5f)"
                        + " sProjectedIn=%.4f sProgressIn=%.4f remainingIn=%.4f triggerIn=%.4f"
                        + " crossErrIn=%.4f tangentErrIn=%.4f endErrIn=%.4f"
                        + " brakeZone=%b reverseRequested=%b stopCapture=%b projectionFault=%b"
                        + " requestedTN=(%.4f,%.4f) fieldCmdXYYaw=(%.4f,%.4f,%.4f)"
                        + " targetHDeg=%.4f headingErrDeg=%.4f voltage=%.3f %s",
                segment, tick, terminalReason == null ? "RUNNING" : terminalReason,
                in.x, in.y, in.heading, pose(raw), observedDt, in.dt, in.vx, in.vy,
                out.tangentialVelocity, vNormal, out.previousU, out.closestU,
                out.pathX, out.pathY, tx, ty, out.projection.s, out.progressS, out.remainingArc,
                out.brakeTriggerDistance, out.crossError, out.projectionTangentError,
                out.endPositionError, brakeZone, reverse, capture, out.projectionFault,
                out.candidateTangentPower, out.candidateCrossPower, out.powerX, out.powerY,
                out.yawPower, out.targetHeading, out.headingError, in.voltage, applied));
    }

    /** Logs the actual source snapshot of the pose event, rather than re-reading or projecting it. */
    public void publishedPose(RobotRuntimeStore.Snapshot snapshot, Pose2D raw, int frame) {
        if (!enabled || (snapshot.opModeActive && poseLogged
                && snapshot.timestampNanos - lastPoseNs < SAMPLE_INTERVAL_NS)) return;
        poseLogged = true;
        lastPoseNs = snapshot.timestampNanos;
        event(snapshot.opModeActive ? "POSE" : "POSE_FINAL", snapshot.timestampNanos,
                String.format(Locale.ROOT,
                        "frame=%d seq=%d poseInDeg=(%.4f,%.4f,%.4f) rawInDeg=%s",
                        frame, snapshot.sequence, snapshot.x, snapshot.y, snapshot.heading, pose(raw)));
    }

    static String pose(Pose2D pose) {
        if (pose == null) return "unavailable";
        return String.format(Locale.ROOT, "(%.4f,%.4f,%.4f)", pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.DEGREES));
    }
}
