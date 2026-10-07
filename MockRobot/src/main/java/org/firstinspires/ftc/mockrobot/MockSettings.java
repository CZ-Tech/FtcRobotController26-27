package org.firstinspires.ftc.mockrobot;

import java.util.prefs.Preferences;

/** Persistent simulator settings. All fields are safe to read from the simulation thread. */
public final class MockSettings {
    private final Preferences prefs =
            Preferences.userNodeForPackage(MockSettings.class).node("settings");

    private volatile double positionNoiseSigma = prefs.getDouble("positionNoiseSigma", 0.65);
    private volatile double headingNoiseSigma = prefs.getDouble("headingNoiseSigma", 1.25);
    private volatile double correctionSeconds = prefs.getDouble("correctionSeconds", 0.85);
    private volatile double speedScale = prefs.getDouble("speedScale", 1.0);
    private volatile int poseHz = prefs.getInt("poseHz", 60);
    private volatile int httpLatencyMs = prefs.getInt("httpLatencyMs", 0);
    private volatile boolean noiseEnabled = prefs.getBoolean("noiseEnabled", true);
    private volatile boolean loopRoute = prefs.getBoolean("loopRoute", false);
    private volatile long randomSeed = prefs.getLong("randomSeed", 19656L);

    public double positionNoiseSigma() { return positionNoiseSigma; }
    public double headingNoiseSigma() { return headingNoiseSigma; }
    public double correctionSeconds() { return correctionSeconds; }
    public double speedScale() { return speedScale; }
    public int poseHz() { return poseHz; }
    public int httpLatencyMs() { return httpLatencyMs; }
    public boolean noiseEnabled() { return noiseEnabled; }
    public boolean loopRoute() { return loopRoute; }
    public long randomSeed() { return randomSeed; }

    public void setPositionNoiseSigma(double value) {
        positionNoiseSigma = clamp(value, 0, 12);
        prefs.putDouble("positionNoiseSigma", positionNoiseSigma);
    }

    public void setHeadingNoiseSigma(double value) {
        headingNoiseSigma = clamp(value, 0, 45);
        prefs.putDouble("headingNoiseSigma", headingNoiseSigma);
    }

    public void setCorrectionSeconds(double value) {
        correctionSeconds = clamp(value, 0.05, 10);
        prefs.putDouble("correctionSeconds", correctionSeconds);
    }

    public void setSpeedScale(double value) {
        speedScale = clamp(value, 0.1, 5);
        prefs.putDouble("speedScale", speedScale);
    }

    public void setPoseHz(int value) {
        poseHz = Math.max(1, Math.min(60, value));
        prefs.putInt("poseHz", poseHz);
    }

    public void setHttpLatencyMs(int value) {
        httpLatencyMs = Math.max(0, Math.min(5000, value));
        prefs.putInt("httpLatencyMs", httpLatencyMs);
    }

    public void setNoiseEnabled(boolean value) {
        noiseEnabled = value;
        prefs.putBoolean("noiseEnabled", value);
    }

    public void setLoopRoute(boolean value) {
        loopRoute = value;
        prefs.putBoolean("loopRoute", value);
    }

    public void setRandomSeed(long value) {
        randomSeed = value;
        prefs.putLong("randomSeed", value);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
