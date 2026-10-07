package android.os;

/** JVM compatibility shim used only by the desktop MockRobot build. */
public final class SystemClock {
    private static final long START_NANOS = System.nanoTime();

    private SystemClock() {}

    public static long elapsedRealtime() {
        return (System.nanoTime() - START_NANOS) / 1_000_000L;
    }
}
