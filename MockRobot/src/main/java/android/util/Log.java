package android.util;

import org.firstinspires.ftc.mockrobot.MockLog;

/** JVM compatibility shim used only by the desktop MockRobot build. */
public final class Log {
    private Log() {}

    public static int i(String tag, String message) {
        MockLog.info(tag, message);
        return 0;
    }

    public static int d(String tag, String message) {
        MockLog.debug(tag, message);
        return 0;
    }

    public static int e(String tag, String message) {
        MockLog.error(tag, message, null);
        return 0;
    }

    public static int e(String tag, String message, Throwable throwable) {
        MockLog.error(tag, message, throwable);
        return 0;
    }
}
