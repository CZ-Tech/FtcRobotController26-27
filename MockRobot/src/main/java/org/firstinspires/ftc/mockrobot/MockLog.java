package org.firstinspires.ftc.mockrobot;

import java.time.LocalTime;
import java.time.format.DateTimeFormatter;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.Consumer;

/** Small process-local log bus shared by the Swing GUI and Android Log shim. */
public final class MockLog {
    private static final DateTimeFormatter TIME = DateTimeFormatter.ofPattern("HH:mm:ss.SSS");
    private static final CopyOnWriteArrayList<Consumer<String>> listeners =
            new CopyOnWriteArrayList<>();

    private MockLog() {}

    public static void addListener(Consumer<String> listener) {
        if (listener != null) listeners.add(listener);
    }

    public static void removeListener(Consumer<String> listener) {
        listeners.remove(listener);
    }

    public static void info(String tag, String message) {
        emit("INFO", tag, message, null);
    }

    public static void debug(String tag, String message) {
        emit("DEBUG", tag, message, null);
    }

    public static void error(String tag, String message, Throwable throwable) {
        emit("ERROR", tag, message, throwable);
    }

    private static void emit(String level, String tag, String message, Throwable throwable) {
        String line = "[" + TIME.format(LocalTime.now()) + "] [" + level + "] ["
                + tag + "] " + message;
        if ("ERROR".equals(level)) {
            System.err.println(line);
        } else {
            System.out.println(line);
        }
        for (Consumer<String> listener : listeners) listener.accept(line);
        if (throwable != null) {
            String detail = throwable.toString();
            for (Consumer<String> listener : listeners) listener.accept(detail);
            throwable.printStackTrace(System.err);
        }
    }
}
