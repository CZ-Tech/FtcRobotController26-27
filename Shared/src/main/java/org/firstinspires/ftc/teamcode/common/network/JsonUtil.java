package org.firstinspires.ftc.teamcode.common.network;

import java.util.Locale;

final class JsonUtil {
    private JsonUtil() {}

    static String escape(String value) {
        if (value == null) return "";
        return value.replace("\\", "\\\\")
                .replace("\"", "\\\"")
                .replace("\n", "\\n")
                .replace("\r", "\\r")
                .replace("\t", "\\t");
    }

    static String number(double value) {
        if (Double.isNaN(value) || Double.isInfinite(value)) return "null";
        return String.format(Locale.US, "%.6f", value);
    }
}
