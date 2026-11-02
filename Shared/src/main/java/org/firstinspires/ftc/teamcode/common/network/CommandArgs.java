package org.firstinspires.ftc.teamcode.common.network;

import java.util.ArrayList;
import java.util.List;

/**
 * Hardware-free conversion of string command arguments into typed values.
 *
 * <p>Type names follow {@code GET /commands}: the eight primitives and their wrappers
 * (optionally {@code java.lang.}-prefixed) are converted; any other type is plain text.
 * Conversion is strict so a bad value fails the request instead of silently becoming
 * a default.</p>
 */
public final class CommandArgs {
    private CommandArgs() {}

    /** Converts every argument; sizes of {@code types} and {@code values} must match. */
    public static Object[] convertAll(List<String> types, List<String> values) {
        if (types.size() != values.size()) {
            throw new IllegalArgumentException("Expected " + types.size()
                    + " argument(s), got " + values.size());
        }
        List<Object> converted = new ArrayList<>(values.size());
        for (int i = 0; i < values.size(); i++) {
            try {
                converted.add(convert(values.get(i), types.get(i)));
            } catch (IllegalArgumentException e) {
                throw new IllegalArgumentException(
                        "Argument " + i + ": " + e.getMessage(), e);
            }
        }
        return converted.toArray();
    }

    public static Object convert(String value, String type) {
        if (value == null) throw new IllegalArgumentException("Value is null");
        String name = type == null ? "" : type;
        if (name.startsWith("java.lang.")) name = name.substring("java.lang.".length());

        switch (name) {
            case "int":
            case "Integer":
                try {
                    return Integer.valueOf(value);
                } catch (NumberFormatException e) {
                    throw invalid(value, "int");
                }
            case "long":
            case "Long":
                try {
                    return Long.valueOf(value);
                } catch (NumberFormatException e) {
                    throw invalid(value, "long");
                }
            case "short":
            case "Short":
                try {
                    return Short.valueOf(value);
                } catch (NumberFormatException e) {
                    throw invalid(value, "short");
                }
            case "byte":
            case "Byte":
                try {
                    return Byte.valueOf(value);
                } catch (NumberFormatException e) {
                    throw invalid(value, "byte");
                }
            case "float":
            case "Float":
                try {
                    float f = Float.parseFloat(value);
                    if (Float.isNaN(f) || Float.isInfinite(f)) throw invalid(value, "float");
                    return f;
                } catch (NumberFormatException e) {
                    throw invalid(value, "float");
                }
            case "double":
            case "Double":
                try {
                    double d = Double.parseDouble(value);
                    if (Double.isNaN(d) || Double.isInfinite(d)) throw invalid(value, "double");
                    return d;
                } catch (NumberFormatException e) {
                    throw invalid(value, "double");
                }
            case "boolean":
            case "Boolean":
                if (value.equalsIgnoreCase("true")) return Boolean.TRUE;
                if (value.equalsIgnoreCase("false")) return Boolean.FALSE;
                throw invalid(value, "boolean");
            case "char":
            case "Character":
                if (value.length() != 1) throw invalid(value, "char");
                return value.charAt(0);
            default:
                return value;
        }
    }

    private static IllegalArgumentException invalid(String value, String type) {
        return new IllegalArgumentException("'" + value + "' is not a valid " + type);
    }
}
