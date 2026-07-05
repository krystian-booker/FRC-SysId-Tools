package com.frc2852.mechid;

import java.util.List;

/**
 * Minimal JSON writer for meta.json so the library carries no external dependencies.
 * Only supports what MechID emits: objects, arrays, strings, numbers, and booleans.
 */
final class JsonBuilder {
    private final StringBuilder sb = new StringBuilder();
    private boolean needsComma = false;
    private int indent = 0;

    JsonBuilder beginObject() {
        prefix();
        sb.append('{');
        indent++;
        needsComma = false;
        return this;
    }

    JsonBuilder endObject() {
        indent--;
        newline();
        sb.append('}');
        needsComma = true;
        return this;
    }

    JsonBuilder key(String name) {
        prefix();
        appendString(name);
        sb.append(": ");
        needsComma = false;
        return this;
    }

    JsonBuilder value(String v) {
        prefix();
        if (v == null) {
            sb.append("null");
        } else {
            appendString(v);
        }
        needsComma = true;
        return this;
    }

    JsonBuilder value(double v) {
        prefix();
        if (Double.isNaN(v)) {
            sb.append("null");
        } else if (Double.isInfinite(v)) {
            sb.append(v > 0 ? "1e308" : "-1e308");
        } else {
            sb.append(v);
        }
        needsComma = true;
        return this;
    }

    JsonBuilder value(long v) {
        prefix();
        sb.append(v);
        needsComma = true;
        return this;
    }

    JsonBuilder value(boolean v) {
        prefix();
        sb.append(v);
        needsComma = true;
        return this;
    }

    JsonBuilder field(String name, String v) {
        return key(name).value(v);
    }

    JsonBuilder field(String name, double v) {
        return key(name).value(v);
    }

    JsonBuilder field(String name, long v) {
        return key(name).value(v);
    }

    JsonBuilder field(String name, boolean v) {
        return key(name).value(v);
    }

    JsonBuilder arrayField(String name, double[] values) {
        key(name);
        prefix();
        sb.append('[');
        for (int i = 0; i < values.length; i++) {
            if (i > 0) {
                sb.append(", ");
            }
            sb.append(values[i]);
        }
        sb.append(']');
        needsComma = true;
        return this;
    }

    JsonBuilder arrayField(String name, List<String> values) {
        key(name);
        prefix();
        sb.append('[');
        for (int i = 0; i < values.size(); i++) {
            if (i > 0) {
                sb.append(", ");
            }
            appendString(values.get(i));
        }
        sb.append(']');
        needsComma = true;
        return this;
    }

    private void prefix() {
        if (needsComma) {
            sb.append(',');
            newline();
        } else if (indent > 0 && sb.length() > 0 && sb.charAt(sb.length() - 1) == '{') {
            newline();
        }
    }

    private void newline() {
        sb.append('\n');
        sb.append("  ".repeat(Math.max(0, indent)));
    }

    private void appendString(String s) {
        sb.append('"');
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            switch (c) {
                case '"' -> sb.append("\\\"");
                case '\\' -> sb.append("\\\\");
                case '\n' -> sb.append("\\n");
                case '\r' -> sb.append("\\r");
                case '\t' -> sb.append("\\t");
                default -> {
                    if (c < 0x20) {
                        sb.append(String.format("\\u%04x", (int) c));
                    } else {
                        sb.append(c);
                    }
                }
            }
        }
        sb.append('"');
    }

    @Override
    public String toString() {
        return sb.toString();
    }
}
