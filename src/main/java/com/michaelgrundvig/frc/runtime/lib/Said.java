package com.michaelgrundvig.frc.runtime.lib;

/**
 * What an exception says, for an alert, an outcome or a log line, in one of two forms. {@link #of}
 * is its message, or the whole of it (its class and message) when its message says nothing: a
 * {@code NullPointerException} often has no message, and "null" alone tells the reader nothing.
 * {@link #withKind} names its kind before its message, where a report of what was thrown and
 * carried on past reads better with both.
 */
public final class Said {
  private Said() {}

  /** Its message, stripped, or its {@code toString()} when the message is missing or blank. */
  public static String of(Throwable thrown) {
    String message = thrown.getMessage();
    if (message == null || message.isBlank()) {
      return thrown.toString();
    }
    return message.strip();
  }

  /**
   * Its kind and what it says: "IllegalStateException: no reading yet". An exception whose message
   * is missing or blank is named by its kind alone.
   */
  public static String withKind(Throwable thrown) {
    String message = thrown.getMessage();
    String kind = thrown.getClass().getSimpleName();
    if (message == null || message.isBlank()) {
      return kind;
    }
    return kind + ": " + message.strip();
  }
}
