package com.michaelgrundvig.frc.runtime.lib.health;

import java.util.Locale;

/**
 * A time span in words for the pit: "0.4 s", "40 s", "3 min", "1 h 5 min"; and how often something
 * happened: "once", "3 times".
 */
public final class Durations {
  private Durations() {}

  /** A span of this many seconds, to what's worth reading. */
  public static String said(double seconds) {
    if (seconds < 10) {
      return String.format(Locale.ROOT, "%.1f s", seconds);
    }
    if (seconds < 120) {
      return String.format(Locale.ROOT, "%.0f s", seconds);
    }
    long minutes = Math.round(seconds / 60);
    if (minutes < 60) {
      return minutes + " min";
    }
    return minutes / 60 + " h " + minutes % 60 + " min";
  }

  /** How often something happened, for a card's line: "once", "3 times". */
  public static String times(long count) {
    if (count == 1) {
      return "once";
    }
    return count + " times";
  }
}
