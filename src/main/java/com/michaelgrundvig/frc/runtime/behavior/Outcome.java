package com.michaelgrundvig.frc.runtime.behavior;

import java.util.Locale;
import org.jspecify.annotations.Nullable;

/**
 * How a move ended, a joint's or the drive's: each move records one, with a reason it logs. The one
 * policy every move ends by: when it has arrived, when its deadline passes, and how its reason
 * reads.
 */
public enum Outcome {
  /** It got there: inside its band for 40 ms, or its condition held. */
  ARRIVED,
  /** Its deadline passed first. */
  TIMED_OUT,
  /** Another command took what it requires, its binding released, or its gate fell. */
  CANCELED,
  /** It wouldn't start, or couldn't go on. */
  REFUSED;

  /** How long a move stays where it's going before it has arrived: 40 ms, two loops. */
  public static final long DWELL_NANOS = 40_000_000L;

  /** A run-until's default deadline, in seconds: what it waits on has 3 s to hold. */
  public static final double RUN_UNTIL_SECONDS = 3;

  /**
   * A move's default deadline, in seconds: twice the time its profile takes, and half a second
   * more. Twice leaves room for a joint or a robot slower than its model; the half second, for a
   * short move to settle.
   *
   * @param profileSeconds how long its profile takes, from where it starts
   */
  public static double deadline(double profileSeconds) {
    return 2 * profileSeconds + 0.5;
  }

  /**
   * The reason a move that ended this way logs: its name and how it ended, how long it ran unless
   * it was refused ("pivot → out: ARRIVED in 0.62 s"), what the move adds of its own, and why.
   *
   * @param name the move's name
   * @param seconds how long it ran
   * @param details what the move says of how it ended, each part led by its own ", " or " (": where
   *     its joints were, when each arrived; empty for nothing
   * @param why why it ended so: a gate's terms, a refusal; null for no reason of its own
   */
  public String reason(String name, double seconds, String details, @Nullable String why) {
    StringBuilder text = new StringBuilder(name).append(": ").append(this);
    if (this != REFUSED) {
      text.append(
          String.format(Locale.ROOT, this == ARRIVED ? " in %.2f s" : " after %.2f s", seconds));
    }
    text.append(details);
    if (why != null) {
      text.append(": ").append(why);
    }
    return text.toString();
  }
}
