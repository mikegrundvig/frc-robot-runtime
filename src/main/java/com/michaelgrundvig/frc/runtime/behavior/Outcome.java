package com.michaelgrundvig.frc.runtime.behavior;

import java.util.Locale;
import java.util.Optional;
import org.jspecify.annotations.Nullable;

/**
 * How a move ended, whether it is a joint's or the drivetrain's. Each move records one, with a
 * reason that it logs. This is also the one policy every move ends by: when it counts as arrived,
 * when its deadline passes, and how its reason reads.
 */
public enum Outcome {
  /** It got there: it stayed inside its band for 40 ms, or its condition held. */
  ARRIVED,
  /** Its deadline passed first. */
  TIMED_OUT,
  /** Another command took what it requires, its binding released, or its gate fell. */
  CANCELED,
  /** It would not start, or could not go on. */
  REFUSED;

  /** How long a move must stay where it is going before it has arrived: 40 ms, two loops. */
  public static final long DWELL_NANOS = 40_000_000L;

  /** The default deadline of a run-until, in seconds: what it waits on has 3 s to hold. */
  public static final double RUN_UNTIL_SECONDS = 3;

  /**
   * The default deadline of a move, in seconds: twice the time its profile takes, and half a second
   * more. Doubling leaves room for a joint or a robot that is slower than its model. The half
   * second leaves a short move time to settle.
   *
   * @param profileSeconds how long its profile takes, from where it starts
   */
  public static double deadline(double profileSeconds) {
    return 2 * profileSeconds + 0.5;
  }

  /**
   * The reason that a move which ended this way logs. In order, it holds the move's name and how it
   * ended, how long the move ran unless it was refused ("pivot → out: ARRIVED in 0.62 s"), what the
   * move adds of its own, and why it ended so.
   *
   * @param name the move's name
   * @param seconds how long it ran
   * @param details what the move itself says of how it ended, such as where its joints were and
   *     when each arrived. Each part comes with its own leading ", " or " (". Empty for nothing.
   * @param why why it ended this way, such as a gate's terms or a refusal, if the move has a reason
   *     of its own
   */
  public String reasonGiven(String name, double seconds, String details, Optional<String> why) {
    StringBuilder text = new StringBuilder(name).append(": ").append(this);
    if (this != REFUSED) {
      String howLong = this == ARRIVED ? " in %.2f s" : " after %.2f s";
      text.append(String.format(Locale.ROOT, howLong, seconds));
    }
    text.append(details);
    if (why.isPresent()) {
      text.append(": ").append(why.get());
    }
    return text.toString();
  }

  /**
   * The same reason as {@link #reasonGiven}, for a caller that holds why as a string that may be
   * null.
   *
   * @param why why it ended this way; null when the move has no reason of its own
   */
  public String reason(String name, double seconds, String details, @Nullable String why) {
    return reasonGiven(name, seconds, details, Optional.ofNullable(why));
  }
}
