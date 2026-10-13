package com.michaelgrundvig.frc.runtime.behavior;

import com.michaelgrundvig.frc.world.config.Rules;
import com.michaelgrundvig.frc.world.record.MoveResult;
import java.util.Optional;

/**
 * How a move ended, whether it is a joint's or the drivetrain's. Each move records one, with the
 * facts of its run, which it logs. This is also the one policy every move ends by: when it counts
 * as arrived, and when its deadline passes. How an ending reads as a sentence is the model's to say
 * ({@link com.michaelgrundvig.frc.world.record.MoveSentences}).
 */
public enum Outcome {
  /** It got there: it stayed inside its band for 40 ms, or its condition held. */
  ARRIVED,
  /** Its deadline passed first: its own, by its profile, or the one its team gave it. */
  TIMED_OUT,
  /**
   * Another command took what it requires, its binding released, its gate fell, or the robot was
   * disabled ({@link Disabling}).
   */
  CANCELED,
  /** It would not start, or could not go on. */
  REFUSED;

  /** How long a move must stay where it is going before it has arrived: 40 ms, two loops. */
  public static final long DWELL_NANOS = 40_000_000L;

  /**
   * The default deadline of a move to a place, in seconds: twice the time its profile takes, and
   * half a second more. Doubling leaves room for a joint or a robot that is slower than its model.
   * The half second leaves a short move time to settle.
   *
   * <p>Only a move to a place has a default. A move that runs until a condition holds has no
   * natural duration, since nothing says how long a piece takes to turn up, so it has no deadline
   * unless its team gives it one ({@code within}).
   *
   * @param profileSeconds how long its profile takes, from where it starts
   */
  public static double deadline(double profileSeconds) {
    return 2 * profileSeconds + 0.5;
  }

  /**
   * Why a time cannot be the deadline a team gives a move ({@code within}), if it cannot. A
   * deadline is a time above zero. None, a negative one, one without end, and one that is not a
   * number are all refused, the same for a joint's move and the drivetrain's.
   *
   * @param seconds the deadline the team gave
   */
  public static Optional<String> whyNotADeadline(double seconds) {
    boolean isATimeAboveZero = Double.isFinite(seconds) && seconds > 0;
    if (isATimeAboveZero) {
      return Optional.empty();
    }
    return Optional.of(
        "a deadline is a time above 0 s, not "
            + Rules.shown(seconds)
            + " s: give within() how long the move may take");
  }

  /** This outcome as a move's logged facts say it. */
  public MoveResult result() {
    return switch (this) {
      case ARRIVED -> MoveResult.ARRIVED;
      case TIMED_OUT -> MoveResult.TIMED_OUT;
      case CANCELED -> MoveResult.CANCELED;
      case REFUSED -> MoveResult.REFUSED;
    };
  }
}
