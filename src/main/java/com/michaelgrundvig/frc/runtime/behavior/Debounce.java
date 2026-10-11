package com.michaelgrundvig.frc.runtime.behavior;

import java.util.function.LongSupplier;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.system.RobotController;

/**
 * A debounce that keeps its time in whole nanoseconds: a condition has to stay changed for a time
 * before the debounced answer changes. It answers as WPILib's {@code Debouncer} does, and takes
 * that class's {@link DebounceType}, so a gate's or a button's {@code debounce} reads the same.
 *
 * <p>It exists because WPILib's keeps its time as seconds in a double. A time that is a whole
 * number of loops (0.1 s at 20 ms) is then compared with a difference of two doubles that comes out
 * a hair under or over it, depending on how long the robot has been on. The edge landed a loop
 * early or late from one run to the next. The robot's clock counts nanoseconds, and whole
 * nanoseconds subtract exactly, so here the edge lands on the same loop whatever the clock reads.
 */
public final class Debounce {
  private static final double NANOS_PER_SECOND = 1e9;

  private final long nanos;
  private final DebounceType type;

  /** The robot's clock, in nanoseconds. */
  private final LongSupplier clock;

  /** What the debounced answer is until the condition has stayed changed for the time. */
  private boolean baseline;

  /** When the condition last agreed with the baseline, on the robot's clock. */
  private long agreedAtNanos;

  /**
   * @param seconds how long the condition has to stay changed
   * @param type which change waits: becoming true, becoming false, or both
   */
  public Debounce(double seconds, DebounceType type) {
    this(seconds, type, RobotController::getTime);
  }

  /** A debounce on a clock of the caller's, which a test steps by hand. */
  Debounce(double seconds, DebounceType type, LongSupplier clock) {
    nanos = wholeNanos(seconds);
    this.type = type;
    this.clock = clock;
    baseline = type == DebounceType.FALLING;
    agreedAtNanos = clock.getAsLong();
  }

  /**
   * The debounced answer for the condition as it is now. Ask once a loop, whatever the condition
   * says, so that the time it has stayed changed is kept.
   */
  public boolean calculate(boolean condition) {
    long now = clock.getAsLong();
    if (condition == baseline) {
      agreedAtNanos = now;
    }
    boolean hasStayedChanged = now - agreedAtNanos >= nanos;
    if (!hasStayedChanged) {
      return baseline;
    }
    if (type == DebounceType.BOTH) {
      baseline = condition;
      agreedAtNanos = now;
    }
    return condition;
  }

  /**
   * A time as whole nanoseconds. A time that is not a number never passes, as it never does for
   * WPILib's debouncer.
   */
  private static long wholeNanos(double seconds) {
    if (Double.isNaN(seconds)) {
      return Long.MAX_VALUE;
    }
    return Math.round(seconds * NANOS_PER_SECOND);
  }
}
