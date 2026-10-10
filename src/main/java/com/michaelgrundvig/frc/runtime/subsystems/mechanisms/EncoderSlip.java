package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.world.robot.EncoderGap;
import org.jspecify.annotations.Nullable;

/**
 * A joint's absolute encoder against its motor's count, each loop it has both and knows where it
 * is: apart ({@link EncoderGap}) for a quarter second, one of them slipped, and where the joint is
 * isn't known ({@link JointHoming}). Agreeing again, inside the margin, the encoder is trusted
 * again.
 */
final class EncoderSlip {
  /** How long they must be apart before one is taken to have slipped. */
  static final long SLIP_NANOS = 250_000_000L;

  private final EncoderGap gap;

  /** Since when they've been apart, in nanoseconds: -1 while they agree. */
  private long since = -1;

  /**
   * @param gap how far apart they may be, and whether they are
   */
  EncoderSlip(EncoderGap gap) {
    this.gap = gap;
  }

  /**
   * Whether they've been apart long enough to say one slipped, as of this loop: how far, in the
   * joint's words, or null. Either reading missing this loop (NaN) says nothing, and starts over.
   *
   * @param encoder where the encoder says the joint is
   * @param motor where its motor's count says it is
   * @param velocity how fast it moves: a loop's travel at it is allowed beside the margin
   */
  @Nullable String apart(long now, double encoder, double motor, double velocity) {
    if (!gap.apart(encoder, motor, velocity, RobotLoop.PERIOD_SECONDS)) {
      since = -1;
      return null;
    }
    if (since < 0) {
      since = now;
    }
    return now - since >= SLIP_NANOS ? gap.shown(encoder, motor) : null;
  }

  /** Whether they agree now, inside the margin: an encoder distrusted for its gap is trusted. */
  boolean agree(double encoder, double motor) {
    return gap.agree(encoder, motor);
  }
}
