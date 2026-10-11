package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.world.robot.EncoderGap;
import java.util.Optional;
import java.util.OptionalLong;

/**
 * Notices when a joint's absolute encoder and its motor's count have come apart, which means one of
 * them slipped: a magnet loose on its shaft, or a belt that skipped.
 *
 * <p>The model says how far apart the two may be ({@link EncoderGap}). They are compared in every
 * loop that has both readings, and once they have been too far apart for a quarter of a second, one
 * is taken to have slipped. A single loop apart is not enough, because the two are not read at the
 * same instant.
 */
final class EncoderSlip {
  /** How long the two must be apart before one is taken to have slipped, in nanoseconds. */
  static final long SLIP_NANOS = 250_000_000L;

  private final EncoderGap gap;

  /** Since when the two have been apart, in nanoseconds, or empty while they agree. */
  private OptionalLong apartSince = OptionalLong.empty();

  /**
   * @param gap how far apart the two may be, from the model
   */
  EncoderSlip(EncoderGap gap) {
    this.gap = gap;
  }

  /**
   * Compares the two for this loop, and says how far apart they are, in the joint's words, once
   * they have been apart long enough for one to have slipped. Empty until then.
   *
   * <p>A loop in which either reading is missing says nothing, and the count of how long they have
   * been apart starts over.
   *
   * @param encoder where the encoder says the joint is, or NaN for no reading this loop
   * @param motor where the motor's count says it is
   * @param velocity how fast the joint moves. The distance it travels in one loop is allowed on top
   *     of the margin, since the two readings are a loop apart at worst.
   */
  Optional<String> slipped(long now, double encoder, double motor, double velocity) {
    if (!gap.apart(encoder, motor, velocity, RobotLoop.PERIOD_SECONDS)) {
      apartSince = OptionalLong.empty();
      return Optional.empty();
    }
    if (apartSince.isEmpty()) {
      apartSince = OptionalLong.of(now);
    }
    boolean apartLongEnough = now - apartSince.getAsLong() >= SLIP_NANOS;
    if (!apartLongEnough) {
      return Optional.empty();
    }
    return Optional.of(gap.shown(encoder, motor));
  }

  /**
   * Whether the two agree now, inside the margin. An encoder that was distrusted for having come
   * apart from the motor's count is trusted again once they do.
   */
  boolean agree(double encoder, double motor) {
    return gap.agree(encoder, motor);
  }
}
