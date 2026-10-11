package com.michaelgrundvig.frc.runtime.sim;

/**
 * Where a simulated absolute encoder says its joint is, as a test has failed the encoder. A healthy
 * one says where the joint really is. One that has slipped on its shaft, as a loose magnet does,
 * says the joint is that far off. One that is frozen goes on saying what it said last, however the
 * joint moves.
 *
 * <p>The position is the joint's, unwrapped, in the joint's units. Turning it into the encoder's
 * own signal, a share of a turn, is for whoever reads the encoder.
 */
final class EncoderReading {
  private boolean frozen;

  /**
   * How far off it says the joint is, in the joint's units. Zero for an encoder that has not
   * slipped.
   */
  private double slip;

  /** What it said last, once it has said anything: what it goes on saying while frozen. */
  private double lastSaid;

  private boolean hasSaidAnything;

  /** Freezes it at what it said last, or lets it follow the joint again. */
  void freeze(boolean frozen) {
    this.frozen = frozen;
  }

  /** Slips it on its shaft: from now on it says the joint is this far off. Zero puts it back. */
  void slip(double off) {
    slip = off;
  }

  /**
   * Where it says the joint is now.
   *
   * @param truePosition where the joint really is
   */
  double of(double truePosition) {
    // An encoder frozen before it ever spoke has nothing to repeat, so it speaks once.
    boolean repeatsItself = frozen && hasSaidAnything;
    if (!repeatsItself) {
      lastSaid = truePosition + slip;
      hasSaidAnything = true;
    }
    return lastSaid;
  }
}
