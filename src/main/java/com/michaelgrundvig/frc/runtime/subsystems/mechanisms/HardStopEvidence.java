package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

/**
 * What a push into a hard stop has seen so far, for the fault to say when the push runs out of
 * time: whether the joint's motor controller ever answered, how far the joint moved, and the most
 * current its motor drew. It is kept here, and not in the push's phase, because it changes every
 * loop and a phase is made only when the joint moves on to another.
 */
final class HardStopEvidence {
  /** How the push went, as its fault says it. */
  enum Seen {
    /** The motor controller never answered with a loop's readings. */
    NO_READING,
    /** The joint never moved as far as {@link JointHoming#MOVED_SHARE} of its range. */
    NEVER_MOVED,
    /** The joint moved, but never stood still on a stop while its motor drew a stall's current. */
    MOVED
  }

  /** Where the joint first read during the push, or NaN before it has. */
  private double from = Double.NaN;

  /** The farthest it has read from there, either way. */
  private double farthest;

  /** The most current its motor has drawn, in amps. */
  private double mostAmps;

  /** Forgets the push before, as a new one starts. */
  void start() {
    from = Double.NaN;
    farthest = 0;
    mostAmps = 0;
  }

  /** Takes in a loop's readings. A loop the motor controller did not answer says nothing. */
  void saw(JointHoming.Pushed read) {
    if (!read.answersThisLoop()) {
      return;
    }
    mostAmps = Math.max(mostAmps, read.statorCurrentAmps());
    // A position that is not a number says nothing of how far the joint moved.
    if (!Double.isFinite(read.position())) {
      return;
    }
    if (Double.isNaN(from)) {
      from = read.position();
    }
    farthest = Math.max(farthest, Math.abs(read.position() - from));
  }

  /**
   * How the push went.
   *
   * @param least the least distance the joint must have moved to have moved at all, in its units
   */
  Seen seen(double least) {
    if (Double.isNaN(from)) {
      return Seen.NO_READING;
    }
    if (farthest < least) {
      return Seen.NEVER_MOVED;
    }
    return Seen.MOVED;
  }

  /** The most current the motor drew during the push, in amps. */
  double mostAmps() {
    return mostAmps;
  }
}
