package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

/**
 * What a joint does in a loop no command asks anything of it: a joint held at positions holds its
 * goal, and one that only runs at outputs, a roller, stops. Where it holds is where its last move
 * to a position arrived, or, if that move didn't, where the joint came to a stop; where it was
 * first read, enabled, or found by its homing; or where it was as it last ran at an output or
 * stopped. Idle is the joint's own, so a joint a command owns but drives no longer doesn't sag; a
 * default command replaces it, as v3's do.
 */
final class Idle {
  /** Whether the joint holds positions at all: it has a range or named positions. */
  private final boolean positional;

  /** Whether it holds a position when idle: it was last asked for one, or it holds positions. */
  private boolean holds;

  /** Where it holds, in the joint's units: NaN until the joint is first read. */
  private double at = Double.NaN;

  /**
   * @param positional whether the joint holds positions at all
   */
  Idle(boolean positional) {
    this.positional = positional;
    holds = positional;
  }

  /** Where it holds, in the joint's units: NaN until the joint is first read. */
  double at() {
    return at;
  }

  /** It holds at a position from now: where the joint was read, or where homing found it. */
  void holdAt(double position) {
    at = position;
  }

  /**
   * A move to a position ended: it holds where the move arrived, or, if it didn't, where the joint
   * comes to a stop, so a joint that jammed holds where it jammed rather than pushing on toward a
   * target it couldn't reach.
   */
  void moveEnded(double position) {
    holds = true;
    at = position;
  }

  /**
   * What the joint does this loop: what a command asked, or, when none did, its idle.
   *
   * @param asked what a command asked this loop, or {@link Demand#NONE}
   * @param applied what it was driven at the loop before
   * @param here where it is, inside its limits: where it holds from an output or a stop
   */
  Demand resolve(Demand asked, Request applied, double here) {
    if (asked.kind() != Request.NONE) {
      holds = asked.kind() == Request.POSITION || positional;
      return asked;
    }
    if (!holds) {
      return Demand.of(Request.NEUTRAL, 0, 1);
    }
    if (applied != Request.POSITION) {
      // From an output, or a stop: it holds where it is now, its profile from there.
      at = here;
      return Demand.of(Request.POSITION, at, 1).restarted();
    }
    return Demand.of(Request.POSITION, at, 1);
  }
}
