package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

/**
 * What a joint is driven at this loop, as each stage beneath its commands leaves it: what a command
 * asks, or its idle ({@link Idle}), then its interlocks ({@link Interlocks}), then its limit
 * switches ({@link LimitStops}). A stage that changes nothing hands it on as it is.
 *
 * @param kind what it's asked to do
 * @param value what the kind takes, in the joint's units: a position, a speed, volts, or a share of
 *     the battery
 * @param fraction for a position, the share of its cruise it's profiled at
 * @param restart whether its profile starts again from where the joint is, rather than going on
 *     from where it had it: what drives the joint changed under it
 * @param heldBy what holds it short of what was asked, each as it reads, "; " between; empty for
 *     nothing
 */
record Demand(Request kind, double value, double fraction, boolean restart, String heldBy) {
  /** Nothing asked. */
  static final Demand NONE = of(Request.NONE, 0, 1);

  /** Something asked, nothing yet holding it. */
  static Demand of(Request kind, double value, double fraction) {
    return new Demand(kind, value, fraction, false, "");
  }

  /** The same, its profile starting again from where the joint is. */
  Demand restarted() {
    return new Demand(kind, value, fraction, true, heldBy);
  }

  /** What holds a joint, and one thing more, "; " between: either alone if the other's empty. */
  static String joined(String heldBy, String more) {
    return heldBy.isEmpty() ? more : heldBy + "; " + more;
  }
}
