package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

/**
 * Whether a joint's mechanism drives the joint, or stands in for a joint that has no mechanism
 * ({@link JointMechanism#standIn}).
 */
sealed interface JointMode {
  /** The mechanism drives its joint, as the joint's block says. */
  record Drives() implements JointMode {}

  /**
   * The mechanism stands in for a joint with no mechanism. It is never read or driven, refuses
   * every move and holds every gate false, saying why.
   *
   * @param why why the joint has no mechanism
   */
  record StandsIn(String why) implements JointMode {}
}
