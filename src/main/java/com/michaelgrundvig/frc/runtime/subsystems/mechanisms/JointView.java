package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

/**
 * What a joint shows to the interlocks on other joints, and nothing more: where it is, how fast it
 * runs, and whether each of those is known.
 *
 * <p>An interlock on one joint often depends on another joint ("the roller runs only when the pivot
 * is out"). It reads that other joint through this view and never through the joint's mechanism, so
 * that an interlock can read a joint but cannot drive it. Each joint's mechanism publishes its own
 * view ({@link JointMechanism#view}). The view is read live, so an interlock sees this loop's
 * readings whatever the commands have done so far in the loop.
 */
interface JointView {
  /**
   * Whether the joint's motor controllers answered with a reading in this loop. If they did, how
   * fast the joint runs is known, whether or not the joint is homed.
   */
  boolean answers();

  /**
   * Whether where the joint is, is known: the joint is homed, and its motor controllers answered
   * with a reading in this loop.
   */
  boolean knowsWhere();

  /** Where the joint is, in its units: what its motor controller or its encoder reads. */
  double position();

  /** How fast the joint runs, in its units a second. */
  double velocity();
}
