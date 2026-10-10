package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

/**
 * What a joint shows the interlocks on other joints, and nothing more: where it is and how fast it
 * runs, and whether each is known. Its mechanism publishes it ({@link JointMechanism#view}), read
 * live: an interlock reads this loop's, whatever the commands did to it before.
 */
interface JointView {
  /**
   * Whether its motor controllers answer, with this loop's reading: then how fast it runs is known,
   * homed or not.
   */
  boolean answers();

  /** Whether where it is is known: it's homed, and its motor controllers answer this loop. */
  boolean knowsWhere();

  /** Where it is, in its units: what its motor controller or its encoder reads. */
  double position();

  /** How fast it runs, in its units a second. */
  double velocity();
}
