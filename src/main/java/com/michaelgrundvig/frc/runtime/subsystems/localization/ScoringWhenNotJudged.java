package com.michaelgrundvig.frc.runtime.subsystems.localization;

/**
 * What the scoring gate says while it can't judge the estimate: the robot's localizer doesn't
 * report what the gate judges by (what the pose rests on, the numbers behind that, and how sure it
 * is). The default localizer reports all three, so this matters only for a robot that chose
 * another. An alert says the gate isn't judging, whichever this is.
 */
public enum ScoringWhenNotJudged {
  /**
   * Not ready: scoring and aiming commands bound to the gate wait. The default, since a pose
   * nothing judged is no reason to shoot.
   */
  NOT_READY,

  /**
   * Ready: the gate holds nothing back, and the robot scores as a robot without a gate does. For a
   * team that chose a localizer it trusts without the gate.
   */
  READY
}
