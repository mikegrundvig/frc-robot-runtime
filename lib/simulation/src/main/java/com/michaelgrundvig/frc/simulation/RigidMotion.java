package com.michaelgrundvig.frc.simulation;

/**
 * A small rigid motion of the robot across the floor, in the robot's own frame as the motion
 * started: how far its center moved forward and to its left, and how far it turned. {@link
 * RigidFit} fills one in each physics step, so the physics keeps a single one and nothing is made
 * per step.
 */
final class RigidMotion {
  /** How far the robot's center moved along its forward axis, in meters. */
  double forwardMeters;

  /** How far the robot's center moved along its left axis, in meters. */
  double leftMeters;

  /** How far the robot turned, counterclockwise seen from above, in radians. */
  double turnRadians;
}
