package com.michaelgrundvig.frc.simulation;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The robot's chassis as it truly is: where it is, how it's tilted, and how it moves. Only the
 * sensor models and the simulator's own outputs read it; the robot's code sees it through its
 * sensors alone. A live view of its {@link Physics}: read it before the next step or change.
 */
public interface ChassisTruth {
  /** Where the robot is on the field, flat: its center, on the floor, and its heading. */
  Pose2d pose();

  /** Where the robot is, raised and tilted as it is: what its cameras see from. */
  Pose3d pose3d();

  /** Its pitch, WPILib's way: nose up is negative. */
  double pitchRadians();

  /** Its roll, WPILib's way: the left side up is positive. */
  double rollRadians();

  /** Its velocity across the field over the last step, in m/s. */
  Translation2d velocity();

  /** How fast it turned over the last step, counterclockwise, in rad/s. */
  double yawRateRadiansPerSecond();

  /** Every turn it truly made since the start, driving or carried: what a gyro integrates. */
  double yawTurnedRadians();

  /** How long the world has run, in seconds. */
  double seconds();

  /**
   * How fast a point fixed on the robot moves across the field, over the last step: the robot's
   * velocity, and its turn swinging the point around its center.
   *
   * @param place where the point sits, from the robot's center, in its frame
   */
  Translation2d pointVelocity(Translation2d place);
}
