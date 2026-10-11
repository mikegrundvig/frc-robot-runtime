package com.michaelgrundvig.frc.runtime.subsystems.drive;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;

/**
 * How the driver's sticks are shaped into what they ask of the robot.
 *
 * <p>A stick asks nothing until it is pushed past a small deadband, so a stick that does not spring
 * back quite to center does not make the robot creep. Past the deadband, how far it is pushed is
 * squared: half the stick asks a quarter of the speed, which gives the driver fine control near
 * center and still the top speed at full stick.
 */
final class Sticks {
  /**
   * How far from center a stick must be pushed before it asks anything, as a share of its throw.
   */
  static final double DEADBAND = 0.1;

  private Sticks() {}

  /**
   * The velocity a translation stick asks, in meters a second, along the same axes the stick is
   * given on. The stick's direction is kept, and its length is shaped.
   *
   * @param stick the translation stick, each axis from -1 to 1
   * @param topSpeedMetersPerSecond the speed full stick asks
   */
  static Translation2d velocityAsked(Translation2d stick, double topSpeedMetersPerSecond) {
    double pushedPastDeadband = MathUtil.applyDeadband(stick.getNorm(), DEADBAND);
    // A stick pushed into a corner reads longer than 1, so its share is capped at the whole.
    double shareOfTopSpeed = Math.min(1.0, pushedPastDeadband * pushedPastDeadband);
    Rotation2d direction = new Rotation2d(Math.atan2(stick.getY(), stick.getX()));
    return new Translation2d(shareOfTopSpeed * topSpeedMetersPerSecond, direction);
  }

  /**
   * The turning rate a turn stick asks, in radians a second, counterclockwise for a positive stick.
   *
   * @param stick the turn stick, from -1 to 1
   * @param topTurnRateRadiansPerSecond the turning rate full stick asks
   */
  static double turnRateAsked(double stick, double topTurnRateRadiansPerSecond) {
    double pushedPastDeadband = MathUtil.applyDeadband(stick, DEADBAND);
    // Squaring loses the stick's sign, so it is put back.
    double shareOfTopTurnRate =
        Math.copySign(pushedPastDeadband * pushedPastDeadband, pushedPastDeadband);
    return shareOfTopTurnRate * topTurnRateRadiansPerSecond;
  }

  /** Whether a stick is pushed past its deadband, and so asks for something. */
  static boolean isPushed(double stick) {
    return Math.abs(stick) > DEADBAND;
  }
}
