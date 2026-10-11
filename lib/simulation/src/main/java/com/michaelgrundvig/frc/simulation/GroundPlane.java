package com.michaelgrundvig.frc.simulation;

import com.michaelgrundvig.frc.world.field.Terrain;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The plane the robot stands on: the plane that best fits the ground under its wheels, which gives
 * the robot its height and, through its two slopes, its attitude.
 *
 * <p><b>The model.</b> In the robot's own frame, the plane is
 *
 * <pre>
 *   height(x, y) = heightAtCenter + riseForward · x + riseLeft · y
 * </pre>
 *
 * <p>Each wheel at (x, y) gives one measurement, the terrain's height under it. With one row [1, x,
 * y] for each wheel in a matrix A, the three unknowns are the least-squares answer (AᵀA)⁻¹ Aᵀ ·
 * heights. Aᵀ · heights is just three sums over the wheels (Σh, Σx·h, Σy·h), and (AᵀA)⁻¹ depends
 * only on where the wheels sit, so it is worked out once.
 *
 * <p><b>What is left out.</b> The robot is a rigid plane on four points. No wheel leaves the ground
 * or sinks into it, nothing has a suspension, and the robot does not rock: with four wheels on
 * uneven ground, the plane splits the difference between them.
 */
final class GroundPlane {
  private final Translation2d[] wheelPlaces;

  /**
   * (AᵀA)⁻¹ for these wheels, three rows of three, row after row. Row 0 gives the height at the
   * robot's center, row 1 the rise forward, and row 2 the rise to the left, each from the three
   * sums (Σh, Σx·h, Σy·h).
   */
  private final double[] normalInverse;

  private double heightAtCenterMeters;
  private double riseForward;
  private double riseLeft;

  /**
   * @param wheelPlaces where each wheel sits, from the robot's center, in the robot's frame
   */
  GroundPlane(Translation2d[] wheelPlaces) {
    this.wheelPlaces = wheelPlaces;
    normalInverse = normalInverseFor(wheelPlaces);
  }

  /**
   * Fits the plane to the ground under the wheels, with the robot at a pose on the field.
   *
   * @param terrain the ground's height at each point of the field
   * @param pose where the robot's center is, and which way it faces
   */
  void fitUnder(Terrain terrain, Pose2d pose) {
    double sumHeights = 0;
    double sumXHeights = 0;
    double sumYHeights = 0;
    for (Translation2d wheel : wheelPlaces) {
      Translation2d wheelOnTheField =
          pose.getTranslation().plus(wheel.rotateBy(pose.getRotation()));
      double height = terrain.heightAt(wheelOnTheField);
      sumHeights += height;
      sumXHeights += wheel.getX() * height;
      sumYHeights += wheel.getY() * height;
    }
    heightAtCenterMeters = unknown(0, sumHeights, sumXHeights, sumYHeights);
    riseForward = unknown(1, sumHeights, sumXHeights, sumYHeights);
    riseLeft = unknown(2, sumHeights, sumXHeights, sumYHeights);
  }

  /** One row of (AᵀA)⁻¹ times the three sums: one of the plane's three unknowns. */
  private double unknown(int row, double sumHeights, double sumXHeights, double sumYHeights) {
    return normalInverse[3 * row] * sumHeights
        + normalInverse[3 * row + 1] * sumXHeights
        + normalInverse[3 * row + 2] * sumYHeights;
  }

  /** The plane's height under the robot's center, in meters. */
  double heightAtCenterMeters() {
    return heightAtCenterMeters;
  }

  /** How much the plane rises for each meter toward the robot's front: a slope, not an angle. */
  double riseForward() {
    return riseForward;
  }

  /** How much the plane rises for each meter toward the robot's left. */
  double riseLeft() {
    return riseLeft;
  }

  /**
   * The plane's height under a point of the robot.
   *
   * @param place the point, from the robot's center, in the robot's frame
   */
  double heightUnder(Translation2d place) {
    return heightAtCenterMeters + riseForward * place.getX() + riseLeft * place.getY();
  }

  /**
   * (AᵀA)⁻¹ for the plane fit, as the cofactors of AᵀA over its determinant. With a row [1, x, y]
   * for each wheel, AᵀA is
   *
   * <pre>
   *   [ n    Σx    Σy  ]
   *   [ Σx   Σx²   Σxy ]
   *   [ Σy   Σxy   Σy² ]
   * </pre>
   *
   * <p>It is singular only for wheels that all lie on one line, which no drivetrain's do.
   */
  private static double[] normalInverseFor(Translation2d[] wheelPlaces) {
    double n = wheelPlaces.length;
    double sumX = 0;
    double sumY = 0;
    double sumXX = 0;
    double sumXY = 0;
    double sumYY = 0;
    for (Translation2d wheel : wheelPlaces) {
      sumX += wheel.getX();
      sumY += wheel.getY();
      sumXX += wheel.getX() * wheel.getX();
      sumXY += wheel.getX() * wheel.getY();
      sumYY += wheel.getY() * wheel.getY();
    }
    double m00 = n;
    double m01 = sumX;
    double m02 = sumY;
    double m10 = sumX;
    double m11 = sumXX;
    double m12 = sumXY;
    double m20 = sumY;
    double m21 = sumXY;
    double m22 = sumYY;
    double determinant =
        m00 * (m11 * m22 - m12 * m21)
            - m01 * (m10 * m22 - m12 * m20)
            + m02 * (m10 * m21 - m11 * m20);
    return new double[] {
      (m11 * m22 - m12 * m21) / determinant,
      (m02 * m21 - m01 * m22) / determinant,
      (m01 * m12 - m02 * m11) / determinant,
      (m12 * m20 - m10 * m22) / determinant,
      (m00 * m22 - m02 * m20) / determinant,
      (m02 * m10 - m00 * m12) / determinant,
      (m10 * m21 - m11 * m20) / determinant,
      (m01 * m20 - m00 * m21) / determinant,
      (m00 * m11 - m01 * m10) / determinant
    };
  }
}
