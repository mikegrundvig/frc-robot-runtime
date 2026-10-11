package com.michaelgrundvig.frc.simulation;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;

/**
 * The one rigid motion of the robot that best fits its wheels' travel between two instants. Today's
 * physics moves the robot by it ({@link KinematicPhysics}).
 *
 * <p><b>The model.</b> If the robot moves forward by dx, left by dy, and turns by dθ, all small, a
 * wheel sitting at (x, y) from the robot's center travels
 *
 * <pre>
 *   forward: dx − y·dθ
 *   left:    dy + x·dθ
 * </pre>
 *
 * <p>Stacking those two rows for each of the n wheels gives a 2n × 3 matrix A with travel = A ·
 * (dx, dy, dθ). The wheels report 2n numbers for 3 unknowns, and they need not agree, so the motion
 * taken is the least-squares answer: (AᵀA)⁻¹ Aᵀ · travel. The matrix (AᵀA)⁻¹ Aᵀ depends only on
 * where the wheels sit, so it is worked out once, here in the constructor.
 *
 * <p><b>What is left out.</b> A wheel's travel is taken as a straight line at the angle halfway
 * between where it pointed before and after. Wheels that disagree are not made to scrub: the fit
 * splits the difference between them.
 *
 * <p>It is the same fit odometry makes, worked the same way to the bit, so the simulated robot
 * moves exactly as recorded runs had it move. It is written here, and not shared with localization,
 * so that the simulator owes nothing to the estimator it tests.
 */
final class RigidFit {
  /**
   * A determinant smaller than this share of the cube of the matrix's entries counts as zero: far
   * below what any real layout of wheels gives, and far above rounding error.
   */
  private static final double SINGULAR_SHARE = 1e-12;

  private final int wheelCount;

  // The three rows of (AᵀA)⁻¹ Aᵀ, one for each unknown. Each row holds 2n weights, two for each
  // wheel in the wheels' order: the weight of wheel i's forward travel at index 2i, and the weight
  // of its leftward travel at index 2i + 1.
  private final double[] forwardWeights;
  private final double[] leftWeights;
  private final double[] turnWeights;

  /**
   * Each wheel's travel between the two instants, in the robot's frame, in the same layout as a row
   * of weights: wheel i's forward travel at index 2i and its leftward travel at 2i + 1. It is kept
   * between calls so that a fit makes no new array.
   */
  private final double[] travel;

  /**
   * @param wheelPlaces where each wheel sits, from the robot's center
   * @throws IllegalArgumentException if the wheels all sit at one point, where no turn can be told
   */
  RigidFit(Translation2d... wheelPlaces) {
    wheelCount = wheelPlaces.length;
    double sumX = 0;
    double sumY = 0;
    double sumSquares = 0;
    for (Translation2d place : wheelPlaces) {
      sumX += place.getX();
      sumY += place.getY();
      sumSquares += place.getX() * place.getX() + place.getY() * place.getY();
    }
    // AᵀA, written out from the two rows each wheel adds to A, [1, 0, −y] and [0, 1, x]:
    //
    //   [  n     0    −Σy      ]
    //   [  0     n     Σx      ]
    //   [ −Σy    Σx    Σ(x²+y²) ]
    double n = wheelCount;
    double[] normalInverse =
        inverseOf3x3(new double[] {n, 0, -sumY, 0, n, sumX, -sumY, sumX, sumSquares});
    forwardWeights = weightsRow(normalInverse, 0, wheelPlaces);
    leftWeights = weightsRow(normalInverse, 1, wheelPlaces);
    turnWeights = weightsRow(normalInverse, 2, wheelPlaces);
    travel = new double[2 * wheelCount];
  }

  /**
   * One row of (AᵀA)⁻¹ Aᵀ: the row of (AᵀA)⁻¹ for one unknown, times each column of Aᵀ. Wheel i
   * gives Aᵀ two columns: (1, 0, −y) for its forward travel and (0, 1, x) for its leftward travel.
   *
   * @param normalInverse (AᵀA)⁻¹, three rows of three, row after row
   * @param unknown which row: 0 for dx, 1 for dy, 2 for dθ
   */
  private static double[] weightsRow(
      double[] normalInverse, int unknown, Translation2d[] wheelPlaces) {
    double fromDx = normalInverse[unknown * 3];
    double fromDy = normalInverse[unknown * 3 + 1];
    double fromDTheta = normalInverse[unknown * 3 + 2];
    double[] weights = new double[2 * wheelPlaces.length];
    for (int i = 0; i < wheelPlaces.length; i++) {
      weights[2 * i] = fromDx - fromDTheta * wheelPlaces[i].getY();
      weights[2 * i + 1] = fromDy + fromDTheta * wheelPlaces[i].getX();
    }
    return weights;
  }

  /**
   * Works out the robot's motion from one instant to the next, in its own frame as it started.
   *
   * @param fromDistances how far each wheel had rolled at the first instant, in meters
   * @param fromAngles which way each wheel pointed then, in radians
   * @param toDistances how far each wheel had rolled at the second instant
   * @param toAngles which way each wheel pointed then
   * @param motion where the answer is put
   */
  void motion(
      double[] fromDistances,
      double[] fromAngles,
      double[] toDistances,
      double[] toAngles,
      RigidMotion motion) {
    for (int i = 0; i < wheelCount; i++) {
      double rolledMeters = toDistances[i] - fromDistances[i];
      // The wheel may have steered in between. It is taken to have rolled at the angle halfway,
      // going the short way round from one angle to the other.
      double steeredRadians = MathUtil.angleModulus(toAngles[i] - fromAngles[i]);
      double halfwayAngle = fromAngles[i] + steeredRadians / 2;
      travel[2 * i] = rolledMeters * Math.cos(halfwayAngle);
      travel[2 * i + 1] = rolledMeters * Math.sin(halfwayAngle);
    }
    // (AᵀA)⁻¹ Aᵀ · travel, one row for each unknown. The sums run in the wheels' order, which the
    // recorded runs depend on to the bit.
    double forward = 0;
    double left = 0;
    double turn = 0;
    for (int k = 0; k < travel.length; k++) {
      forward += forwardWeights[k] * travel[k];
      left += leftWeights[k] * travel[k];
      turn += turnWeights[k] * travel[k];
    }
    motion.forwardMeters = forward;
    motion.leftMeters = left;
    motion.turnRadians = turn;
  }

  /**
   * A 3 × 3 matrix's inverse, as its cofactors over its determinant. The matrix and its inverse are
   * each given as nine numbers, row after row.
   *
   * @throws IllegalArgumentException if the matrix is singular, which is the wheels all at one
   *     point
   */
  private static double[] inverseOf3x3(double[] matrix) {
    double m00 = matrix[0];
    double m01 = matrix[1];
    double m02 = matrix[2];
    double m10 = matrix[3];
    double m11 = matrix[4];
    double m12 = matrix[5];
    double m20 = matrix[6];
    double m21 = matrix[7];
    double m22 = matrix[8];
    // The first row's cofactors, which also give the determinant.
    double cofactor00 = m11 * m22 - m12 * m21;
    double cofactor01 = m12 * m20 - m10 * m22;
    double cofactor02 = m10 * m21 - m11 * m20;
    double determinant = m00 * cofactor00 + m01 * cofactor01 + m02 * cofactor02;
    // The determinant is judged against the size of the matrix's own entries, cubed, so that the
    // test does not depend on the units the wheels' places are measured in.
    double entrySize = Math.abs(m00) + Math.abs(m11) + Math.abs(m22);
    double smallestDeterminant = SINGULAR_SHARE * entrySize * entrySize * entrySize;
    boolean invertible = Math.abs(determinant) > smallestDeterminant;
    if (!invertible) {
      throw new IllegalArgumentException("the wheels must not all sit at one point");
    }
    double perDeterminant = 1 / determinant;
    return new double[] {
      cofactor00 * perDeterminant,
      (m02 * m21 - m01 * m22) * perDeterminant,
      (m01 * m12 - m02 * m11) * perDeterminant,
      cofactor01 * perDeterminant,
      (m00 * m22 - m02 * m20) * perDeterminant,
      (m02 * m10 - m00 * m12) * perDeterminant,
      cofactor02 * perDeterminant,
      (m01 * m20 - m00 * m21) * perDeterminant,
      (m00 * m11 - m01 * m10) * perDeterminant
    };
  }
}
