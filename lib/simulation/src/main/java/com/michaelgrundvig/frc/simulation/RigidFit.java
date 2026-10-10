package com.michaelgrundvig.frc.simulation;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;

/**
 * The one rigid motion of the robot that best fits its wheels' travel between two instants: each
 * wheel's travel a small vector (how far it rolled, at the angle it pointed), and the motion the
 * least-squares fit of them all. Today's physics moves the robot by it ({@link KinematicPhysics}).
 *
 * <p>It's the same fit odometry makes, worked the same way to the bit, so the simulated robot moves
 * exactly as recorded runs had it move; it's written here so the simulator owes nothing to the
 * estimator it tests.
 */
final class RigidFit {
  private final double[] moduleX;
  private final double[] moduleY;

  /** (A^T A)^-1 A^T for the rigid-motion model: 3 rows of 2n, row-major. */
  private final double[] pseudoInverse;

  /**
   * @param modules where each wheel sits, from the robot's center
   */
  RigidFit(Translation2d... modules) {
    int n = modules.length;
    moduleX = new double[n];
    moduleY = new double[n];
    double sumX = 0;
    double sumY = 0;
    double sumSquares = 0;
    for (int i = 0; i < n; i++) {
      moduleX[i] = modules[i].getX();
      moduleY[i] = modules[i].getY();
      sumX += moduleX[i];
      sumY += moduleY[i];
      sumSquares += moduleX[i] * moduleX[i] + moduleY[i] * moduleY[i];
    }
    // A has two rows per wheel: [1, 0, -y] and [0, 1, x], mapping (dx, dy, dtheta) to that wheel's
    // travel. Its normal matrix A^T A, inverted:
    double[] inverse = invert3(new double[] {n, 0, -sumY, 0, n, sumX, -sumY, sumX, sumSquares});
    pseudoInverse = new double[3 * 2 * n];
    for (int row = 0; row < 3; row++) {
      for (int i = 0; i < n; i++) {
        // Column 2i of A^T is (1, 0, -y_i); column 2i + 1 is (0, 1, x_i).
        pseudoInverse[row * 2 * n + 2 * i] = inverse[row * 3] - inverse[row * 3 + 2] * moduleY[i];
        pseudoInverse[row * 2 * n + 2 * i + 1] =
            inverse[row * 3 + 1] + inverse[row * 3 + 2] * moduleX[i];
      }
    }
  }

  /**
   * The robot's motion from one instant to the next, in its own frame as it started.
   *
   * @param fromDistances how far each wheel had rolled, at the first
   * @param fromAngles which way each pointed then
   * @param toDistances and at the second
   * @param toAngles and then
   * @param motion where to put it: forward and left in meters, and the turn counterclockwise
   */
  void motion(
      double[] fromDistances,
      double[] fromAngles,
      double[] toDistances,
      double[] toAngles,
      double[] motion) {
    int n = moduleX.length;
    double[] travel = new double[2 * n];
    for (int i = 0; i < n; i++) {
      double distance = toDistances[i] - fromDistances[i];
      // The wheel may have steered in between: take the angle halfway.
      double angle = fromAngles[i] + MathUtil.angleModulus(toAngles[i] - fromAngles[i]) / 2;
      travel[2 * i] = distance * Math.cos(angle);
      travel[2 * i + 1] = distance * Math.sin(angle);
    }
    double dx = 0;
    double dy = 0;
    double dtheta = 0;
    for (int k = 0; k < 2 * n; k++) {
      dx += pseudoInverse[k] * travel[k];
      dy += pseudoInverse[2 * n + k] * travel[k];
      dtheta += pseudoInverse[4 * n + k] * travel[k];
    }
    motion[0] = dx;
    motion[1] = dy;
    motion[2] = dtheta;
  }

  /**
   * A 3x3 matrix's inverse, row-major.
   *
   * @throws IllegalArgumentException if it's singular: the wheels all at one point
   */
  private static double[] invert3(double[] m) {
    double c00 = m[4] * m[8] - m[5] * m[7];
    double c01 = m[5] * m[6] - m[3] * m[8];
    double c02 = m[3] * m[7] - m[4] * m[6];
    double determinant = m[0] * c00 + m[1] * c01 + m[2] * c02;
    double scale = Math.abs(m[0]) + Math.abs(m[4]) + Math.abs(m[8]);
    if (!(Math.abs(determinant) > 1e-12 * scale * scale * scale)) {
      throw new IllegalArgumentException("the wheels must not all sit at one point");
    }
    double inverse = 1 / determinant;
    return new double[] {
      c00 * inverse,
      (m[2] * m[7] - m[1] * m[8]) * inverse,
      (m[1] * m[5] - m[2] * m[4]) * inverse,
      c01 * inverse,
      (m[0] * m[8] - m[2] * m[6]) * inverse,
      (m[2] * m[3] - m[0] * m[5]) * inverse,
      c02 * inverse,
      (m[1] * m[6] - m[0] * m[7]) * inverse,
      (m[0] * m[4] - m[1] * m[3]) * inverse
    };
  }
}
