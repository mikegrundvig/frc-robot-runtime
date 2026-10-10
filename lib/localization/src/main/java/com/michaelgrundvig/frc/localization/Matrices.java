package com.michaelgrundvig.frc.localization;

/**
 * The few small-matrix operations localization needs, on row-major arrays, so the estimator's
 * every-sample math allocates nothing.
 */
final class Matrices {
  private Matrices() {}

  /**
   * Inverts a 3x3 matrix into {@code out}.
   *
   * @return false, leaving {@code out} unchanged, if the matrix is singular
   */
  static boolean invert3(double[] m, double[] out) {
    double c00 = m[4] * m[8] - m[5] * m[7];
    double c01 = m[5] * m[6] - m[3] * m[8];
    double c02 = m[3] * m[7] - m[4] * m[6];
    double determinant = m[0] * c00 + m[1] * c01 + m[2] * c02;
    double scale = Math.abs(m[0]) + Math.abs(m[4]) + Math.abs(m[8]);
    if (!(Math.abs(determinant) > 1e-12 * scale * scale * scale)) {
      return false;
    }
    double inverse = 1 / determinant;
    out[0] = c00 * inverse;
    out[1] = (m[2] * m[7] - m[1] * m[8]) * inverse;
    out[2] = (m[1] * m[5] - m[2] * m[4]) * inverse;
    out[3] = c01 * inverse;
    out[4] = (m[0] * m[8] - m[2] * m[6]) * inverse;
    out[5] = (m[2] * m[3] - m[0] * m[5]) * inverse;
    out[6] = c02 * inverse;
    out[7] = (m[1] * m[6] - m[0] * m[7]) * inverse;
    out[8] = (m[0] * m[4] - m[1] * m[3]) * inverse;
    return true;
  }

  /** v^T M v, for a 3x3 M. */
  static double quadratic3(double[] m, double[] v) {
    double sum = 0;
    for (int row = 0; row < 3; row++) {
      for (int column = 0; column < 3; column++) {
        sum += v[row] * m[row * 3 + column] * v[column];
      }
    }
    return sum;
  }
}
