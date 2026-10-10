package com.michaelgrundvig.frc.localization;

import java.util.HashMap;
import java.util.Map;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;

/**
 * Swerve kinematics for localization. Each module's travel between two samples is a small vector
 * (how far it rolled, at the angle it pointed); the robot's motion is the one rigid motion that
 * best fits all of them (least squares, as WPILib's odometry does). How badly it fits is the
 * residual: modules that disagree with every rigid motion are slipping, lifted, or being pushed,
 * which is when odometry deserves the least trust (one top team's skid ratio, another's wheel
 * scores).
 *
 * <p>The residual is reported as how far the disagreement may have put the fitted travel off: the
 * modules' misses, spread over what the fit leaves free (two numbers a module, less the motion's
 * three), carried into the travel as least squares carries each module's error into it. One module
 * of four slipping puts the fit off by a quarter of its slip, and the residual says that much,
 * within about a tenth, whichever way it slips (exactly, for a square drivetrain's wheel slipping
 * forward or sideways).
 *
 * <p>A module that didn't report (its sample reads a value that isn't a number: its motor
 * controllers off the bus, say) is left out, and the motion fits the rest: two modules or more fix
 * it, so a robot driving on three keeps its odometry. With fewer, nothing can be fitted, and the
 * motion is none, as doubtful as the farthest a reporting wheel rolled.
 */
public final class SwerveKinematicsModel implements DriveKinematicsModel {
  private final double[] moduleX;
  private final double[] moduleY;

  /**
   * The least-squares fit over some of the modules: which ones, (A^T A)^-1 A^T for the rigid-motion
   * model (3 rows of two a module, row-major), and how the modules' misses, squared, carry into the
   * fitted travel's error, squared (the travel's share of (A^T A)^-1, over the fit's spare degrees
   * of freedom, two a module less the motion's three).
   */
  private record Fit(int[] modules, double[] pseudoInverse, double missToTravel) {}

  /** Each set of reporting modules' fit, by the bits of their indexes, made as each set comes. */
  private final Map<Integer, Fit> fits = new HashMap<>();

  /** Every module's fit. */
  private final Fit all;

  /**
   * @param modules where each module sits, from the robot's center, in the samples' order
   */
  public SwerveKinematicsModel(Translation2d... modules) {
    int n = modules.length;
    if (n < 2) {
      throw new IllegalArgumentException("swerve needs at least 2 modules: " + n);
    }
    if (n > 30) {
      throw new IllegalArgumentException("swerve with at most 30 modules: " + n);
    }
    moduleX = new double[n];
    moduleY = new double[n];
    for (int i = 0; i < n; i++) {
      moduleX[i] = modules[i].getX();
      moduleY[i] = modules[i].getY();
    }
    int[] every = new int[n];
    for (int i = 0; i < n; i++) {
      every[i] = i;
    }
    Fit fit = fit(every);
    if (fit == null) {
      throw new IllegalArgumentException("the modules must not all sit at one point");
    }
    all = fit;
  }

  /** The fit over these modules; null if they can't fix a motion: fewer than two, or one point. */
  private @Nullable Fit fit(int[] modules) {
    int n = modules.length;
    if (n < 2) {
      return null;
    }
    double sumX = 0;
    double sumY = 0;
    double sumSquares = 0;
    for (int i : modules) {
      sumX += moduleX[i];
      sumY += moduleY[i];
      sumSquares += moduleX[i] * moduleX[i] + moduleY[i] * moduleY[i];
    }
    // A has two rows per module: [1, 0, -y] and [0, 1, x], mapping (dx, dy, dtheta) to that
    // module's travel. Its normal matrix A^T A:
    double[] normal = {n, 0, -sumY, 0, n, sumX, -sumY, sumX, sumSquares};
    double[] inverse = new double[9];
    if (!Matrices.invert3(normal, inverse)) {
      return null;
    }
    double[] pseudoInverse = new double[3 * 2 * n];
    for (int row = 0; row < 3; row++) {
      for (int k = 0; k < n; k++) {
        int i = modules[k];
        // Column 2k of A^T is (1, 0, -y_i); column 2k + 1 is (0, 1, x_i).
        pseudoInverse[row * 2 * n + 2 * k] = inverse[row * 3] - inverse[row * 3 + 2] * moduleY[i];
        pseudoInverse[row * 2 * n + 2 * k + 1] =
            inverse[row * 3 + 1] + inverse[row * 3 + 2] * moduleX[i];
      }
    }
    return new Fit(modules, pseudoInverse, (inverse[0] + inverse[4]) / (2 * n - 3));
  }

  @Override
  public BodyMotion motion(OdometrySample from, OdometrySample to) {
    int n = moduleX.length;
    if (from.wheels() != n || to.wheels() != n) {
      throw new IllegalArgumentException(
          "samples of " + from.wheels() + " and " + to.wheels() + " wheels, for " + n + " modules");
    }
    int reporting = 0;
    int count = 0;
    double farthest = 0;
    for (int i = 0; i < n; i++) {
      if (from.reported(i) && to.reported(i)) {
        reporting |= 1 << i;
        count++;
        farthest = Math.max(farthest, Math.abs(to.distanceMeters(i) - from.distanceMeters(i)));
      }
    }
    Fit fit = count == n ? all : fits.computeIfAbsent(reporting, this::fitOf);
    if (fit.modules().length < 2) {
      return new BodyMotion(0, 0, 0, farthest);
    }
    int m = fit.modules().length;
    double[] pseudoInverse = fit.pseudoInverse();
    double[] travel = new double[2 * m];
    for (int k = 0; k < m; k++) {
      int i = fit.modules()[k];
      double distance = to.distanceMeters(i) - from.distanceMeters(i);
      // The module may have steered during the interval: take the angle halfway through it.
      double angle =
          from.angleRadians(i)
              + MathUtil.angleModulus(to.angleRadians(i) - from.angleRadians(i)) / 2;
      travel[2 * k] = distance * Math.cos(angle);
      travel[2 * k + 1] = distance * Math.sin(angle);
    }
    double dx = 0;
    double dy = 0;
    double dtheta = 0;
    for (int k = 0; k < 2 * m; k++) {
      dx += pseudoInverse[k] * travel[k];
      dy += pseudoInverse[2 * m + k] * travel[k];
      dtheta += pseudoInverse[4 * m + k] * travel[k];
    }
    double squares = 0;
    for (int k = 0; k < m; k++) {
      int i = fit.modules()[k];
      double missX = travel[2 * k] - (dx - moduleY[i] * dtheta);
      double missY = travel[2 * k + 1] - (dy + moduleX[i] * dtheta);
      squares += missX * missX + missY * missY;
    }
    return new BodyMotion(dx, dy, dtheta, Math.sqrt(squares * fit.missToTravel()));
  }

  /** The fit over the modules a mask's bits name, or none (no modules) if they can't fix one. */
  private Fit fitOf(int mask) {
    int[] modules = new int[Integer.bitCount(mask)];
    int k = 0;
    for (int i = 0; i < moduleX.length; i++) {
      if ((mask & (1 << i)) != 0) {
        modules[k++] = i;
      }
    }
    Fit fit = fit(modules);
    return fit != null ? fit : new Fit(new int[0], new double[0], 0);
  }
}
