package com.michaelgrundvig.frc.localization.calibration;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.linalg.VecBuilder;

/**
 * Where a camera really is on the robot, from the tags it sees while the robot's pose is known
 * (from the other cameras, or a surveyed spot): the mount that best explains every sighting, found
 * by least squares over its six degrees of freedom, starting from the configured mount.
 *
 * <p>Each sighting says where the tag should be if the mount were right: the robot's pose, then the
 * mount, then the camera's view of the tag. The solver moves the mount until those land on the
 * tags' real places, and turns the tags to their real orientations. Each miss is weighed by how
 * well a camera sees it: a tag's place across the line of sight to within millimeters (growing with
 * distance), its distance along the line of sight far less well (growing with distance squared: the
 * tag's apparent size gives it), and its orientation in between.
 *
 * <p>Sightings must vary: a camera 4 in low and one tilted down 2° put a tag 3 m away in the same
 * wrong place. Tags seen from near and far, and from several angles, tell them apart; the result
 * says how sure it is of each value, so a poorly varied set shows as wide uncertainty.
 */
public final class CameraMountCalibration {
  // How well a camera sees a tag, per meter of distance: across the line of sight, along it (per
  // meter squared), and the tag's orientation, in radians.
  private static final double ACROSS_PER_METER = 0.002;
  private static final double ALONG_PER_METER_SQUARED = 0.004;
  private static final double TURN_PER_METER = 0.02;

  private static final int MAX_ITERATIONS = 100;
  private static final double STEP = 1e-6;

  private final Transform3d configured;
  private final List<Sighting> sightings = new ArrayList<>();

  /**
   * A tag seen: both poses the vision software found for it, the one chosen at the current mount,
   * and that one's line of sight in the field's frame and the spreads to weigh its misses by.
   */
  private static final class Sighting {
    final Pose3d robot;
    final Transform3d best;
    final @Nullable Transform3d alternate;
    final Pose3d fieldToTag;
    Transform3d view;
    Translation3d ray = Translation3d.ZERO;
    double acrossMeters;
    double alongMeters;
    double turnRadians;

    Sighting(Pose3d robot, Transform3d best, @Nullable Transform3d alternate, Pose3d fieldToTag) {
      this.robot = robot;
      this.best = best;
      this.alternate = alternate;
      this.fieldToTag = fieldToTag;
      view = best;
    }

    /**
     * Chooses the candidate that fits better at a mount, place and orientation together (the wrong
     * one is turned over), and weighs it along its line of sight.
     *
     * @return whether the choice changed
     */
    boolean choose(Transform3d mount) {
      Transform3d before = view;
      view = best;
      aim(mount);
      Transform3d other = alternate;
      if (other != null) {
        double bestMiss = weightedMiss(mount);
        view = other;
        aim(mount);
        if (weightedMiss(mount) >= bestMiss) {
          view = best;
          aim(mount);
        }
      }
      return view != before;
    }

    private void aim(Transform3d mount) {
      double distance = Math.max(view.getTranslation().getNorm(), 0.1);
      ray = view.getTranslation().div(distance).rotateBy(robot.transformBy(mount).getRotation());
      acrossMeters = ACROSS_PER_METER * distance;
      alongMeters = ALONG_PER_METER_SQUARED * distance * distance;
      turnRadians = TURN_PER_METER * distance;
    }

    private double weightedMiss(Transform3d mount) {
      double[] residuals = new double[7];
      residualsInto(mount, residuals, 0);
      double sum = 0;
      for (double residual : residuals) {
        sum += residual * residual;
      }
      return sum;
    }

    /** This sighting's miss at a mount, each part over how well a camera sees it. */
    void residualsInto(Transform3d mount, double[] residuals, int at) {
      Pose3d seen = robot.transformBy(mount).transformBy(view);
      Translation3d miss = seen.getTranslation().minus(fieldToTag.getTranslation());
      double along = miss.getX() * ray.getX() + miss.getY() * ray.getY() + miss.getZ() * ray.getZ();
      Translation3d across = miss.minus(ray.times(along));
      var turn = seen.getRotation().relativeTo(fieldToTag.getRotation()).toVector();
      residuals[at] = across.getX() / acrossMeters;
      residuals[at + 1] = across.getY() / acrossMeters;
      residuals[at + 2] = across.getZ() / acrossMeters;
      residuals[at + 3] = along / alongMeters;
      residuals[at + 4] = turn.get(0) / turnRadians;
      residuals[at + 5] = turn.get(1) / turnRadians;
      residuals[at + 6] = turn.get(2) / turnRadians;
    }

    /** How far the tag lands from its place across the line of sight, at a mount. */
    double acrossMiss(Transform3d mount) {
      Translation3d miss =
          robot
              .transformBy(mount)
              .transformBy(view)
              .getTranslation()
              .minus(fieldToTag.getTranslation());
      double along = miss.getX() * ray.getX() + miss.getY() * ray.getY() + miss.getZ() * ray.getZ();
      return miss.minus(ray.times(along)).getNorm();
    }
  }

  /**
   * The solved mount.
   *
   * @param robotToCamera where the camera is, as {@code CameraConfig.robotToCamera()} gives it
   * @param residualBeforeMeters how far the tags landed from their places across the line of sight,
   *     where a camera sees them sharply, with the configured mount, on average (root mean square)
   * @param residualAfterMeters and with the solved mount
   * @param sightings how many tag sightings it rests on
   * @param stdDevs how sure it is of each value: x, y, z in meters, then rotations about x, y, z in
   *     radians
   */
  public record Result(
      Transform3d robotToCamera,
      double residualBeforeMeters,
      double residualAfterMeters,
      int sightings,
      double[] stdDevs) {}

  /**
   * @param configuredRobotToCamera where the configuration says the camera is: the start
   */
  public CameraMountCalibration(Transform3d configuredRobotToCamera) {
    configured = configuredRobotToCamera;
  }

  /**
   * A tag the camera saw. Of the two poses the vision software found for it, the solver uses the
   * one that fits the mount it's converging on: the other is the tag turned over.
   *
   * @param robot where the robot was, tilted as it was, when the image was taken
   * @param bestCameraToTag the vision software's best pose of the tag, from the camera
   * @param alternateCameraToTag its other candidate; zero when there is none
   * @param fieldToTag where the tag really is
   */
  public void add(
      Pose3d robot,
      Transform3d bestCameraToTag,
      Transform3d alternateCameraToTag,
      Pose3d fieldToTag) {
    sightings.add(
        new Sighting(
            robot,
            bestCameraToTag,
            alternateCameraToTag.getTranslation().getNorm() > 1e-9 ? alternateCameraToTag : null,
            fieldToTag));
  }

  public int sightings() {
    return sightings.size();
  }

  /**
   * Solves for the mount.
   *
   * @return empty with fewer than three sightings
   */
  public Optional<Result> solve() {
    if (sightings.size() < 3) {
      return Optional.empty();
    }
    // Choose each tag's pose at the mount so far, solve, and again until the choices settle.
    Transform3d mount = configured;
    for (int round = 0; round < 5; round++) {
      boolean changed = false;
      for (Sighting sighting : sightings) {
        changed |= sighting.choose(mount);
      }
      if (round > 0 && !changed) {
        break;
      }
      mount = minimize(mount);
    }
    double[] residuals = residuals(mount);
    return Optional.of(
        new Result(
            mount,
            placeRms(configured),
            placeRms(mount),
            sightings.size(),
            stdDevs(mount, residuals)));
  }

  /** Levenberg-Marquardt from a start, over the sightings' current choices. */
  private Transform3d minimize(Transform3d start) {
    Transform3d mount = start;
    double[] residuals = residuals(mount);
    double cost = sumOfSquares(residuals);
    double damping = 1e-3;
    for (int iteration = 0; iteration < MAX_ITERATIONS; iteration++) {
      double[][] jacobian = jacobian(mount, residuals);
      double[][] normal = new double[6][6];
      double[] gradient = new double[6];
      for (int row = 0; row < residuals.length; row++) {
        for (int a = 0; a < 6; a++) {
          gradient[a] += jacobian[row][a] * residuals[row];
          for (int b = 0; b < 6; b++) {
            normal[a][b] += jacobian[row][a] * jacobian[row][b];
          }
        }
      }
      boolean improved = false;
      while (damping < 1e12) {
        double[][] damped = new double[6][6];
        for (int a = 0; a < 6; a++) {
          damped[a] = normal[a].clone();
          damped[a][a] += damping * Math.max(normal[a][a], 1e-12);
        }
        double[] negativeGradient = new double[6];
        for (int a = 0; a < 6; a++) {
          negativeGradient[a] = -gradient[a];
        }
        double[] step = solve(damped, negativeGradient);
        if (step == null) {
          damping *= 10;
          continue;
        }
        Transform3d next = mount.plus(nudge(step));
        double[] nextResiduals = residuals(next);
        double nextCost = sumOfSquares(nextResiduals);
        if (nextCost < cost) {
          boolean converged = cost - nextCost < 1e-14 * Math.max(1, cost) || norm(step) < 1e-10;
          mount = next;
          residuals = nextResiduals;
          cost = nextCost;
          damping = Math.max(damping / 10, 1e-12);
          improved = !converged;
          break;
        }
        damping *= 10;
      }
      if (!improved) {
        break;
      }
    }
    return mount;
  }

  /**
   * How far each value could be off, from how well the solved mount fits and how varied the
   * sightings are. Sightings from one robot pose share that pose's error, so they count as one: the
   * spread widens by how many sightings each distinct pose holds.
   */
  private double[] stdDevs(Transform3d mount, double[] residuals) {
    double[][] jacobian = jacobian(mount, residuals);
    double[][] normal = new double[6][6];
    for (double[] row : jacobian) {
      for (int a = 0; a < 6; a++) {
        for (int b = 0; b < 6; b++) {
          normal[a][b] += row[a] * row[b];
        }
      }
    }
    // Each sighting's miss across the line of sight has two degrees of freedom, not three.
    int freedom = Math.max(1, sightings.size() * 6 - 6);
    double variance = sumOfSquares(residuals) / freedom;
    variance *= Math.max(1.0, (double) sightings.size() / distinctPoses());
    double[] stdDevs = new double[6];
    for (int a = 0; a < 6; a++) {
      double[] unit = new double[6];
      unit[a] = 1;
      double[] column = solve(normal, unit);
      stdDevs[a] =
          column == null ? Double.POSITIVE_INFINITY : Math.sqrt(Math.max(0, column[a] * variance));
    }
    return stdDevs;
  }

  /** How many of the sightings were taken from robot poses 5 cm or 2° apart from the others. */
  private int distinctPoses() {
    List<Pose3d> distinct = new ArrayList<>();
    for (Sighting sighting : sightings) {
      boolean seen = false;
      for (Pose3d pose : distinct) {
        if (pose.getTranslation().getDistance(sighting.robot.getTranslation()) < 0.05
            && pose.getRotation().relativeTo(sighting.robot.getRotation()).getAngle()
                < Math.toRadians(2)) {
          seen = true;
          break;
        }
      }
      if (!seen) {
        distinct.add(sighting.robot);
      }
    }
    return distinct.size();
  }

  /** A small change to the mount, in its own frame: three of translation, three of rotation. */
  private static Transform3d nudge(double[] delta) {
    return new Transform3d(
        new Translation3d(delta[0], delta[1], delta[2]),
        new Rotation3d(VecBuilder.fill(delta[3], delta[4], delta[5])));
  }

  private double[][] jacobian(Transform3d mount, double[] residuals) {
    double[][] jacobian = new double[residuals.length][6];
    for (int a = 0; a < 6; a++) {
      double[] delta = new double[6];
      delta[a] = STEP;
      double[] plus = residuals(mount.plus(nudge(delta)));
      delta[a] = -STEP;
      double[] minus = residuals(mount.plus(nudge(delta)));
      for (int row = 0; row < residuals.length; row++) {
        jacobian[row][a] = (plus[row] - minus[row]) / (2 * STEP);
      }
    }
    return jacobian;
  }

  /**
   * Every sighting's miss, each part over how well a camera sees it: across the line of sight,
   * along it, then the tag's orientation.
   */
  private double[] residuals(Transform3d mount) {
    double[] residuals = new double[sightings.size() * 7];
    for (int i = 0; i < sightings.size(); i++) {
      sightings.get(i).residualsInto(mount, residuals, 7 * i);
    }
    return residuals;
  }

  /** How far the tags land from their places across the line of sight, root mean square. */
  private double placeRms(Transform3d mount) {
    double sum = 0;
    for (Sighting sighting : sightings) {
      double across = sighting.acrossMiss(mount);
      sum += across * across;
    }
    return Math.sqrt(sum / sightings.size());
  }

  private static double sumOfSquares(double[] values) {
    double sum = 0;
    for (double value : values) {
      sum += value * value;
    }
    return sum;
  }

  private static double norm(double[] values) {
    return Math.sqrt(sumOfSquares(values));
  }

  /** Solves a small square system by elimination with partial pivoting; null if it's singular. */
  private static double @Nullable [] solve(double[][] matrix, double[] right) {
    int n = right.length;
    double[][] a = new double[n][];
    double[] b = right.clone();
    for (int i = 0; i < n; i++) {
      a[i] = matrix[i].clone();
    }
    for (int column = 0; column < n; column++) {
      int pivot = column;
      for (int row = column + 1; row < n; row++) {
        if (Math.abs(a[row][column]) > Math.abs(a[pivot][column])) {
          pivot = row;
        }
      }
      if (Math.abs(a[pivot][column]) < 1e-300) {
        return null;
      }
      double[] swapRow = a[column];
      a[column] = a[pivot];
      a[pivot] = swapRow;
      double swap = b[column];
      b[column] = b[pivot];
      b[pivot] = swap;
      for (int row = column + 1; row < n; row++) {
        double factor = a[row][column] / a[column][column];
        for (int k = column; k < n; k++) {
          a[row][k] -= factor * a[column][k];
        }
        b[row] -= factor * b[column];
      }
    }
    double[] x = new double[n];
    for (int row = n - 1; row >= 0; row--) {
      double sum = b[row];
      for (int k = row + 1; k < n; k++) {
        sum -= a[row][k] * x[k];
      }
      x[row] = sum / a[row][row];
    }
    return x;
  }
}
