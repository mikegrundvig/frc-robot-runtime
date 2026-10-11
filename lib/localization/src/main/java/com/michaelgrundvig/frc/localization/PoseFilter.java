package com.michaelgrundvig.frc.localization;

import java.util.Arrays;
import java.util.List;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.MathUtil;

/**
 * The pose estimator's filter: a Kalman filter over the robot's position and a heading offset. The
 * field heading is the gyro's yaw plus the offset, so the gyro turns the estimate and the filter
 * only corrects the gyro's slow drift. It predicts through odometry steps ({@link #predict}) and
 * corrects with camera measurements ({@link #innovation} then {@link #correct}). What to feed it,
 * when, and whether to believe a measurement is the estimator's business.
 *
 * <p><b>What it holds.</b>
 *
 * <ul>
 *   <li>The state: x and y in meters on the field, and the heading offset in radians.
 *   <li>The state's covariance, P: 3 by 3, stored row by row in the order x, y, heading ({@link
 *       #XX}, {@link #XY} and so on name its cells).
 *   <li>The cross-covariance, C: how each of x, y and the heading offset has come to depend on each
 *       error the filter considers, 3 rows by {@link #consideredCount()} columns ({@link
 *       ConsideredErrors} says which column is which).
 * </ul>
 *
 * <p><b>Errors it considers but never estimates.</b> Some errors repeat: in every image a camera
 * takes, or in every turn the gyro makes. Averaged as if each image's or each turn's error were
 * new, they would shrink the uncertainty to millimeters while the estimate stays centimeters off.
 * So the filter treats them as Schmidt "consider" parameters (Schmidt 1966; Zanetti and D'Souza
 * 2015). Each has a fixed variance, the filter never corrects it, and the filter keeps track of how
 * the estimate's own errors have come to depend on it.
 *
 * <ul>
 *   <li><b>The heading, for a position a camera solved with it.</b> Such a position leans on the
 *       gyro's heading: if the heading is off by an angle, the position is off by that angle times
 *       the distance to the tags, at right angles to them ({@link VisionMeasurement#lever}). So
 *       each fix measures the position and the heading's error together. By default such a fix
 *       still does not correct the heading: the gyro owns it, and only a measurement with a heading
 *       of its own corrects its drift. With {@code positionFixesCorrectHeading}, every fix corrects
 *       it, which is an ordinary full Kalman update.
 *   <li><b>Each IMU's scale.</b> A gyro that reads 0.3% too much turn is 1.1° off after one turn
 *       and 5.4° after five, the same way each time, not a little more at random. The backup IMU
 *       has a scale error of its own: three turns one way on the gyro and three back on the backup
 *       do not cancel.
 *   <li><b>Each camera's turn on its mount</b>, what is left after calibrating it. It moves the
 *       camera's positions just as a heading error does, and its headings by the angle itself.
 *   <li><b>Each camera's shift on its mount:</b> what is left of where it sits, along the robot's
 *       forward and left. It moves every position the camera finds by the same amount on the robot,
 *       so on the field it turns as the robot turns.
 *   <li><b>Each camera's headings' shift:</b> the rest of what its headings share from image to
 *       image, from the tags it sees being a little off the layout. Two tags a third of a meter
 *       apart, each half an inch from where the layout says, turn a multi-tag heading by up to 4°.
 *       It changes as the robot moves and turns and the camera sees other tags from elsewhere, so
 *       the filter lets go of it over a correlation distance and angle.
 *   <li><b>Each tag's placement</b> on the field, along the field and across it. A tag off the
 *       layout puts every position solved from it off by its share of the solve, in every image of
 *       every camera that sees it. Two cameras seeing the same tags share its error, and turning
 *       away and back finds it again.
 * </ul>
 *
 * <p>The considered errors' own variances stay fixed, and two considered errors stay unrelated to
 * each other, since nothing ever corrects them. So a robot standing still, its camera seeing the
 * same tags image after image, still follows any change in where it is, but cannot place itself
 * better than that camera's repeated errors and its tags' allow. Several cameras, each with errors
 * of its own, together do better, but no better than the tags they share.
 */
final class PoseFilter {
  // The state's three values, in the order every row and column here follows. A camera
  // measurement's rows follow the same order: its x, its y, and its heading if it has one.
  static final int X = 0;
  static final int Y = 1;
  static final int HEADING = 2;
  private static final int STATES = 3;

  /** A measurement of position alone has two rows; one with a heading of its own has three. */
  private static final int POSITION_ROWS = 2;

  private static final int POSE_ROWS = 3;

  // The cells of a 3 by 3 matrix stored row by row, named by the row's and the column's value:
  // XH is the covariance of x with the heading.
  private static final int XX = 0;
  private static final int XY = 1;
  private static final int XH = 2;
  private static final int YX = 3;
  private static final int YY = 4;
  private static final int YH = 5;
  private static final int HX = 6;
  private static final int HY = 7;
  private static final int HH = 8;
  private static final int CELLS = 9;

  /**
   * Below this much turn, a step is taken as straight: the arc's formulas divide by the turn, and
   * for a turn this small they give the straight line anyway.
   */
  private static final double STRAIGHT_BELOW_RADIANS = 1e-9;

  /** An uncertainty as big as the field: the estimate knows nothing. */
  private static final PoseCovariance NO_IDEA = PoseCovariance.fromStdDevs(100, 100, Math.PI);

  /**
   * How unsure the filter is of the errors it considers.
   *
   * @param gyroScaleStdDev the gyro's scale error: radians of heading per radian it turns
   * @param backupScaleStdDev the backup IMU's
   * @param cameraTurnStdDevRadians each camera's turn on its mount
   * @param cameraShiftStdDevMeters each camera's shift on its mount, forward and left
   * @param cameraShiftStdDevRadians and of its headings
   * @param tagPlacementStdDevMeters each tag's placement, along x and along y
   * @param shiftCorrelationMeters how far the robot moves before a camera's headings' shift is
   *     mostly a new one: what the filter keeps of it falls by a factor of e over this distance
   * @param shiftCorrelationRadians and over this much turning
   */
  record Considered(
      double gyroScaleStdDev,
      double backupScaleStdDev,
      double cameraTurnStdDevRadians,
      double cameraShiftStdDevMeters,
      double cameraShiftStdDevRadians,
      double tagPlacementStdDevMeters,
      double shiftCorrelationMeters,
      double shiftCorrelationRadians) {}

  /**
   * How far a camera's measurement is from the estimate, and how surprising that is.
   *
   * @param xMeters the position's innovation along the field's x: measured less predicted
   * @param yMeters and along y
   * @param withHeading whether the measurement has a heading of its own
   * @param headingRadians the heading's innovation; NaN for a measurement without a heading
   * @param chiSquared the whole innovation's squared distance from zero, measured in its own
   *     uncertainty (its squared Mahalanobis distance): what the estimator's gate tests
   * @param positionChiSquared the same for the position alone
   * @param stamp which of the filter's innovations this is. {@link #correct(Innovation)} applies
   *     only the latest, because the filter holds the working of only one at a time
   */
  record Innovation(
      double xMeters,
      double yMeters,
      boolean withHeading,
      double headingRadians,
      double chiSquared,
      double positionChiSquared,
      long stamp) {
    double distanceMeters() {
      return Math.hypot(xMeters, yMeters);
    }
  }

  /** How many errors the filter considers, which is how many columns C has. */
  private final int consideredCount;

  /** How many tags' placements have columns of their own. */
  private final int tagCount;

  private final boolean positionFixesCorrectHeading;

  // The considered errors' variances.
  private final double gyroScaleVariance;
  private final double backupScaleVariance;
  private final double headingsShiftVariance;
  private final double tagPlacementVariance;

  /** What a camera's turn and shift on its mount are taken to be until a doubt raises them. */
  private final double usualCameraTurnVariance;

  private final double usualCameraShiftVariance;

  /**
   * Each camera's turn's variance, and its shift's along each axis, by the camera's place. They
   * start as usual, and a doubt about a camera's mount raises them ({@link #setCameraVariances}).
   */
  private final double[] cameraTurnVariance = new double[ConsideredErrors.MAX_CAMERAS];

  private final double[] cameraShiftVariance = new double[ConsideredErrors.MAX_CAMERAS];

  private final double shiftCorrelationMeters;
  private final double shiftCorrelationRadians;

  // The state.
  private double x;
  private double y;
  private double headingOffset;

  /** P, the state's covariance: 3 by 3, row by row. */
  private final double[] covariance = new double[CELLS];

  /** C, the state's cross-covariance with the considered errors: 3 rows of consideredCount. */
  private final double[] crossCovariance;

  // The working of one camera measurement, from its innovation to its correction. The filter holds
  // one measurement's working at a time: innovation() fills these, and correct() reads them. They
  // are fields, not values made for each measurement, so that applying one allocates nothing.

  /** How many rows the measurement has: {@link #POSITION_ROWS} or {@link #POSE_ROWS}. */
  private int rowCount;

  /** The measurement less what the estimate predicts for it: x, y in meters, heading in radians. */
  private final double[] innovation = new double[POSE_ROWS];

  /**
   * H, the measurement's Jacobian with respect to the state: one row for each row of the
   * measurement, one column for each of x, y and the heading offset.
   */
  private final double[] stateSeenBy = new double[CELLS];

  /** The considered errors the measurement sees, with how much each of its rows sees of each. */
  private final SharedErrors shared = new SharedErrors();

  /**
   * The covariance of the state's errors with the measurement's: P H^T, plus what reaches the
   * measurement through the considered errors. 3 rows for the state, a column for each row of the
   * measurement.
   */
  private final double[] stateWithMeasurement = new double[CELLS];

  /** S, the innovation's covariance: rowCount by rowCount, in the top left of a 3 by 3. */
  private final double[] innovationCovariance = new double[CELLS];

  private final double[] innovationCovarianceInverse = new double[CELLS];

  /** K, the gain: 3 rows for the state, a column for each row of the measurement. */
  private final double[] gain = new double[CELLS];

  /** K S, used once in the covariance's update. */
  private final double[] gainTimesInnovationCovariance = new double[CELLS];

  /**
   * The covariance of the measurement's errors with each considered error: a row for each row of
   * the measurement, of consideredCount columns.
   */
  private final double[] measurementWithConsidered;

  /** How many camera innovations the filter has worked out: the latest one's stamp. */
  private long innovations;

  /**
   * A filter in which a position fix leaves the heading to measurements with a heading of their
   * own.
   *
   * @param tags how many tags' placement to keep apart: the field's, each given a place from 0
   */
  PoseFilter(Considered sizes, int tags) {
    this(sizes, tags, false);
  }

  /**
   * @param tags how many tags' placement to keep apart: the field's, each given a place from 0
   * @param positionFixesCorrectHeading whether a position fix also corrects the heading, through
   *     its lever and how the position's error has come to relate to the heading's: a full Kalman
   *     update. False leaves the heading to measurements with a heading of their own: the gyro owns
   *     it.
   */
  PoseFilter(Considered sizes, int tags, boolean positionFixesCorrectHeading) {
    if (tags < 0) {
      throw new IllegalArgumentException("how many tags to keep apart: " + tags);
    }
    tagCount = tags;
    consideredCount = ConsideredErrors.count(tags);
    crossCovariance = new double[STATES * consideredCount];
    measurementWithConsidered = new double[POSE_ROWS * consideredCount];
    this.positionFixesCorrectHeading = positionFixesCorrectHeading;
    gyroScaleVariance = square(sizes.gyroScaleStdDev());
    backupScaleVariance = square(sizes.backupScaleStdDev());
    usualCameraTurnVariance = square(sizes.cameraTurnStdDevRadians());
    usualCameraShiftVariance = square(sizes.cameraShiftStdDevMeters());
    headingsShiftVariance = square(sizes.cameraShiftStdDevRadians());
    tagPlacementVariance = square(sizes.tagPlacementStdDevMeters());
    shiftCorrelationMeters = sizes.shiftCorrelationMeters();
    shiftCorrelationRadians = sizes.shiftCorrelationRadians();
    Arrays.fill(cameraTurnVariance, usualCameraTurnVariance);
    Arrays.fill(cameraShiftVariance, usualCameraShiftVariance);
  }

  double x() {
    return x;
  }

  double y() {
    return y;
  }

  /** The heading offset, in radians: the field heading is the yaw plus this. */
  double headingOffset() {
    return headingOffset;
  }

  /** The estimated pose, given the yaw now. */
  Pose2d pose(double yawRadians) {
    return new Pose2d(x, y, new Rotation2d(yawRadians + headingOffset));
  }

  // ---------------------------------------------------------------------------------------------
  // Being told where the robot is.
  // ---------------------------------------------------------------------------------------------

  /** Takes a reset's position and covariance, and its heading if it gives one. */
  void set(PoseReset reset, double yawRadians) {
    x = reset.pose().getX();
    y = reset.pose().getY();
    PoseCovariance given = reset.covariance();
    if (given.hasHeading()) {
      headingOffset = reset.pose().getRotation().getRadians() - yawRadians;
      covariance[HH] = given.heading();
      forgetDependenceOnConsideredErrors(HEADING);
    }
    setPositionCovariance(given.xx(), given.xy(), given.yy());
  }

  /** A new heading, as sure as given. The position stays where it is, unrelated to the heading. */
  void resetHeading(double newOffsetRadians, double variance) {
    headingOffset = newOffsetRadians;
    covariance[HH] = variance;
    unrelatePositionAndHeading();
    forgetDependenceOnConsideredErrors(HEADING);
  }

  /** Moves the position, leaving it unrelated to the heading and as sure as given each way. */
  void shift(double dxMeters, double dyMeters, double variance) {
    x += dxMeters;
    y += dyMeters;
    setPositionCovariance(variance, 0, variance);
  }

  /** Turns the heading, with a fresh uncertainty for it and for the position. */
  void turn(double radians, double positionVariance, double headingVariance) {
    headingOffset += radians;
    covariance[HH] = headingVariance;
    forgetDependenceOnConsideredErrors(HEADING);
    setPositionCovariance(positionVariance, 0, positionVariance);
  }

  /**
   * Sets one camera's turn's and shift's variances, from now on: raised by a doubt about its mount,
   * or brought back down.
   *
   * <p>A variance that falls takes the estimate's dependence on that error down with it, in
   * proportion. Otherwise the estimate would depend on the error more than an error that small
   * allows, and the covariance of the estimate and the error together would no longer be one any
   * real errors could have (it would not be positive semidefinite).
   */
  void setCameraVariances(int camera, double turnVariance, double shiftVariance) {
    shrinkDependence(ConsideredErrors.cameraTurn(camera), turnVariance, cameraTurnVariance[camera]);
    shrinkDependence(
        ConsideredErrors.cameraShiftForward(camera), shiftVariance, cameraShiftVariance[camera]);
    shrinkDependence(
        ConsideredErrors.cameraShiftLeft(camera), shiftVariance, cameraShiftVariance[camera]);
    cameraTurnVariance[camera] = turnVariance;
    cameraShiftVariance[camera] = shiftVariance;
  }

  /**
   * Scales the state's dependence on one considered error by the square root of how far its
   * variance fell, which is how far its standard deviation fell. A variance that rose or stayed
   * changes nothing.
   */
  private void shrinkDependence(int column, double newVariance, double oldVariance) {
    boolean fell = newVariance < oldVariance;
    if (!fell) {
      return;
    }
    double factor = oldVariance > 0 ? Math.sqrt(newVariance / oldVariance) : 0;
    for (int state = 0; state < STATES; state++) {
      crossCovariance[cross(state, column)] *= factor;
    }
  }

  // ---------------------------------------------------------------------------------------------
  // Predicting through odometry.
  // ---------------------------------------------------------------------------------------------

  /**
   * The Kalman filter's prediction, through part of an odometry step: moves the position by the
   * wheels' travel along the arc the gyro turned through, and grows the uncertainty by what the
   * move adds.
   *
   * @param startYaw the gyro's yaw where this part of the step starts, in radians
   * @param turn how far the gyro turned over it
   * @param forward the wheels' travel forward over it, in the robot's frame at its start, in meters
   * @param left and to the left
   * @param positionNoise the variance to add to each position axis
   * @param headingNoise the variance to add to the heading
   * @param turnedBy what turned the robot. An IMU's scale error grows the heading's error by the
   *     turn. The wheels' own turning errors are already in the heading noise.
   */
  void predict(
      double startYaw,
      double turn,
      double forward,
      double left,
      double positionNoise,
      double headingNoise,
      YawSource turnedBy) {
    // A robot that turns evenly while it drives traces an arc. The straight line from the arc's
    // start to its end, in the robot's frame at the start, is the travel times these two factors:
    // sin(turn)/turn along the way it was heading, (1 - cos(turn))/turn across it. (This is the
    // exact integral, the same as WPILib's Twist2d.exp.)
    double along = forward;
    double across = left;
    boolean turned = Math.abs(turn) > STRAIGHT_BELOW_RADIANS;
    if (turned) {
      double sineOverTurn = Math.sin(turn) / turn;
      double oneMinusCosineOverTurn = (1 - Math.cos(turn)) / turn;
      along = sineOverTurn * forward - oneMinusCosineOverTurn * left;
      across = oneMinusCosineOverTurn * forward + sineOverTurn * left;
    }
    double heading = startYaw + headingOffset;
    double cos = Math.cos(heading);
    double sin = Math.sin(heading);
    double moveX = cos * along - sin * across;
    double moveY = sin * along + cos * across;
    x += moveX;
    y += moveY;

    swingUncertaintyWithHeading(moveX, moveY);
    switch (turnedBy) {
      case GYRO -> addScaleError(turn, ConsideredErrors.GYRO_SCALE, gyroScaleVariance);
      case BACKUP -> addScaleError(turn, ConsideredErrors.BACKUP_SCALE, backupScaleVariance);
      case WHEELS -> {}
    }
    letGoOfHeadingsShifts(Math.hypot(moveX, moveY), Math.abs(turn));

    covariance[XX] += positionNoise;
    covariance[YY] += positionNoise;
    covariance[HH] += headingNoise;
  }

  /**
   * The covariance's prediction, P = F P F^T, for a move over the field.
   *
   * <p>F is the Jacobian of the new state with respect to the old. It is the identity except in its
   * heading column, because a heading offset that is a little off swings the move about its start:
   * an offset off by dθ puts x off by -moveY dθ and y off by moveX dθ. So F is
   *
   * <pre>
   *   1  0  -moveY
   *   0  1   moveX
   *   0  0   1
   * </pre>
   *
   * <p>The lines below are F P F^T multiplied out, each cell changed by what the heading column
   * adds to it. C, the dependence on the considered errors, is predicted by the same F: C = F C.
   */
  private void swingUncertaintyWithHeading(double moveX, double moveY) {
    double xPerHeading = -moveY;
    double yPerHeading = moveX;
    double xWithHeading = covariance[XH];
    double yWithHeading = covariance[YH];
    double headingVariance = covariance[HH];
    covariance[XX] += 2 * xPerHeading * xWithHeading + xPerHeading * xPerHeading * headingVariance;
    covariance[XY] +=
        xPerHeading * yWithHeading
            + yPerHeading * xWithHeading
            + xPerHeading * yPerHeading * headingVariance;
    covariance[YX] = covariance[XY];
    covariance[YY] += 2 * yPerHeading * yWithHeading + yPerHeading * yPerHeading * headingVariance;
    covariance[XH] += xPerHeading * headingVariance;
    covariance[HX] = covariance[XH];
    covariance[YH] += yPerHeading * headingVariance;
    covariance[HY] = covariance[YH];
    for (int column = 0; column < consideredCount; column++) {
      double headingWithError = crossCovariance[cross(HEADING, column)];
      crossCovariance[cross(X, column)] += xPerHeading * headingWithError;
      crossCovariance[cross(Y, column)] += yPerHeading * headingWithError;
    }
  }

  /**
   * An IMU that reads its turns a little large or small adds its scale error times the turn to the
   * heading's error. It is the same error every turn, so the heading comes to depend on it: with s
   * the scale error and v its variance, the heading's error grows by turn times s, which adds
   *
   * <pre>
   *   to the heading's variance          2 turn cov(heading, s) + turn² v
   *   to cov(x, heading)                 turn cov(x, s)
   *   to cov(y, heading)                 turn cov(y, s)
   *   to cov(heading, s)                 turn v
   * </pre>
   *
   * <p>A scale with no variance is not considered at all.
   */
  private void addScaleError(double turn, int scaleColumn, double scaleVariance) {
    boolean considered = scaleVariance > 0;
    if (!considered) {
      return;
    }
    covariance[HH] +=
        2 * turn * crossCovariance[cross(HEADING, scaleColumn)] + turn * turn * scaleVariance;
    covariance[XH] += turn * crossCovariance[cross(X, scaleColumn)];
    covariance[HX] = covariance[XH];
    covariance[YH] += turn * crossCovariance[cross(Y, scaleColumn)];
    covariance[HY] = covariance[YH];
    crossCovariance[cross(HEADING, scaleColumn)] += turn * scaleVariance;
  }

  /**
   * As the robot moves and turns, each camera sees other tags, or the same ones from elsewhere, and
   * its headings' shift becomes a different one. So the state's dependence on each camera's
   * headings' shift decays, by a factor of e for each correlation distance moved and each
   * correlation angle turned. A camera's shift on its mount and each tag's placement stay what they
   * are, and their columns are left alone.
   */
  private void letGoOfHeadingsShifts(double movedMeters, double turnedRadians) {
    double kept =
        Math.exp(-movedMeters / shiftCorrelationMeters - turnedRadians / shiftCorrelationRadians);
    if (kept < 1) {
      for (int camera = 0; camera < ConsideredErrors.MAX_CAMERAS; camera++) {
        for (int state = 0; state < STATES; state++) {
          crossCovariance[cross(state, ConsideredErrors.cameraHeadingsShift(camera))] *= kept;
        }
      }
    }
  }

  // ---------------------------------------------------------------------------------------------
  // Correcting with a camera measurement.
  // ---------------------------------------------------------------------------------------------

  /**
   * Works out a camera measurement's innovation against the estimate, and everything {@link
   * #correct(Innovation)} needs to apply it: the first half of a Kalman update, up to the gain. The
   * estimator gates the innovation between the two halves.
   *
   * @param yawThen the gyro's yaw when the image was taken
   * @param camera the camera's place, from 0 to {@link ConsideredErrors#MAX_CAMERAS} - 1: its turn
   *     and shifts are the ones it shares with its other images. {@link ConsideredErrors#NO_PLACE}
   *     for a camera without a place, whose turn and shifts are taken as new in each image
   * @param tagPlaces each of its tags' place, in the order of its {@link VisionMeasurement#tagIds}:
   *     the placement it shares with every image of that tag. {@link ConsideredErrors#NO_PLACE} for
   *     a tag without a place, whose placement is taken as new in each image
   * @return the innovation, or null when the measurement cannot be compared with the estimate at
   *     all: its uncertainty and the estimate's together leave no spread to measure a distance in
   */
  @Nullable Innovation innovation(
      VisionMeasurement measurement, double yawThen, int camera, int[] tagPlaces) {
    PoseCovariance noise = measurement.covariance();
    rowCount = noise.hasHeading() ? POSE_ROWS : POSITION_ROWS;
    double headingThen = yawThen + headingOffset;

    describeWhatItSees(measurement, headingThen, camera, tagPlaces);
    subtractThePrediction(measurement, headingThen);
    covarianceOfStateWithMeasurement();
    covarianceOfInnovation(noise);

    // The position's own chi-squared, d^T S^-1 d for the 2 by 2 position block of S, with the
    // inverse of a 2 by 2 written out: [a b; b c]^-1 = [c -b; -b a] / (a c - b²).
    double sxx = innovationCovariance[XX];
    double sxy = innovationCovariance[XY];
    double syy = innovationCovariance[YY];
    double determinant = sxx * syy - sxy * sxy;
    // Written so a determinant that is not a number counts as degenerate too.
    boolean positionHasSpread = determinant > 0;
    if (!positionHasSpread) {
      return null;
    }
    double positionChiSquared =
        (syy * innovation[X] * innovation[X]
                - 2 * sxy * innovation[X] * innovation[Y]
                + sxx * innovation[Y] * innovation[Y])
            / determinant;
    double chiSquared = positionChiSquared;
    if (rowCount == POSE_ROWS) {
      if (!Matrices.invert3(innovationCovariance, innovationCovarianceInverse)) {
        return null;
      }
      chiSquared = Matrices.quadratic3(innovationCovarianceInverse, innovation);
    } else {
      innovationCovarianceInverse[XX] = syy / determinant;
      innovationCovarianceInverse[XY] = -sxy / determinant;
      innovationCovarianceInverse[YX] = -sxy / determinant;
      innovationCovarianceInverse[YY] = sxx / determinant;
    }
    workOutGain();
    return new Innovation(
        innovation[X],
        innovation[Y],
        rowCount == POSE_ROWS,
        innovation[HEADING],
        chiSquared,
        positionChiSquared,
        ++innovations);
  }

  /**
   * Fills in H, what each row of the measurement sees of the state, and the considered errors the
   * measurement shares with other measurements.
   *
   * <p>Each position row sees its own axis, and sees the heading through the tags' lever: a heading
   * off by dθ moves a position solved with it by dθ times the robot-to-tags vector, turned a
   * quarter turn. With the lever (lx, ly), H is
   *
   * <pre>
   *   1  0  -ly
   *   0  1   lx
   *   0  0   1     (the heading row, used only by a measurement with a heading of its own)
   * </pre>
   */
  private void describeWhatItSees(
      VisionMeasurement measurement, double headingThen, int camera, int[] tagPlaces) {
    double leverX = measurement.lever().robotToTags().getX();
    double leverY = measurement.lever().robotToTags().getY();
    stateSeenBy[cell(X, X)] = 1;
    stateSeenBy[cell(X, Y)] = 0;
    stateSeenBy[cell(X, HEADING)] = -leverY;
    stateSeenBy[cell(Y, X)] = 0;
    stateSeenBy[cell(Y, Y)] = 1;
    stateSeenBy[cell(Y, HEADING)] = leverX;
    stateSeenBy[cell(HEADING, X)] = 0;
    stateSeenBy[cell(HEADING, Y)] = 0;
    stateSeenBy[cell(HEADING, HEADING)] = 1;

    // A camera with a place shares its errors with its other images, at their own variances. One
    // without a place has the usual variances and no columns.
    int turnColumn = ConsideredErrors.NO_COLUMN;
    int shiftForwardColumn = ConsideredErrors.NO_COLUMN;
    int shiftLeftColumn = ConsideredErrors.NO_COLUMN;
    int headingsShiftColumn = ConsideredErrors.NO_COLUMN;
    double turnVariance = usualCameraTurnVariance;
    double shiftVariance = usualCameraShiftVariance;
    if (ConsideredErrors.hasPlace(camera)) {
      turnColumn = ConsideredErrors.cameraTurn(camera);
      shiftForwardColumn = ConsideredErrors.cameraShiftForward(camera);
      shiftLeftColumn = ConsideredErrors.cameraShiftLeft(camera);
      headingsShiftColumn = ConsideredErrors.cameraHeadingsShift(camera);
      turnVariance = cameraTurnVariance[camera];
      shiftVariance = cameraShiftVariance[camera];
    }
    double cos = Math.cos(headingThen);
    double sin = Math.sin(headingThen);
    shared.clear();
    // The camera's turn on its mount moves its position as a heading error does, and its heading by
    // the turn itself.
    shared.add(turnColumn, turnVariance, -leverY, leverX, 1);
    // Its shift on its mount moves the position along the robot's forward, and along its left, as
    // the robot was turned on the field.
    shared.add(shiftForwardColumn, shiftVariance, cos, sin, 0);
    shared.add(shiftLeftColumn, shiftVariance, -sin, cos, 0);
    // Its headings' shift moves only its heading.
    shared.add(headingsShiftColumn, headingsShiftVariance, 0, 0, 1);
    // A tag that is off the layout moves the position the other way, by the tag's share of it.
    List<Double> shares = measurement.tagShares();
    for (int index = 0; index < shares.size(); index++) {
      double share = shares.get(index);
      int alongXColumn = ConsideredErrors.NO_COLUMN;
      int alongYColumn = ConsideredErrors.NO_COLUMN;
      if (index < tagPlaces.length && isOneOfTheFilterTags(tagPlaces[index])) {
        alongXColumn = ConsideredErrors.tagAlongX(tagPlaces[index]);
        alongYColumn = ConsideredErrors.tagAlongY(tagPlaces[index]);
      }
      shared.add(alongXColumn, tagPlacementVariance, -share, 0, 0);
      shared.add(alongYColumn, tagPlacementVariance, 0, -share, 0);
    }
  }

  private boolean isOneOfTheFilterTags(int tagPlace) {
    return ConsideredErrors.hasPlace(tagPlace) && tagPlace < tagCount;
  }

  /**
   * Fills in the innovation: the measurement less what the estimate predicts for it.
   *
   * <p>The position was solved with the heading its lever carries, and the estimate's heading for
   * that moment may have moved since (a measurement applied late, in the history). What the
   * estimate predicts for such a position is its own position turned about the tags by the
   * difference between the two headings, which is that difference times the lever, turned a quarter
   * turn.
   */
  private void subtractThePrediction(VisionMeasurement measurement, double headingThen) {
    double leverX = measurement.lever().robotToTags().getX();
    double leverY = measurement.lever().robotToTags().getY();
    double solvedHeadingOff =
        MathUtil.angleModulus(measurement.lever().heading().getRadians() - headingThen);
    innovation[X] = measurement.pose().getX() - x - solvedHeadingOff * leverY;
    innovation[Y] = measurement.pose().getY() - y + solvedHeadingOff * leverX;
    if (rowCount == POSE_ROWS) {
      innovation[HEADING] =
          MathUtil.angleModulus(measurement.pose().getRotation().getRadians() - headingThen);
    } else {
      innovation[HEADING] = Double.NaN;
    }
  }

  /**
   * Fills in the covariance of the state's errors with the measurement's. In a plain Kalman filter
   * this is P H^T. Here the measurement also sees considered errors the state depends on, so each
   * adds the state's dependence on it (its column of C) times how much the measurement sees of it.
   * An error without a column is one the state does not depend on.
   */
  private void covarianceOfStateWithMeasurement() {
    for (int state = 0; state < STATES; state++) {
      for (int row = 0; row < rowCount; row++) {
        double sum =
            covariance[cell(state, X)] * stateSeenBy[cell(row, X)]
                + covariance[cell(state, Y)] * stateSeenBy[cell(row, Y)]
                + covariance[cell(state, HEADING)] * stateSeenBy[cell(row, HEADING)];
        for (int error = 0; error < shared.count(); error++) {
          int column = shared.column(error);
          if (ConsideredErrors.hasColumn(column)) {
            sum += crossCovariance[cross(state, column)] * shared.seenBy(row, error);
          }
        }
        stateWithMeasurement[cell(state, row)] = sum;
      }
    }
  }

  /**
   * Fills in S, the innovation's covariance: how far apart the measurement and the estimate's
   * prediction of it are expected to be. In a plain Kalman filter this is H P H^T + R. Here each
   * shared error adds two more things: its own variance, through how much two rows see of it, and
   * its covariance with the state (its column of C), through what the other row sees of the state.
   * An error without a column is unrelated to the state and adds only its own variance.
   *
   * <p>Each cell is S[rowA][rowB]. The first term, H times the covariance of the state with the
   * measurement, already holds H P H^T and one of the two cross terms. The loop adds the other
   * cross term and the errors' own variances.
   */
  private void covarianceOfInnovation(PoseCovariance noise) {
    for (int rowA = 0; rowA < rowCount; rowA++) {
      for (int rowB = 0; rowB < rowCount; rowB++) {
        double sum =
            stateSeenBy[cell(rowA, X)] * stateWithMeasurement[cell(X, rowB)]
                + stateSeenBy[cell(rowA, Y)] * stateWithMeasurement[cell(Y, rowB)]
                + stateSeenBy[cell(rowA, HEADING)] * stateWithMeasurement[cell(HEADING, rowB)];
        for (int error = 0; error < shared.count(); error++) {
          double seenByRowA = shared.seenBy(rowA, error);
          if (seenByRowA != 0) {
            double errorWithRowB = shared.variance(error) * shared.seenBy(rowB, error);
            int column = shared.column(error);
            if (ConsideredErrors.hasColumn(column)) {
              errorWithRowB +=
                  crossCovariance[cross(X, column)] * stateSeenBy[cell(rowB, X)]
                      + crossCovariance[cross(Y, column)] * stateSeenBy[cell(rowB, Y)]
                      + crossCovariance[cross(HEADING, column)] * stateSeenBy[cell(rowB, HEADING)];
            }
            sum += seenByRowA * errorWithRowB;
          }
        }
        innovationCovariance[cell(rowA, rowB)] = sum;
      }
    }
    // R, the measurement's own noise.
    innovationCovariance[XX] += noise.xx();
    innovationCovariance[XY] += noise.xy();
    innovationCovariance[YX] += noise.xy();
    innovationCovariance[YY] += noise.yy();
    if (rowCount == POSE_ROWS) {
      innovationCovariance[HH] += noise.heading();
    }
    makeInnovationCovarianceSymmetric();
  }

  /**
   * Fills in the gain, K = C S^-1, where C is the covariance of the state with the measurement.
   *
   * <p>A measurement without a heading of its own leaves the heading alone unless the filter was
   * made to let every fix correct it: its gain for the heading offset is set to zero. That is not
   * the gain a Kalman filter would choose, which is why {@link #correct} updates the covariance in
   * the form that is right for any gain.
   */
  private void workOutGain() {
    boolean leavesHeadingAlone = rowCount == POSITION_ROWS && !positionFixesCorrectHeading;
    for (int state = 0; state < STATES; state++) {
      for (int row = 0; row < rowCount; row++) {
        double sum = 0;
        for (int k = 0; k < rowCount; k++) {
          sum += stateWithMeasurement[cell(state, k)] * innovationCovarianceInverse[cell(k, row)];
        }
        if (state == HEADING && leavesHeadingAlone) {
          gain[cell(state, row)] = 0;
        } else {
          gain[cell(state, row)] = sum;
        }
      }
    }
  }

  /**
   * Applies the camera measurement last given to {@link #innovation}: the second half of the Kalman
   * update. It corrects the state, and the covariance of what the measurement may correct, while
   * only keeping account of the errors it considers (a Schmidt-Kalman update).
   */
  void correct(Innovation applied) {
    if (applied.stamp() != innovations) {
      throw new IllegalStateException(
          "a camera's innovation is applied right after it's worked out: "
              + (innovations - applied.stamp())
              + " more came between");
    }
    // The state: x = x + K d, with d the innovation.
    x += gainTimesInnovation(X);
    y += gainTimesInnovation(Y);
    headingOffset += gainTimesInnovation(HEADING);

    // The covariance: P = P - K C^T - C K^T + K S K^T, with C the covariance of the state with the
    // measurement. This form is right for any gain, so also for one that leaves the heading alone.
    // (For the full Kalman gain K = C S^-1 the last two terms cancel, leaving the textbook
    // P - K C^T.)
    for (int state = 0; state < STATES; state++) {
      for (int row = 0; row < rowCount; row++) {
        double sum = 0;
        for (int k = 0; k < rowCount; k++) {
          sum += gain[cell(state, k)] * innovationCovariance[cell(k, row)];
        }
        gainTimesInnovationCovariance[cell(state, row)] = sum;
      }
    }
    for (int stateA = 0; stateA < STATES; stateA++) {
      for (int stateB = 0; stateB < STATES; stateB++) {
        double change = 0;
        for (int row = 0; row < rowCount; row++) {
          change +=
              -gain[cell(stateA, row)] * stateWithMeasurement[cell(stateB, row)]
                  - stateWithMeasurement[cell(stateA, row)] * gain[cell(stateB, row)]
                  + gainTimesInnovationCovariance[cell(stateA, row)] * gain[cell(stateB, row)];
        }
        covariance[cell(stateA, stateB)] += change;
      }
    }
    makeCovarianceSymmetric();

    // The dependence on the considered errors: C = C - K M, where M is the covariance of the
    // measurement's errors with each considered error. The measurement relates to an error through
    // the state (H times that error's column of C) and, for an error it sees itself, through the
    // error's own variance times how much each row sees of it.
    for (int row = 0; row < rowCount; row++) {
      for (int column = 0; column < consideredCount; column++) {
        measurementWithConsidered[cross(row, column)] =
            crossCovariance[cross(X, column)] * stateSeenBy[cell(row, X)]
                + crossCovariance[cross(Y, column)] * stateSeenBy[cell(row, Y)]
                + crossCovariance[cross(HEADING, column)] * stateSeenBy[cell(row, HEADING)];
      }
      for (int error = 0; error < shared.count(); error++) {
        int column = shared.column(error);
        if (ConsideredErrors.hasColumn(column)) {
          measurementWithConsidered[cross(row, column)] +=
              shared.variance(error) * shared.seenBy(row, error);
        }
      }
    }
    for (int state = 0; state < STATES; state++) {
      for (int column = 0; column < consideredCount; column++) {
        double change = 0;
        for (int row = 0; row < rowCount; row++) {
          change += gain[cell(state, row)] * measurementWithConsidered[cross(row, column)];
        }
        crossCovariance[cross(state, column)] -= change;
      }
    }
  }

  /** One state's correction: its row of the gain times the innovation. */
  private double gainTimesInnovation(int state) {
    double correction = 0;
    for (int row = 0; row < rowCount; row++) {
      correction += gain[cell(state, row)] * innovation[row];
    }
    return correction;
  }

  // ---------------------------------------------------------------------------------------------
  // Reporting and storing the uncertainty.
  // ---------------------------------------------------------------------------------------------

  /**
   * The covariance, as the estimate reports it: the position's block and the heading's variance,
   * with rounding that would make it impossible (a variance a hair below zero, x and y varying
   * together a hair more than their variances allow) taken out.
   */
  PoseCovariance covariance() {
    if (!Double.isFinite(covariance[XX])
        || !Double.isFinite(covariance[XY])
        || !Double.isFinite(covariance[YY])
        || !Double.isFinite(covariance[HH])) {
      // Only a bug gets here, since every input is checked on the way in. Say the estimate knows
      // nothing, rather than throw in the robot loop.
      return NO_IDEA;
    }
    double xx = Math.max(0, covariance[XX]);
    double yy = Math.max(0, covariance[YY]);
    double mostXyCanBe = Math.sqrt(xx * yy);
    double xy = Math.clamp(covariance[XY], -mostXyCanBe, mostXyCanBe);
    return new PoseCovariance(xx, xy, yy, Math.max(0, covariance[HH]));
  }

  /** How many errors it considers: the two IMUs' scales, each camera's four, and each tag's two. */
  int consideredCount() {
    return consideredCount;
  }

  /**
   * How many numbers its uncertainty takes to store: P's nine, then C's three rows, then each
   * camera's turn variance, then each camera's shift variance. {@link #copyUncertaintyTo} writes
   * them in that order and {@link #restore} reads them back.
   */
  int uncertaintyLength() {
    return CELLS + crossCovariance.length + 2 * ConsideredErrors.MAX_CAMERAS;
  }

  /** Copies the uncertainty, {@link #uncertaintyLength()} numbers, for the history to keep. */
  void copyUncertaintyTo(double[] into, int at) {
    System.arraycopy(covariance, 0, into, at, CELLS);
    int crossAt = at + CELLS;
    System.arraycopy(crossCovariance, 0, into, crossAt, crossCovariance.length);
    int turnVariancesAt = crossAt + crossCovariance.length;
    System.arraycopy(cameraTurnVariance, 0, into, turnVariancesAt, ConsideredErrors.MAX_CAMERAS);
    int shiftVariancesAt = turnVariancesAt + ConsideredErrors.MAX_CAMERAS;
    System.arraycopy(cameraShiftVariance, 0, into, shiftVariancesAt, ConsideredErrors.MAX_CAMERAS);
  }

  /**
   * Puts the filter back to a state, and to an uncertainty that {@link #copyUncertaintyTo} kept.
   */
  void restore(double x, double y, double headingOffset, double[] uncertainty, int at) {
    this.x = x;
    this.y = y;
    this.headingOffset = headingOffset;
    System.arraycopy(uncertainty, at, covariance, 0, CELLS);
    int crossAt = at + CELLS;
    System.arraycopy(uncertainty, crossAt, crossCovariance, 0, crossCovariance.length);
    int turnVariancesAt = crossAt + crossCovariance.length;
    System.arraycopy(
        uncertainty, turnVariancesAt, cameraTurnVariance, 0, ConsideredErrors.MAX_CAMERAS);
    int shiftVariancesAt = turnVariancesAt + ConsideredErrors.MAX_CAMERAS;
    System.arraycopy(
        uncertainty, shiftVariancesAt, cameraShiftVariance, 0, ConsideredErrors.MAX_CAMERAS);
  }

  // ---------------------------------------------------------------------------------------------
  // The matrices' layouts.
  // ---------------------------------------------------------------------------------------------

  /** Where a cell of a 3 by 3 matrix sits in its array, stored row by row. */
  private static int cell(int row, int column) {
    return row * STATES + column;
  }

  /**
   * Where a cell of a matrix with one column for each considered error sits in its array, stored
   * row by row: C, whose rows are the state's, and the measurement's covariance with the considered
   * errors, whose rows are the measurement's.
   */
  private int cross(int row, int column) {
    return row * consideredCount + column;
  }

  /**
   * Sets the position's own block of P, and leaves the position unrelated to the heading and to
   * every considered error: it was just set afresh, so nothing that came before bears on it.
   */
  private void setPositionCovariance(double xx, double xy, double yy) {
    covariance[XX] = xx;
    covariance[XY] = xy;
    covariance[YX] = xy;
    covariance[YY] = yy;
    unrelatePositionAndHeading();
    forgetDependenceOnConsideredErrors(X);
    forgetDependenceOnConsideredErrors(Y);
  }

  private void unrelatePositionAndHeading() {
    covariance[XH] = 0;
    covariance[HX] = 0;
    covariance[YH] = 0;
    covariance[HY] = 0;
  }

  /** Clears one state's row of C: that state was just set afresh. */
  private void forgetDependenceOnConsideredErrors(int state) {
    Arrays.fill(crossCovariance, cross(state, 0), cross(state + 1, 0), 0);
  }

  private void makeCovarianceSymmetric() {
    covariance[XY] = covariance[YX] = (covariance[XY] + covariance[YX]) / 2;
    covariance[XH] = covariance[HX] = (covariance[XH] + covariance[HX]) / 2;
    covariance[YH] = covariance[HY] = (covariance[YH] + covariance[HY]) / 2;
  }

  private void makeInnovationCovarianceSymmetric() {
    for (int rowA = 0; rowA < rowCount; rowA++) {
      for (int rowB = rowA + 1; rowB < rowCount; rowB++) {
        double mean =
            (innovationCovariance[cell(rowA, rowB)] + innovationCovariance[cell(rowB, rowA)]) / 2;
        innovationCovariance[cell(rowA, rowB)] = mean;
        innovationCovariance[cell(rowB, rowA)] = mean;
      }
    }
  }

  private static double square(double value) {
    return value * value;
  }
}
