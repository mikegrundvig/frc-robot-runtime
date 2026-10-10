package com.michaelgrundvig.frc.localization;

import java.util.Arrays;
import java.util.List;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.MathUtil;

/**
 * The pose estimator's filter: the robot's position and a heading offset (the field heading is the
 * gyro's yaw plus the offset), with a 3x3 covariance of all three and how they relate. It predicts
 * through odometry steps and corrects with measurements; what to feed it, when, and whether to
 * believe a measurement is the estimator's business.
 *
 * <p><b>Errors it considers but never estimates.</b> Some errors repeat: in every image a camera
 * takes, or in every turn the gyro makes. Averaged as if each image's or turn's error were new,
 * they would shrink the uncertainty to millimeters while the estimate stays centimeters off. So the
 * filter keeps track of them as Schmidt "consider" parameters (Schmidt 1966; Zanetti and D'Souza
 * 2015): each has a fixed uncertainty, the filter never corrects it, and it follows how the
 * estimate's own errors have come to depend on it.
 *
 * <ul>
 *   <li><b>The heading, for a position a camera solved with it.</b> Such a position leans on the
 *       gyro's heading: off by an angle, it is off by the angle times the distance to the tags,
 *       square to them ({@link VisionMeasurement#lever}). So each fix measures the position and the
 *       heading's error together (its H is [I | J d], d the robot-to-tags vector and J a quarter
 *       turn), but by default such a fix doesn't correct the heading: the gyro owns it, and a
 *       measurement with a heading of its own corrects its drift, with all three of its rows (its
 *       position too, through how the position's error has come to depend on the heading's). With
 *       {@code fixesTurnHeading}, every fix corrects it: a full update.
 *   <li><b>Each IMU's scale:</b> a gyro that reads 0.3% too much turn is 1.1° off after a turn and
 *       5.4° after five, the same way each time, not a little more at random. The backup IMU's is
 *       its own: three turns one way on the gyro and three back on the backup don't cancel.
 *   <li><b>Each camera's turn on its mount</b>, what's left after calibrating it: it moves the
 *       camera's positions just as a heading error does, and its headings by the angle itself.
 *   <li><b>Each camera's shift on its mount:</b> what's left of where it sits, along the robot's
 *       forward and left. It moves every position the camera finds by the same amount on the robot,
 *       so on the field it turns as the robot turns.
 *   <li><b>Each camera's headings' shift:</b> the rest of what its headings share from image to
 *       image, the tags it sees off the layout turning its multi-tag solves. Two tags a third of a
 *       meter apart, each half an inch from where the layout says, turn a multi-tag heading by up
 *       to 4°. It changes as the robot moves and turns and the camera sees other tags from
 *       elsewhere, so the filter lets go of it over a correlation distance and angle.
 *   <li><b>Each tag's placement</b> on the field, along the field and across it: a tag off the
 *       layout puts every position solved from it off by its share of the solve, in every image of
 *       every camera that sees it. Two cameras seeing the same tags share its error, and turning
 *       away and back finds it again.
 * </ul>
 *
 * <p>Standing still, a camera seeing the same tags image after image still follows any change in
 * where the robot is, but can't place it better than its own repeated errors and its tags' allow;
 * several cameras, each with its own, together do better, but no better than the tags they share.
 *
 * <p>The covariance is row-major: x, y, offset. How the estimate relates to the considered errors
 * is a 3 by {@link #considered()} block, row-major too: their own variances stay fixed, and two
 * considered errors stay unrelated, since nothing ever corrects them. How many there are depends on
 * how many tags the field has.
 */
final class PoseFilter {
  /** The most cameras whose errors the filter keeps apart; any more are treated as independent. */
  static final int MAX_CAMERAS = 8;

  /**
   * How many errors the filter considers for each camera: its turn, its shift forward and left, and
   * its headings' shift.
   */
  private static final int PER_CAMERA = 4;

  /**
   * Where a camera's headings' shift sits among its considered errors, after its turn and shift.
   */
  private static final int HEADINGS_SHIFT = 3;

  /** And for each tag: its placement along x and along y. */
  private static final int PER_TAG = 2;

  /**
   * How many errors the filter considers: the gyro's scale and the backup IMU's, then each camera's
   * turn and shifts, then each tag's placement.
   */
  private final int considered;

  /** How many tags' placement it keeps apart. */
  private final int tags;

  private static final int SCALE = 0;
  private static final int BACKUP_SCALE = 1;

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

  private final double scaleVariance;
  private final double backupScaleVariance;
  private final double turnVariance;
  private final double shiftVariance;
  private final boolean fixesTurnHeading;
  private final double headingShiftVariance;
  private final double tagVariance;
  private final double shiftMeters;
  private final double shiftRadians;

  private double x;
  private double y;
  private double offset;
  private final double[] covariance = new double[9];

  /** How each of x, y, and the offset relates to each considered error: 3 by considered. */
  private final double[] related;

  /**
   * Each camera's turn's variance, and its shift's along each axis: as considered, unless a doubt
   * about its mount has raised them ({@link #setCameraVariances}).
   */
  private final double[] cameraTurnVariance = new double[MAX_CAMERAS];

  private final double[] cameraShiftVariance = new double[MAX_CAMERAS];

  // Scratch space for a camera measurement, from its innovation to its correction: two rows for
  // its position, a third for its heading if it has one. Row-major, three columns each.
  private int rows;
  private final double[] innovation3 = new double[3];
  private final double[] h3 = new double[9]; // H: what each row measures of x, y, and the offset
  // The considered errors it shares with other measurements: its camera's, and its tags'. For each,
  // its place among the considered errors (-1 for one without a place, new in each image), its
  // variance, and what each row measures of it (row-major, a row of `measuredCapacity` each).
  private int measured;
  private int measuredCapacity = PER_CAMERA + PER_TAG * 8;
  private int[] measuredColumn = new int[measuredCapacity];
  private double[] measuredVariance = new double[measuredCapacity];
  private double[] hMeasured = new double[3 * measuredCapacity];
  private final double[] hRelated; // H times the relations: rows x considered
  private final double[] ph3 = new double[9]; // P H^T: 3 x rows
  private final double[] s3 = new double[9]; // S = H P H^T + R: rows x rows
  private final double[] sInverse3 = new double[9];
  private final double[] gain = new double[9]; // K = P H^T S^-1: 3 x rows
  private final double[] product = new double[9];

  /** How many camera innovations the filter has worked out: the latest's stamp. */
  private long innovations;

  /**
   * @param tags how many tags' placement to keep apart: the field's, each given a place from 0
   */
  PoseFilter(Considered sizes, int tags) {
    this(sizes, tags, false);
  }

  /**
   * @param tags how many tags' placement to keep apart: the field's, each given a place from 0
   * @param fixesTurnHeading whether a position fix also corrects the heading, through its lever and
   *     how the position's error has come to relate to the heading's: a full Kalman update. False
   *     leaves the heading to measurements with a heading of their own: the gyro owns it.
   */
  PoseFilter(Considered sizes, int tags, boolean fixesTurnHeading) {
    if (tags < 0) {
      throw new IllegalArgumentException("how many tags to keep apart: " + tags);
    }
    this.tags = tags;
    considered = 2 + PER_CAMERA * MAX_CAMERAS + PER_TAG * tags;
    related = new double[3 * considered];
    hRelated = new double[3 * considered];
    this.fixesTurnHeading = fixesTurnHeading;
    scaleVariance = square(sizes.gyroScaleStdDev());
    backupScaleVariance = square(sizes.backupScaleStdDev());
    turnVariance = square(sizes.cameraTurnStdDevRadians());
    shiftVariance = square(sizes.cameraShiftStdDevMeters());
    headingShiftVariance = square(sizes.cameraShiftStdDevRadians());
    tagVariance = square(sizes.tagPlacementStdDevMeters());
    shiftMeters = sizes.shiftCorrelationMeters();
    shiftRadians = sizes.shiftCorrelationRadians();
    Arrays.fill(cameraTurnVariance, turnVariance);
    Arrays.fill(cameraShiftVariance, shiftVariance);
  }

  /**
   * Sets one camera's turn's and shift's variances, from now on: raised by a doubt about its mount,
   * or back down. A variance that falls takes the estimate's relation to the error down with it, in
   * proportion, so the joint covariance stays positive semidefinite (its Schur complement is
   * unchanged).
   */
  void setCameraVariances(int camera, double turn, double shift) {
    scaleRelations(turnColumn(camera), turn, cameraTurnVariance[camera]);
    scaleRelations(turnColumn(camera) + 1, shift, cameraShiftVariance[camera]);
    scaleRelations(turnColumn(camera) + 2, shift, cameraShiftVariance[camera]);
    cameraTurnVariance[camera] = turn;
    cameraShiftVariance[camera] = shift;
  }

  private void scaleRelations(int column, double to, double from) {
    if (to < from) {
      double factor = from > 0 ? Math.sqrt(to / from) : 0;
      for (int row = 0; row < 3; row++) {
        related[row * considered + column] *= factor;
      }
    }
  }

  /**
   * How far a camera's measurement is from the estimate, and how surprising that is.
   *
   * @param x the position's innovation, meters
   * @param y the position's innovation, meters
   * @param withHeading whether the measurement has a heading
   * @param heading the heading's innovation, radians; NaN for a measurement without heading
   * @param chiSquared the innovation's squared distance under its uncertainty, all of it
   * @param positionChiSquared the same for the position alone
   * @param stamp which of the filter's innovations it is: {@link #correct(Innovation)} applies only
   *     the latest, whose gain the filter still holds
   */
  record Innovation(
      double x,
      double y,
      boolean withHeading,
      double heading,
      double chiSquared,
      double positionChiSquared,
      long stamp) {
    double meters() {
      return Math.hypot(x, y);
    }
  }

  double x() {
    return x;
  }

  double y() {
    return y;
  }

  double offset() {
    return offset;
  }

  Pose2d pose(double yaw) {
    return new Pose2d(x, y, new Rotation2d(yaw + offset));
  }

  /** Takes a reset's position and covariance, and its heading if it gives one. */
  void set(PoseReset reset, double yaw) {
    x = reset.pose().getX();
    y = reset.pose().getY();
    PoseCovariance given = reset.covariance();
    if (given.hasHeading()) {
      offset = reset.pose().getRotation().getRadians() - yaw;
      covariance[8] = given.heading();
      unrelate(2);
    }
    setPosition(given.xx(), given.xy(), given.yy());
  }

  /** A new heading, as sure as given, the position left where it is and unrelated to it. */
  void resetHeading(double newOffset, double variance) {
    offset = newOffset;
    covariance[8] = variance;
    clearCrossTerms();
    unrelate(2);
  }

  /** Moves the position, unrelated to the heading and as sure as given in each direction. */
  void shift(double dx, double dy, double variance) {
    x += dx;
    y += dy;
    setPosition(variance, 0, variance);
  }

  /** Turns the heading, with a fresh uncertainty for it and for the position. */
  void turn(double radians, double positionVariance, double headingVariance) {
    offset += radians;
    covariance[8] = headingVariance;
    unrelate(2);
    setPosition(positionVariance, 0, positionVariance);
  }

  /**
   * Moves through part of an odometry step: the wheels' travel along the arc the gyro turned
   * through, starting at a yaw, and the uncertainty the move adds.
   *
   * @param startYaw the gyro's yaw where this part of the step starts
   * @param turn how far the gyro turned over it
   * @param dx the wheels' travel forward over it, in the robot's frame at its start
   * @param dy and to the left
   * @param positionNoise variance to add to each position axis
   * @param headingNoise variance to add to the heading
   * @param turnedBy what turned it: an IMU's scale error grows the heading's error by the turn; the
   *     wheels' own turning errors are in the heading noise
   */
  void predict(
      double startYaw,
      double turn,
      double dx,
      double dy,
      double positionNoise,
      double headingNoise,
      YawSource turnedBy) {
    double heading = startYaw + offset;
    double along = dx;
    double across = dy;
    if (Math.abs(turn) > 1e-9) {
      double s = Math.sin(turn) / turn;
      double c = (1 - Math.cos(turn)) / turn;
      along = s * dx - c * dy;
      across = c * dx + s * dy;
    }
    double cos = Math.cos(heading);
    double sin = Math.sin(heading);
    double moveX = cos * along - sin * across;
    double moveY = sin * along + cos * across;
    x += moveX;
    y += moveY;

    // P = F P F^T, where F = I plus a heading column: turning the offset swings the move.
    double a = -moveY;
    double b = moveX;
    double p02 = covariance[2];
    double p12 = covariance[5];
    double p22 = covariance[8];
    covariance[0] += 2 * a * p02 + a * a * p22;
    covariance[1] += a * p12 + b * p02 + a * b * p22;
    covariance[3] = covariance[1];
    covariance[4] += 2 * b * p12 + b * b * p22;
    covariance[2] += a * p22;
    covariance[6] = covariance[2];
    covariance[5] += b * p22;
    covariance[7] = covariance[5];
    // The same F for how the position relates to the considered errors.
    for (int k = 0; k < considered; k++) {
      double withHeading = related[2 * considered + k];
      related[k] += a * withHeading;
      related[considered + k] += b * withHeading;
    }

    // An IMU reading its turns a little large or small adds its scale error times the turn to the
    // heading's error: the same error every turn, so it is related to all the others.
    int scale = turnedBy == YawSource.GYRO ? SCALE : BACKUP_SCALE;
    double variance = turnedBy == YawSource.GYRO ? scaleVariance : backupScaleVariance;
    if (turnedBy != YawSource.WHEELS && variance > 0) {
      covariance[8] += 2 * turn * related[2 * considered + scale] + turn * turn * variance;
      covariance[2] += turn * related[scale];
      covariance[6] = covariance[2];
      covariance[5] += turn * related[considered + scale];
      covariance[7] = covariance[5];
      related[2 * considered + scale] += turn * variance;
    }

    // As the robot moves and turns, each camera sees other tags, or the same ones from elsewhere,
    // and its headings' shift becomes a different one. Its mount's shift, and each tag's
    // placement, stay what they are.
    double kept = Math.exp(-Math.hypot(moveX, moveY) / shiftMeters - Math.abs(turn) / shiftRadians);
    if (kept < 1) {
      for (int slot = 0; slot < MAX_CAMERAS; slot++) {
        for (int row = 0; row < 3; row++) {
          related[row * considered + turnColumn(slot) + HEADINGS_SHIFT] *= kept;
        }
      }
    }

    covariance[0] += positionNoise;
    covariance[4] += positionNoise;
    covariance[8] += headingNoise;
  }

  /**
   * A camera measurement's innovation against the estimate, readying {@link #correct(Innovation)}
   * to apply it.
   *
   * @param yawThen the gyro's yaw when the image was taken
   * @param camera which camera, from 0 to {@link #MAX_CAMERAS} - 1, whose turn and shifts it shares
   *     with the camera's other images; -1 for one without a place, whose turn and shifts are taken
   *     as new in each image
   * @param tags each of its tags' place, as the filter was made to keep them apart, in the order of
   *     its {@link VisionMeasurement#tagIds}: the placement it shares with every image of the tag;
   *     -1 for one without a place, whose placement is taken as new in each image
   * @return null if its uncertainty and the estimate's together are degenerate
   */
  @Nullable Innovation innovation(
      VisionMeasurement measurement, double yawThen, int camera, int[] tags) {
    PoseCovariance noise = measurement.covariance();
    rows = noise.hasHeading() ? 3 : 2;
    // H: each position row sees its own axis, and the heading through the tags' lever: a heading
    // off by dθ moves the fix dθ times the robot-to-tags vector, turned a quarter turn.
    double leverX = measurement.lever().robotToTags().getX();
    double leverY = measurement.lever().robotToTags().getY();
    h3[0] = 1;
    h3[1] = 0;
    h3[2] = -leverY;
    h3[3] = 0;
    h3[4] = 1;
    h3[5] = leverX;
    h3[6] = 0;
    h3[7] = 0;
    h3[8] = 1;
    // And what it shares with other measurements. The camera's turn moves its position as a
    // heading error does, and its heading by the turn itself; its mount's shift moves the position
    // along the robot's forward and left, as the robot is turned; its headings' shift, the heading.
    // A tag off the layout moves the position the other way, by the tag's share of it.
    measured = 0;
    double facing = yawThen + offset;
    double cos = Math.cos(facing);
    double sin = Math.sin(facing);
    int first = camera >= 0 ? turnColumn(camera) : -1;
    double turned = camera >= 0 ? cameraTurnVariance[camera] : turnVariance;
    double shifted = camera >= 0 ? cameraShiftVariance[camera] : shiftVariance;
    measure(first, turned, -leverY, leverX, 1);
    measure(first >= 0 ? first + 1 : -1, shifted, cos, sin, 0);
    measure(first >= 0 ? first + 2 : -1, shifted, -sin, cos, 0);
    measure(first >= 0 ? first + HEADINGS_SHIFT : -1, headingShiftVariance, 0, 0, 1);
    List<Double> shares = measurement.tagShares();
    for (int i = 0; i < shares.size(); i++) {
      double share = shares.get(i);
      int tag = i < tags.length && tags[i] >= 0 && tags[i] < this.tags ? tagColumn(tags[i]) : -1;
      measure(tag, tagVariance, -share, 0, 0);
      measure(tag >= 0 ? tag + 1 : -1, tagVariance, 0, -share, 0);
    }
    // The position was solved with the heading of its lever, and the estimate's heading then may
    // have moved since (a measurement applied late, in the history): what the estimate predicts
    // for it is its position turned by the difference, through the lever.
    double solvedOff =
        MathUtil.angleModulus(measurement.lever().heading().getRadians() - (yawThen + offset));
    innovation3[0] = measurement.pose().getX() - x - solvedOff * leverY;
    innovation3[1] = measurement.pose().getY() - y + solvedOff * leverX;
    innovation3[2] =
        rows == 3
            ? MathUtil.angleModulus(
                measurement.pose().getRotation().getRadians() - (yawThen + offset))
            : Double.NaN;
    // P H^T, for the estimate's rows: through the estimate's own covariance, and through how it
    // relates to the errors the measurement shares.
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < rows; j++) {
        double sum =
            covariance[i * 3] * h3[j * 3]
                + covariance[i * 3 + 1] * h3[j * 3 + 1]
                + covariance[i * 3 + 2] * h3[j * 3 + 2];
        for (int m = 0; m < measured; m++) {
          int k = measuredColumn[m];
          if (k >= 0) {
            sum += related[i * considered + k] * hMeasured[j * measuredCapacity + m];
          }
        }
        ph3[i * 3 + j] = sum;
      }
    }
    // S = H P H^T + R: the estimate's part, then the shared errors': how they relate to the
    // estimate, and their own variances. One without a place among the considered errors is still
    // in every image: it's taken as new in each, unrelated to anything.
    for (int i = 0; i < rows; i++) {
      for (int j = 0; j < rows; j++) {
        double sum = h3[i * 3] * ph3[j] + h3[i * 3 + 1] * ph3[3 + j] + h3[i * 3 + 2] * ph3[6 + j];
        for (int m = 0; m < measured; m++) {
          double hm = hMeasured[i * measuredCapacity + m];
          if (hm != 0) {
            double own = measuredVariance[m] * hMeasured[j * measuredCapacity + m];
            int k = measuredColumn[m];
            if (k >= 0) {
              own +=
                  related[k] * h3[j * 3]
                      + related[considered + k] * h3[j * 3 + 1]
                      + related[2 * considered + k] * h3[j * 3 + 2];
            }
            sum += hm * own;
          }
        }
        s3[i * 3 + j] = sum;
      }
    }
    s3[0] += noise.xx();
    s3[1] += noise.xy();
    s3[3] += noise.xy();
    s3[4] += noise.yy();
    if (rows == 3) {
      s3[8] += noise.heading();
    }
    symmetrizeS();
    // The position's own agreement, from the position block of S.
    double s00 = s3[0];
    double s01 = s3[1];
    double s11 = s3[4];
    double determinant = s00 * s11 - s01 * s01;
    if (!(determinant > 0)) {
      return null;
    }
    double positionChiSquared =
        (s11 * innovation3[0] * innovation3[0]
                - 2 * s01 * innovation3[0] * innovation3[1]
                + s00 * innovation3[1] * innovation3[1])
            / determinant;
    double chiSquared = positionChiSquared;
    if (rows == 3) {
      if (!Matrices.invert3(s3, sInverse3)) {
        return null;
      }
      chiSquared = Matrices.quadratic3(sInverse3, innovation3);
    } else {
      sInverse3[0] = s11 / determinant;
      sInverse3[1] = -s01 / determinant;
      sInverse3[3] = -s01 / determinant;
      sInverse3[4] = s00 / determinant;
    }
    // K = P H^T S^-1. A measurement without a heading of its own leaves the heading alone: its gain
    // for the offset is zero, as it is for every considered error.
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < rows; j++) {
        double sum = 0;
        for (int k = 0; k < rows; k++) {
          sum += ph3[i * 3 + k] * sInverse3[k * 3 + j];
        }
        gain[i * 3 + j] = i == 2 && rows == 2 && !fixesTurnHeading ? 0 : sum;
      }
    }
    return new Innovation(
        innovation3[0],
        innovation3[1],
        rows == 3,
        innovation3[2],
        chiSquared,
        positionChiSquared,
        ++innovations);
  }

  /**
   * One error the measurement shares with others: its place among the considered errors (-1 for
   * none), its variance, and what the measurement's x, y, and heading rows see of it. One with no
   * variance is left out: nothing can relate to it.
   */
  private void measure(int column, double variance, double hx, double hy, double hHeading) {
    if (!(variance > 0)) {
      return;
    }
    if (measured == measuredCapacity) {
      int capacity = 2 * measuredCapacity;
      double[] h = new double[3 * capacity];
      for (int row = 0; row < 3; row++) {
        System.arraycopy(hMeasured, row * measuredCapacity, h, row * capacity, measured);
      }
      hMeasured = h;
      measuredColumn = Arrays.copyOf(measuredColumn, capacity);
      measuredVariance = Arrays.copyOf(measuredVariance, capacity);
      measuredCapacity = capacity;
    }
    measuredColumn[measured] = column;
    measuredVariance[measured] = variance;
    hMeasured[measured] = hx;
    hMeasured[measuredCapacity + measured] = hy;
    hMeasured[2 * measuredCapacity + measured] = hHeading;
    measured++;
  }

  /**
   * Applies the camera measurement last given to {@link #innovation}: a Kalman update of what it
   * may correct, considering the rest.
   */
  void correct(Innovation innovation) {
    if (innovation.stamp() != innovations) {
      throw new IllegalStateException(
          "a camera's innovation is applied right after it's worked out: "
              + (innovations - innovation.stamp())
              + " more came between");
    }
    for (int i = 0; i < 3; i++) {
      double move = 0;
      for (int j = 0; j < rows; j++) {
        move += gain[i * 3 + j] * innovation3[j];
      }
      if (i == 0) {
        x += move;
      } else if (i == 1) {
        y += move;
      } else {
        offset += move;
      }
    }
    // P = P - K C^T - C K^T + K S K^T, with C = P H^T: right for any gain, so for one that leaves
    // the heading or a considered error alone too. (For the full Kalman gain, the last two terms
    // cancel to P - K C^T.)
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < rows; j++) {
        double sum = 0;
        for (int k = 0; k < rows; k++) {
          sum += gain[i * 3 + k] * s3[k * 3 + j];
        }
        product[i * 3 + j] = sum; // K S
      }
    }
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        double change = 0;
        for (int k = 0; k < rows; k++) {
          change +=
              -gain[i * 3 + k] * ph3[j * 3 + k]
                  - ph3[i * 3 + k] * gain[j * 3 + k]
                  + product[i * 3 + k] * gain[j * 3 + k];
        }
        covariance[i * 3 + j] += change;
      }
    }
    symmetrize();
    // How the estimate relates to each considered error changes by the gain times how the
    // measurement related to it: C's considered rows, H times P's considered columns, and for an
    // error it shares, its own variance through what it measures of it.
    for (int r = 0; r < rows; r++) {
      for (int k = 0; k < considered; k++) {
        hRelated[r * considered + k] =
            related[k] * h3[r * 3]
                + related[considered + k] * h3[r * 3 + 1]
                + related[2 * considered + k] * h3[r * 3 + 2];
      }
      for (int m = 0; m < measured; m++) {
        int k = measuredColumn[m];
        if (k >= 0) {
          hRelated[r * considered + k] += measuredVariance[m] * hMeasured[r * measuredCapacity + m];
        }
      }
    }
    for (int i = 0; i < 3; i++) {
      for (int k = 0; k < considered; k++) {
        double change = 0;
        for (int r = 0; r < rows; r++) {
          change += gain[i * 3 + r] * hRelated[r * considered + k];
        }
        related[i * considered + k] -= change;
      }
    }
  }

  /** The covariance, as the estimate reports it. */
  PoseCovariance covariance() {
    if (!Double.isFinite(covariance[0])
        || !Double.isFinite(covariance[1])
        || !Double.isFinite(covariance[4])
        || !Double.isFinite(covariance[8])) {
      // Only a bug gets here (every input is checked on the way in): say the estimate knows
      // nothing, rather than throw in the robot loop.
      return NO_IDEA;
    }
    double xx = Math.max(0, covariance[0]);
    double yy = Math.max(0, covariance[4]);
    double limit = Math.sqrt(xx * yy);
    double xy = Math.clamp(covariance[1], -limit, limit);
    return new PoseCovariance(xx, xy, yy, Math.max(0, covariance[8]));
  }

  /** How many errors it considers: the IMUs' scales, each camera's four, and each tag's two. */
  int considered() {
    return considered;
  }

  /**
   * How many numbers its uncertainty takes to store: the covariance, its relations, and each
   * camera's turn's and shift's variances.
   */
  int uncertainty() {
    return 9 + 3 * considered + 2 * MAX_CAMERAS;
  }

  /** Copies the uncertainty, {@link #uncertainty()} numbers, for the history to keep. */
  void copyUncertaintyTo(double[] into, int at) {
    System.arraycopy(covariance, 0, into, at, 9);
    System.arraycopy(related, 0, into, at + 9, related.length);
    int cameras = at + 9 + related.length;
    System.arraycopy(cameraTurnVariance, 0, into, cameras, MAX_CAMERAS);
    System.arraycopy(cameraShiftVariance, 0, into, cameras + MAX_CAMERAS, MAX_CAMERAS);
  }

  void restore(double x, double y, double offset, double[] uncertainties, int at) {
    this.x = x;
    this.y = y;
    this.offset = offset;
    System.arraycopy(uncertainties, at, covariance, 0, 9);
    System.arraycopy(uncertainties, at + 9, related, 0, related.length);
    int cameras = at + 9 + related.length;
    System.arraycopy(uncertainties, cameras, cameraTurnVariance, 0, MAX_CAMERAS);
    System.arraycopy(uncertainties, cameras + MAX_CAMERAS, cameraShiftVariance, 0, MAX_CAMERAS);
  }

  /** A camera's turn's column; its shift forward and left, and its headings' shift, follow it. */
  private static int turnColumn(int camera) {
    return 2 + PER_CAMERA * camera;
  }

  /** A tag's placement's column along x; along y follows it. */
  private static int tagColumn(int tag) {
    return 2 + PER_CAMERA * MAX_CAMERAS + PER_TAG * tag;
  }

  /** The position's own block, and nothing relating it to the heading or to considered errors. */
  private void setPosition(double xx, double xy, double yy) {
    covariance[0] = xx;
    covariance[1] = xy;
    covariance[3] = xy;
    covariance[4] = yy;
    clearCrossTerms();
    unrelate(0);
    unrelate(1);
  }

  /** The heading unrelated to the position. */
  private void clearCrossTerms() {
    covariance[2] = 0;
    covariance[6] = 0;
    covariance[5] = 0;
    covariance[7] = 0;
  }

  /** One of x, y, or the offset, unrelated to every considered error: it was just set afresh. */
  private void unrelate(int row) {
    Arrays.fill(related, row * considered, (row + 1) * considered, 0);
  }

  private void symmetrize() {
    covariance[1] = covariance[3] = (covariance[1] + covariance[3]) / 2;
    covariance[2] = covariance[6] = (covariance[2] + covariance[6]) / 2;
    covariance[5] = covariance[7] = (covariance[5] + covariance[7]) / 2;
  }

  private void symmetrizeS() {
    for (int i = 0; i < rows; i++) {
      for (int j = i + 1; j < rows; j++) {
        s3[i * 3 + j] = s3[j * 3 + i] = (s3[i * 3 + j] + s3[j * 3 + i]) / 2;
      }
    }
  }

  private static double square(double value) {
    return value * value;
  }
}
