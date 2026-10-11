package com.michaelgrundvig.frc.runtime.subsystems.localization;

import com.michaelgrundvig.frc.localization.CameraTrust;
import com.michaelgrundvig.frc.localization.DriveConditions;
import com.michaelgrundvig.frc.localization.DrivetrainReadings;
import com.michaelgrundvig.frc.localization.LocalizationEstimate;
import com.michaelgrundvig.frc.localization.LocalizationGate;
import com.michaelgrundvig.frc.localization.LocalizationOutput;
import com.michaelgrundvig.frc.localization.LocalizationService;
import com.michaelgrundvig.frc.localization.LoopContext;
import com.michaelgrundvig.frc.localization.PoseCovariance;
import com.michaelgrundvig.frc.localization.PoseReset;
import com.michaelgrundvig.frc.localization.WheelTrust;
import com.michaelgrundvig.frc.localization.YawSource;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.system.RobotController;
import org.wpilib.util.Alert;

/**
 * Where the robot is, for the robot program: the {@link LocalizationService} (the one owner of the
 * estimate, in {@code lib/localization}), fed on the robot loop, with what the robot does with what
 * it says: the log, the alerts, and the lines for the pit and the drive team. Everything that asks
 * where the robot is asks here, and nothing else changes the estimate.
 *
 * <p>Each loop, in order:
 *
 * <ol>
 *   <li>{@link #periodic}, with what the drivetrain read: the service takes it, with the loop's
 *       time from the robot's clock;
 *   <li>the cameras, through {@link #service()}: each image, at its capture time;
 *   <li>{@link #update}: the service applies them, and this logs what it made of them, and says
 *       whether the estimate is good enough to score from.
 * </ol>
 *
 * <p>The log's keys ({@code Localization/...}) are what the AdvantageScope layout reads, and what a
 * replay compares: they stay as they are.
 */
public final class Localization implements AutoCloseable {

  /** A reset to "exactly here": a millimeter and a milliradian. */
  private static final PoseCovariance EXACTLY = PoseCovariance.fromStdDevs(0.001, 0.001, 0.001);

  /**
   * The logged uncertainty ellipse holds the true position this often, if the estimate is honest:
   * where the consistency study's "outside the ellipse" begins (3.72 standard deviations).
   */
  private static final double ELLIPSE_PROBABILITY = 0.999;

  /**
   * Points around the logged ellipse; the first is repeated at the end, to close it. Sixteen draw a
   * smooth enough outline for 408 bytes a loop, on the robot too, where a match's log carries it.
   */
  private static final int ELLIPSE_POINTS = 16;

  /** Where the ellipse's points sit around it, worked out once. */
  private static final double[] ELLIPSE_COS = new double[ELLIPSE_POINTS];

  private static final double[] ELLIPSE_SIN = new double[ELLIPSE_POINTS];

  static {
    for (int i = 0; i < ELLIPSE_POINTS; i++) {
      ELLIPSE_COS[i] = Math.cos(2 * Math.PI * i / ELLIPSE_POINTS);
      ELLIPSE_SIN[i] = Math.sin(2 * Math.PI * i / ELLIPSE_POINTS);
    }
  }

  /** An ellipse wider than the field says nothing a view could draw. */
  private static final double MAX_ELLIPSE_METERS = 20;

  private static final Pose2d[] NO_ELLIPSE = {};

  /** The logged ellipse's points, reused loop after loop. */
  private final Pose2d[] ellipse = new Pose2d[ELLIPSE_POINTS + 1];

  private final LocalizationService service;

  /** Why the estimate isn't good enough to score from; null when it is. */
  private @Nullable String notReadyToScore = "not started";

  /** The uncertainty as {@link #describe()} last showed it; NaN before it did. */
  private double shownCentimeters = Double.NaN;

  private double shownDegrees = Double.NaN;

  private final Alert gyroDisconnected =
      new Alert(
          "Localization",
          "gyroDisconnected",
          "Gyro disconnected: heading estimated from wheels, and will drift",
          Alert.Level.HIGH);
  private final Alert usingBackup =
      new Alert(
          "Localization",
          "usingBackupImu",
          "Gyro disconnected: heading from the backup IMU",
          Alert.Level.MEDIUM);
  private final Alert imusDisagree =
      new Alert(
          "Localization",
          "imusDisagree",
          "The gyro and the backup IMU disagree",
          Alert.Level.MEDIUM);
  private final Alert camerasWithoutPlaces =
      new Alert(
          "Localization",
          "camerasWithoutPlaces",
          "More cameras than the estimator keeps apart",
          Alert.Level.MEDIUM);
  private final Alert tagsWithoutPlaces =
      new Alert(
          "Localization",
          "tagsWithoutPlaces",
          "Measurements from tags the field's layout doesn't have",
          Alert.Level.MEDIUM);
  private final Alert imuOrientation =
      new Alert(
          "Localization",
          "imuOrientation",
          "The accelerometer reads gravity standing still: check the IMU's orientation in the"
              + " robot's definition",
          Alert.Level.MEDIUM);

  /**
   * @param service localization itself, which this feeds and reports on from now on
   */
  public Localization(LocalizationService service) {
    this.service = service;
  }

  /**
   * Localization itself: where the cameras send their images, and what a test or a view asks beyond
   * what this says.
   */
  public LocalizationService service() {
    return service;
  }

  /**
   * One loop of the drivetrain's readings: after it reads its sensors, and before the cameras.
   *
   * @param enabled whether the robot is enabled; while disabled, people may carry it where its
   *     wheels can't see
   */
  public void periodic(DrivetrainReadings readings, boolean enabled) {
    service.drivetrain(readings, new LoopContext(RobotController.getTime(), enabled));
    WheelTrust wheels = service.wheels();
    DriveConditions conditions = wheels.conditions();
    checkOrientation(wheels);
    checkImus();
    gyroDisconnected.set(!readings.gyro().usable() && !readings.backupImu().usable());
    usingBackup.set(!readings.gyro().usable() && readings.backupImu().usable());
    Logger.recordOutput("Localization/TiltDegrees", Math.toDegrees(conditions.tiltRadians()));
    Logger.recordOutput(
        "Localization/GroundSlopeDegrees", Math.toDegrees(conditions.groundSlopeRadians()));
    Logger.recordOutput("Localization/CollisionShare", conditions.collisionShare());
    Logger.recordOutput(
        "Localization/UnexplainedJoltMetersPerSecSq", wheels.unexplainedMetersPerSecSq());
    Logger.recordOutput("Localization/SlipMetersPerSecond", conditions.slipMetersPerSecond());
    Logger.recordOutput("Localization/TurnSlipMetersPerSecond", wheels.turnSlipMetersPerSecond());
    Logger.recordOutput("Localization/SpeedSlipMetersPerSecond", wheels.speedSlipMetersPerSecond());
    Logger.recordOutput("Localization/TurnApartDegrees", Math.toDegrees(wheels.turnApartRadians()));
    Logger.recordOutput("Localization/SlideMetersPerSecond", wheels.slideMetersPerSecond());
    Logger.recordOutput(
        "Localization/AccelerometerBiasMetersPerSecSq", wheels.accelerometerBiasMetersPerSecSq());
  }

  /**
   * Applies the loop's measurements, logs what became of them, and says whether the estimate is
   * good enough to score from. After the cameras.
   *
   * @return what the service made of the loop: the estimate, how far each source is trusted, and
   *     what became of each camera measurement, for the cameras' records and the World view
   */
  public LocalizationOutput update() {
    LocalizationOutput output = service.update();
    notReadyToScore = LocalizationGate.SCORING.whyNot(output.estimate()).orElse(null);
    List<String> beyond = new ArrayList<>();
    for (Map.Entry<String, CameraTrust> camera : output.trust().cameras().entrySet()) {
      if (!camera.getValue().keptApart()) {
        beyond.add(camera.getKey());
      }
    }
    if (!beyond.isEmpty()) {
      camerasWithoutPlaces.setText(
          "More cameras than the estimator keeps apart: "
              + String.join(", ", beyond)
              + " count each image's mount and tag errors as new, so their images together claim"
              + " more than they know");
    }
    camerasWithoutPlaces.set(!beyond.isEmpty());
    Set<Integer> offTheLayout = output.trust().tagsWithoutPlaces();
    if (!offTheLayout.isEmpty()) {
      tagsWithoutPlaces.setText(
          "Measurements from tags the field's layout doesn't have: "
              + offTheLayout
              + " count each image's placement error as new, so their images together claim more"
              + " than they know");
    }
    tagsWithoutPlaces.set(!offTheLayout.isEmpty());
    log(output.estimate());
    return output;
  }

  /** Where the robot is: cheap, for driving every loop. */
  public Pose2d pose() {
    return service.pose();
  }

  /** The estimate, with its uncertainty, mode, and health, as of the last step. */
  public LocalizationEstimate estimate() {
    return service.estimate();
  }

  /**
   * The drive team put the robot here: on its start before the match, say. Call it after {@link
   * #periodic}, so a turn made in placing it isn't counted twice.
   *
   * @param how sure the placement is
   * @param reason for the log, such as "placed on Center"
   */
  public void place(Pose2d pose, PoseCovariance how, String reason) {
    service.place(new PoseReset(RobotController.getTime(), pose, how, reason));
  }

  /** The robot is exactly here, now. */
  public void resetPose(Pose2d pose) {
    place(pose, EXACTLY, "reset");
  }

  /**
   * The robot faces this heading, as sure as the given spread; its position stays. If that's wrong,
   * the cameras' headings will agree against it and the estimate turns back to them.
   */
  public void resetHeading(Rotation2d heading, double stdDevRadians) {
    service.resetHeading(heading, stdDevRadians);
  }

  /**
   * Whether the estimate is good enough to score from, by {@link LocalizationGate#SCORING}: bind
   * scoring and aiming commands to it.
   */
  public boolean readyToScore() {
    return notReadyToScore == null;
  }

  /** One line for the pit and the drive team: ready to score, or why not. */
  public String scoring() {
    return notReadyToScore == null ? "Ready to score" : "Not ready: " + notReadyToScore;
  }

  /**
   * One line for the pit: what the pose rests on, and how sure it is, to the centimeter and the
   * tenth of a degree. Each number shown moves only once the real one has moved that much from it,
   * so the line doesn't flicker as the uncertainty wanders within a step.
   */
  public String describe() {
    LocalizationEstimate estimate = service.estimate();
    PoseCovariance uncertainty = estimate.covariance();
    shownCentimeters = held(shownCentimeters, 100 * uncertainty.largestPositionStdDev(), 1);
    shownDegrees = held(shownDegrees, Math.toDegrees(uncertainty.headingStdDev()), 0.1);
    String basis =
        switch (estimate.mode()) {
          case INITIALIZING -> "Not placed yet";
          case TRACKING -> "Tracking tags";
          case DEAD_RECKONING -> "Dead reckoning";
          case LOST -> "Lost: recovering";
          case IMU_FAULT -> "Gyro lost";
        };
    return String.format(
        Locale.ROOT, "%s: within %.0f cm, %.1f°", basis, shownCentimeters, shownDegrees);
  }

  /** A number as shown, to a step: it moves only once the value has moved a step from it. */
  private static double held(double shown, double value, double step) {
    return Math.abs(value - shown) < step ? shown : Math.round(value / step) * step;
  }

  /** The robot's pitch, from the IMU: zero without one. Positive is nose down. */
  public double pitchRadians() {
    return service.pitchRadians();
  }

  /** The robot's roll, from the IMU: zero without one. */
  public double rollRadians() {
    return service.rollRadians();
  }

  /** What turns odometry now: the gyro, the backup IMU, or the wheels. */
  public YawSource yawSource() {
    return service.yawSource();
  }

  /** What the IMU said about trusting the wheels this loop. */
  public DriveConditions conditions() {
    return service.conditions();
  }

  /**
   * Says when the accelerometer of the IMU that turns odometry reads, standing still, past any bias
   * a part has: gravity's share, leaking in because the robot's definition gives the IMU another
   * orientation than it's mounted with. Its pitch and roll can be right all the while (a Pigeon's
   * follow the mount pose saved on it), so only the accelerometer shows it.
   */
  private void checkOrientation(WheelTrust wheels) {
    boolean leaks = wheels.accelerometerLeaksGravity();
    if (leaks) {
      imuOrientation.setText(
          String.format(
              Locale.ROOT,
              "The %s's accelerometer reads %.1f m/s² standing still, gravity's share: check its"
                  + " orientation in the robot's definition (docs/calibration.md)",
              wheels.imu() == YawSource.BACKUP ? "backup IMU" : "gyro",
              wheels.accelerometerBiasMetersPerSecSq()));
    }
    imuOrientation.set(leaks);
  }

  /**
   * Says when the gyro and the backup IMU disagree for long: one is failing, loose, or needs its
   * scale calibrated. It raises the question rather than silently picking one.
   */
  private void checkImus() {
    double disagreement = Math.toDegrees(service.imuDisagreementRadians());
    if (service.imusDisagree()) {
      imusDisagree.setText(
          String.format(
              Locale.ROOT,
              "The gyro and the backup IMU disagree by %.0f° over the last second: one is failing,"
                  + " loose, or needs its scale calibrated",
              Math.abs(disagreement)));
    }
    imusDisagree.set(service.imusDisagree());
    Logger.recordOutput("Localization/ImuDisagreementDegrees", disagreement);
    Logger.recordOutput("Localization/YawSource", service.yawSource().name());
  }

  /**
   * Logs the estimate and what it rests on. For AdvantageScope (layouts/advantagescope.json): the
   * estimate in 3D, raised to the ground under it and tilted as the IMU says, and its 99.9% ellipse
   * as a closed outline on the field.
   */
  private void log(LocalizationEstimate estimate) {
    PoseCovariance uncertainty = estimate.covariance();
    LocalizationEstimate.Health health = estimate.health();
    Pose2d pose = estimate.pose();
    Logger.recordOutput("Localization/Pose", pose);
    Logger.recordOutput("Localization/Pose3d", service.pose3d(pose));
    Logger.recordOutput("Localization/Ellipse", ellipse(pose, uncertainty));
    Logger.recordOutput("Localization/Mode", estimate.mode().name());
    Logger.recordOutput("Localization/PositionStdDevMeters", uncertainty.largestPositionStdDev());
    Logger.recordOutput(
        "Localization/HeadingStdDevDegrees", Math.toDegrees(uncertainty.headingStdDev()));
    Logger.recordOutput("Localization/ResidualMeters", health.residualMeters());
    Logger.recordOutput("Localization/SuspectTravelMeters", health.suspectTravelMeters());
    Logger.recordOutput("Localization/RecentDisagreementMeters", health.recentDisagreementMeters());
    Logger.recordOutput("Localization/Recoveries", health.recoveries());
    Logger.recordOutput("Localization/ReadyToScore", notReadyToScore == null);
    Logger.recordOutput(
        "Localization/NotReadyBecause", notReadyToScore == null ? "" : notReadyToScore);
  }

  /**
   * The estimate's ellipse as a closed outline around its position, in the array kept for it; none
   * while the uncertainty is too large to draw (before the robot is placed).
   */
  Pose2d[] ellipse(Pose2d center, PoseCovariance uncertainty) {
    PoseCovariance.Ellipse shape = uncertainty.ellipse(ELLIPSE_PROBABILITY);
    if (!(shape.major() < MAX_ELLIPSE_METERS)) {
      return NO_ELLIPSE;
    }
    for (int i = 0; i < ELLIPSE_POINTS; i++) {
      Translation2d point = shape.pointAt(ELLIPSE_COS[i], ELLIPSE_SIN[i]);
      ellipse[i] =
          new Pose2d(center.getX() + point.getX(), center.getY() + point.getY(), Rotation2d.ZERO);
    }
    ellipse[ELLIPSE_POINTS] = ellipse[0];
    return ellipse;
  }

  boolean isGyroDisconnectedAlertActive() {
    return gyroDisconnected.get();
  }

  boolean isUsingBackupAlertActive() {
    return usingBackup.get();
  }

  boolean isCamerasWithoutPlacesAlertActive() {
    return camerasWithoutPlaces.get();
  }

  boolean isImuDisagreementAlertActive() {
    return imusDisagree.get();
  }

  boolean isImuOrientationAlertActive() {
    return imuOrientation.get();
  }

  @Override
  public void close() {
    gyroDisconnected.close();
    usingBackup.close();
    imusDisagree.close();
    camerasWithoutPlaces.close();
    tagsWithoutPlaces.close();
    imuOrientation.close();
  }
}
