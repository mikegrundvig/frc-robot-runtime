package com.michaelgrundvig.frc.runtime.subsystems.localization;

import com.michaelgrundvig.frc.localization.CameraTrust;
import com.michaelgrundvig.frc.localization.DefaultLocalizer;
import com.michaelgrundvig.frc.localization.DriveConditions;
import com.michaelgrundvig.frc.localization.DrivetrainReadings;
import com.michaelgrundvig.frc.localization.LocalizationEstimate;
import com.michaelgrundvig.frc.localization.LocalizationGate;
import com.michaelgrundvig.frc.localization.LocalizationOutput;
import com.michaelgrundvig.frc.localization.LocalizationService;
import com.michaelgrundvig.frc.localization.Localizer;
import com.michaelgrundvig.frc.localization.LocalizerReport;
import com.michaelgrundvig.frc.localization.LoopContext;
import com.michaelgrundvig.frc.localization.PoseCovariance;
import com.michaelgrundvig.frc.localization.PoseReset;
import com.michaelgrundvig.frc.localization.Trust;
import com.michaelgrundvig.frc.localization.WheelTrust;
import com.michaelgrundvig.frc.localization.YawSource;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.system.RobotController;
import org.wpilib.util.Alert;

/**
 * Where the robot is, for the robot program. The {@link LocalizationService}, in the localization
 * library, is the one owner of the estimate: the robot's localizer, with the solving of camera
 * images in front of it. This feeds it on the robot loop, and does what the robot does with what it
 * says: the log, the alerts, and the lines for the pit and the drive team. Everything that asks
 * where the robot is asks here, and nothing else changes the estimate.
 *
 * <p>A robot chooses its localizer ({@code RobotConfig}'s {@code localizer}), and a localizer
 * reports only the options it has ({@link Localizer}). Where one is not reported, nothing here
 * shows a number in its place: a line for the pit says "not reported by this localizer", an alert
 * that needs it stays off, and its keys are left out of the log, which instead names what is
 * missing ({@code Localization/NotReported}).
 *
 * <p>Each loop goes in this order:
 *
 * <ol>
 *   <li>{@link #periodic}, with what the drivetrain read. The service takes the readings with the
 *       loop's time from the robot's clock, and moves the estimate along by the wheels and the
 *       gyro.
 *   <li>The cameras, through {@link #service()}. Each image is solved as the robot was when it was
 *       taken, which is why odometry goes first.
 *   <li>{@link #update}. The service applies the loop's images. This logs what it made of them, and
 *       says whether the estimate is good enough to score from.
 * </ol>
 *
 * <p>The log's keys ({@code Localization/...}) are what the AdvantageScope layout reads, and what a
 * replay compares, so they stay as they are.
 */
public final class Localization implements AutoCloseable {

  /** How sure a reset to "exactly here" is: within a millimeter and a milliradian. */
  private static final PoseCovariance EXACTLY = PoseCovariance.fromStdDevs(0.001, 0.001, 0.001);

  /**
   * The logged uncertainty ellipse holds the true position this often, if the estimate is honest.
   * That puts its edge at 3.72 standard deviations, which is where the consistency study's "outside
   * the ellipse" begins.
   */
  private static final double ELLIPSE_PROBABILITY = 0.999;

  /**
   * How many points are logged around the ellipse. The first is repeated at the end, to close the
   * outline. Sixteen draw a smooth enough outline for 408 bytes a loop, which matters on the robot
   * too, where a match's log carries it.
   */
  private static final int ELLIPSE_POINTS = 16;

  /**
   * The cosine and the sine of the angle each point sits at around the ellipse, worked out once.
   * Point {@code i} is at {@code 2π × i / ELLIPSE_POINTS}.
   */
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

  /** What is logged in the ellipse's place while it is too large to draw. */
  private static final Pose2d[] NO_ELLIPSE = {};

  /**
   * The uncertainty in {@link #describe()} is shown to the centimeter and the tenth of a degree.
   */
  private static final double SHOWN_STEP_CENTIMETERS = 1;

  private static final double SHOWN_STEP_DEGREES = 0.1;

  /** The logged ellipse's points, in an array that is filled again loop after loop. */
  private final Pose2d[] ellipse = new Pose2d[ELLIPSE_POINTS + 1];

  private final LocalizationService service;

  /** What the pit reads in place of something the localizer does not report. */
  static final String NOT_REPORTED = "not reported by this localizer";

  /** What the scoring gate made of the estimate. */
  private enum Scoring {
    READY,
    NOT_READY,
    /** The localizer does not report what the gate judges by, so the gate holds nothing back. */
    NOT_JUDGED
  }

  private Scoring scoring = Scoring.NOT_READY;

  /** Why the estimate isn't good enough to score from, while it isn't. */
  private String notReadyBecause = "not started";

  /** The options the localizer did not report in the last update, by name, for the log. */
  private final List<String> notReported = new ArrayList<>();

  /** The uncertainty as {@link #describe()} last showed it. Each is empty before it has. */
  private OptionalDouble shownCentimeters = OptionalDouble.empty();

  private OptionalDouble shownDegrees = OptionalDouble.empty();

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
   * One loop of the drivetrain's readings. Call it after the drivetrain reads its sensors, and
   * before the cameras.
   *
   * @param enabled whether the robot is enabled. While it is disabled, people may carry it where
   *     its wheels can't see.
   */
  public void periodic(DrivetrainReadings readings, boolean enabled) {
    service.drivetrain(readings, new LoopContext(RobotController.getTime(), enabled));
    Optional<WheelTrust> wheels = service.localizer().wheels();
    if (wheels.isPresent()) {
      checkOrientation(wheels.get());
    } else {
      // Nothing compared the accelerometer with the wheels, so nothing can say it is mounted wrong.
      imuOrientation.set(false);
    }
    checkImus();
    showWhatTurnsOdometry(readings);
    wheels.ifPresent(Localization::logWheelTrust);
  }

  /**
   * Says when the gyro is lost, and what the heading comes from instead: the backup IMU if it
   * reports, and otherwise the wheels, whose heading drifts.
   */
  private void showWhatTurnsOdometry(DrivetrainReadings readings) {
    boolean gyroLost = !readings.gyro().usable();
    boolean backupReports = readings.backupImu().usable();
    gyroDisconnected.set(gyroLost && !backupReports);
    usingBackup.set(gyroLost && backupReports);
  }

  /** Logs how far the wheels are trusted this loop, and what the IMU said that decided it. */
  private static void logWheelTrust(WheelTrust wheels) {
    DriveConditions conditions = wheels.conditions();
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
   * good enough to score from. Call it after the cameras.
   *
   * @return what the service made of the loop: the pose with what the localizer reports of it, how
   *     far each source is trusted, and what became of each camera measurement, for the cameras'
   *     records and the World view
   */
  public LocalizationOutput update() {
    LocalizationOutput output = service.update();
    judgeScoring(output.report());
    Optional<Trust> trust = output.trust();
    if (trust.isPresent()) {
      warnOfCamerasNotKeptApart(trust.get().cameras());
      warnOfTagsOffTheLayout(trust.get().tagsWithoutPlaces());
    } else {
      // These two warn of the default localizer's own limits. Another localizer has none to say.
      camerasWithoutPlaces.set(false);
      tagsWithoutPlaces.set(false);
    }
    log(output.report());
    return output;
  }

  /**
   * Holds the estimate to the scoring gate. The gate judges by what the pose rests on, the numbers
   * behind that, and how sure the pose is. A localizer that does not report all three cannot be
   * judged, and the gate then holds nothing back: a team that chose such a localizer scores as
   * teams without a gate do, and the pit's line says the pose was not judged.
   */
  private void judgeScoring(LocalizerReport report) {
    boolean canJudge =
        report.mode().isPresent() && report.health().isPresent() && report.covariance().isPresent();
    if (!canJudge) {
      scoring = Scoring.NOT_JUDGED;
      return;
    }
    Optional<String> whyNot =
        LocalizationGate.SCORING.whyNot(
            report.mode().get(), report.health().get(), report.covariance().get());
    if (whyNot.isEmpty()) {
      scoring = Scoring.READY;
      return;
    }
    scoring = Scoring.NOT_READY;
    notReadyBecause = whyNot.get();
  }

  /**
   * Says which cameras the estimator has no room to keep apart. It keeps each camera's repeated
   * errors (its mount's, its tags') apart from the others', for a limited number of cameras. A
   * camera beyond that number has each image's errors taken as new, so its images together claim
   * more than they know.
   */
  private void warnOfCamerasNotKeptApart(Map<String, CameraTrust> cameras) {
    List<String> notKeptApart = new ArrayList<>();
    for (Map.Entry<String, CameraTrust> camera : cameras.entrySet()) {
      if (!camera.getValue().keptApart()) {
        notKeptApart.add(camera.getKey());
      }
    }
    boolean any = !notKeptApart.isEmpty();
    if (any) {
      camerasWithoutPlaces.setText(
          "More cameras than the estimator keeps apart: "
              + String.join(", ", notKeptApart)
              + " count each image's mount and tag errors as new, so their images together claim"
              + " more than they know");
    }
    camerasWithoutPlaces.set(any);
  }

  /**
   * Says which tags measurements named that the field's layout doesn't have. The estimator can't
   * keep such a tag's placement error apart, so it takes each image's as new.
   */
  private void warnOfTagsOffTheLayout(Set<Integer> offTheLayout) {
    boolean any = !offTheLayout.isEmpty();
    if (any) {
      tagsWithoutPlaces.setText(
          "Measurements from tags the field's layout doesn't have: "
              + offTheLayout
              + " count each image's placement error as new, so their images together claim more"
              + " than they know");
    }
    tagsWithoutPlaces.set(any);
  }

  /** Where the robot is. It is cheap to ask, for driving every loop. */
  public Pose2d pose() {
    return service.pose();
  }

  /** The pose, with what the localizer reports about it, as of the last step. */
  public LocalizerReport report() {
    return service.report();
  }

  /** Whether the robot runs the default localizer, and not one it chose in its configuration. */
  public boolean runsTheDefaultLocalizer() {
    return service.localizer() instanceof DefaultLocalizer;
  }

  /**
   * The default localizer's estimate, whole: for a test or a study of that localizer.
   *
   * @throws IllegalStateException if the robot chose another localizer
   */
  public LocalizationEstimate estimate() {
    if (service.localizer() instanceof DefaultLocalizer ours) {
      return ours.estimate();
    }
    throw new IllegalStateException(
        "only the default localizer has a whole estimate; ask report() of any localizer");
  }

  /**
   * The drive team put the robot here, on its start before the match, say. Call it after {@link
   * #periodic}, so that a turn made in placing it isn't counted twice.
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
   * The robot faces this heading, as sure as the given spread. Its position stays. If the heading
   * is wrong, the cameras' headings will agree against it and the estimate turns back to them.
   */
  public void resetHeading(Rotation2d heading, double stdDevRadians) {
    service.resetHeading(heading, stdDevRadians);
  }

  /**
   * Whether the estimate is good enough to score from, by {@link LocalizationGate#SCORING}. Bind
   * scoring and aiming commands to it. With a localizer that reports too little to judge by, the
   * gate holds nothing back, and this is true ({@link #scoring} says it was not judged).
   */
  public boolean readyToScore() {
    return scoring != Scoring.NOT_READY;
  }

  /** One line for the pit and the drive team: ready to score, or why not. */
  public String scoring() {
    return switch (scoring) {
      case READY -> "Ready to score";
      case NOT_READY -> "Not ready: " + notReadyBecause;
      case NOT_JUDGED -> "Not judged: what the pose rests on is " + NOT_REPORTED;
    };
  }

  /**
   * One line for the pit: what the pose rests on, and how sure it is, to the centimeter and the
   * tenth of a degree. Each number shown moves only once the real one has moved a step from it, so
   * the line doesn't flicker as the uncertainty wanders within a step.
   */
  public String describe() {
    Optional<LocalizationEstimate.Mode> mode = service.localizer().mode();
    Optional<PoseCovariance> uncertainty = service.localizer().covariance();
    if (mode.isEmpty() && uncertainty.isEmpty()) {
      return "What the pose rests on and how sure it is: " + NOT_REPORTED;
    }
    if (uncertainty.isEmpty()) {
      return restsOn(mode.get()) + ": how sure is " + NOT_REPORTED;
    }
    double centimeters = 100 * uncertainty.get().largestPositionStdDev();
    double degrees = Math.toDegrees(uncertainty.get().headingStdDev());
    shownCentimeters =
        OptionalDouble.of(shown(shownCentimeters, centimeters, SHOWN_STEP_CENTIMETERS));
    shownDegrees = OptionalDouble.of(shown(shownDegrees, degrees, SHOWN_STEP_DEGREES));
    String restsOn = "What the pose rests on is " + NOT_REPORTED;
    if (mode.isPresent()) {
      restsOn = restsOn(mode.get());
    }
    return String.format(
        Locale.ROOT,
        "%s: within %.0f cm, %.1f°",
        restsOn,
        shownCentimeters.getAsDouble(),
        shownDegrees.getAsDouble());
  }

  /** What the pose rests on, in the pit's words. */
  private static String restsOn(LocalizationEstimate.Mode mode) {
    return switch (mode) {
      case INITIALIZING -> "Not placed yet";
      case TRACKING -> "Tracking tags";
      case DEAD_RECKONING -> "Dead reckoning";
      case LOST -> "Lost: recovering";
      case IMU_FAULT -> "Gyro lost";
    };
  }

  /**
   * The number to show for a value. It is the number shown before while the value is within a step
   * of that, and the value rounded to the step once it has moved a step away, or when nothing has
   * been shown yet.
   */
  private static double shown(OptionalDouble shownBefore, double value, double step) {
    boolean withinAStep =
        shownBefore.isPresent() && Math.abs(value - shownBefore.getAsDouble()) < step;
    if (withinAStep) {
      return shownBefore.getAsDouble();
    }
    return Math.round(value / step) * step;
  }

  /** The robot's pitch, from the IMU, or zero without one. Positive is nose down. */
  public double pitchRadians() {
    return service.pitchRadians();
  }

  /** The robot's roll, from the IMU, or zero without one. */
  public double rollRadians() {
    return service.rollRadians();
  }

  /**
   * What the default localizer's IMU said about trusting the wheels this loop: for a test of that
   * localizer.
   *
   * @throws IllegalStateException if the robot chose another localizer
   */
  public DriveConditions conditions() {
    if (service.localizer() instanceof DefaultLocalizer ours) {
      return ours.conditions();
    }
    throw new IllegalStateException(
        "only the default localizer has drive conditions; ask the localizer's wheels() of any");
  }

  /**
   * What turns odometry now: the gyro, the backup IMU, or the wheels. Empty when the localizer does
   * not report it.
   */
  public Optional<YawSource> yawSource() {
    return service.localizer().yawSource();
  }

  /**
   * Says when the accelerometer of the IMU that turns odometry reads more, standing still, than any
   * bias a part has. What it reads is a share of gravity, which leaks in when the robot's
   * definition gives the IMU another orientation than it's mounted with. The IMU's pitch and roll
   * can be right all the while (a Pigeon's follow the mount pose saved on it), so only the
   * accelerometer shows it.
   */
  private void checkOrientation(WheelTrust wheels) {
    boolean leaks = wheels.accelerometerLeaksGravity();
    if (leaks) {
      String imu = "gyro";
      if (wheels.imu() == YawSource.BACKUP) {
        imu = "backup IMU";
      }
      imuOrientation.setText(
          String.format(
              Locale.ROOT,
              "The %s's accelerometer reads %.1f m/s² standing still, gravity's share: check its"
                  + " orientation in the robot's definition (docs/calibration.md)",
              imu,
              wheels.accelerometerBiasMetersPerSecSq()));
    }
    imuOrientation.set(leaks);
  }

  /**
   * Says when the gyro and the backup IMU disagree for long. One of them is failing, loose, or
   * needs its scale calibrated, and nothing here can tell which. So it raises the question rather
   * than silently picking one.
   */
  private void checkImus() {
    double disagreementDegrees = Math.toDegrees(service.imuDisagreementRadians());
    boolean disagree = service.imusDisagree();
    if (disagree) {
      imusDisagree.setText(
          String.format(
              Locale.ROOT,
              "The gyro and the backup IMU disagree by %.0f° over the last second: one is failing,"
                  + " loose, or needs its scale calibrated",
              Math.abs(disagreementDegrees)));
    }
    imusDisagree.set(disagree);
    Logger.recordOutput("Localization/ImuDisagreementDegrees", disagreementDegrees);
    Optional<YawSource> turnsOdometry = service.localizer().yawSource();
    if (turnsOdometry.isPresent()) {
      Logger.recordOutput("Localization/YawSource", turnsOdometry.get().name());
    }
  }

  /**
   * Logs the estimate and what it rests on. Two of the values are for AdvantageScope
   * (layouts/advantagescope.json): the estimate in 3D, raised to the ground under it and tilted as
   * the IMU says, and its 99.9% ellipse as a closed outline on the field.
   *
   * <p>What the localizer does not report is left out of the log, never logged as a number that
   * nothing measured. {@code Localization/NotReported} names what was left out.
   */
  private void log(LocalizerReport report) {
    Pose2d pose = report.pose();
    Logger.recordOutput("Localization/Pose", pose);
    Logger.recordOutput("Localization/Pose3d", service.pose3d(pose));
    Optional<PoseCovariance> uncertainty = report.covariance();
    if (uncertainty.isPresent()) {
      Logger.recordOutput("Localization/Ellipse", ellipse(pose, uncertainty.get()));
    }
    if (report.mode().isPresent()) {
      Logger.recordOutput("Localization/Mode", report.mode().get().name());
    }
    if (uncertainty.isPresent()) {
      Logger.recordOutput(
          "Localization/PositionStdDevMeters", uncertainty.get().largestPositionStdDev());
      Logger.recordOutput(
          "Localization/HeadingStdDevDegrees", Math.toDegrees(uncertainty.get().headingStdDev()));
    }
    if (report.health().isPresent()) {
      LocalizationEstimate.Health health = report.health().get();
      Logger.recordOutput("Localization/ResidualMeters", health.residualMeters());
      Logger.recordOutput("Localization/SuspectTravelMeters", health.suspectTravelMeters());
      Logger.recordOutput(
          "Localization/RecentDisagreementMeters", health.recentDisagreementMeters());
      Logger.recordOutput("Localization/Recoveries", health.recoveries());
    }
    if (scoring != Scoring.NOT_JUDGED) {
      boolean ready = scoring == Scoring.READY;
      Logger.recordOutput("Localization/ReadyToScore", ready);
      Logger.recordOutput("Localization/NotReadyBecause", ready ? "" : notReadyBecause);
    }
    logWhatIsNotReported(report);
  }

  /**
   * Names the options the localizer does not report, so a log that lacks their keys says why. The
   * default localizer reports them all, and then nothing is logged here.
   */
  private void logWhatIsNotReported(LocalizerReport report) {
    Localizer localizer = service.localizer();
    notReported.clear();
    noteIfMissing("covariance", report.covariance());
    noteIfMissing("mode", report.mode());
    noteIfMissing("health", report.health());
    noteIfMissing("decisions", localizer.decisions());
    noteIfMissing("wheels", localizer.wheels());
    noteIfMissing("trust", localizer.trust());
    noteIfMissing("yawSource", localizer.yawSource());
    if (notReported.isEmpty()) {
      return;
    }
    Logger.recordOutput("Localization/NotReported", String.join(", ", notReported));
  }

  private void noteIfMissing(String option, Optional<?> reported) {
    if (reported.isEmpty()) {
      notReported.add(option);
    }
  }

  /**
   * The estimate's ellipse as a closed outline around its position, in the array kept for it. While
   * the uncertainty is too large to draw, which it is before the robot is placed, there is no
   * outline, and the array returned is empty.
   */
  Pose2d[] ellipse(Pose2d center, PoseCovariance uncertainty) {
    PoseCovariance.Ellipse shape = uncertainty.ellipse(ELLIPSE_PROBABILITY);
    // An ellipse whose size isn't a number can't be drawn either, and no comparison holds for it.
    boolean smallEnoughToDraw = shape.major() < MAX_ELLIPSE_METERS;
    if (!smallEnoughToDraw) {
      return NO_ELLIPSE;
    }
    for (int i = 0; i < ELLIPSE_POINTS; i++) {
      Translation2d point = shape.pointAt(ELLIPSE_COS[i], ELLIPSE_SIN[i]);
      ellipse[i] =
          new Pose2d(center.getX() + point.getX(), center.getY() + point.getY(), Rotation2d.ZERO);
    }
    // The first point again closes the outline.
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
