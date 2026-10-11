package com.michaelgrundvig.frc.runtime.subsystems.localization;

import com.michaelgrundvig.frc.localization.DrivetrainReadings;
import com.michaelgrundvig.frc.localization.EstimatorConfig;
import com.michaelgrundvig.frc.localization.FrameVerdict;
import com.michaelgrundvig.frc.localization.ImuReadings;
import com.michaelgrundvig.frc.localization.LocalizationEstimate;
import com.michaelgrundvig.frc.localization.LocalizationGate;
import com.michaelgrundvig.frc.localization.OdometrySample;
import com.michaelgrundvig.frc.localization.YawSource;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.subsystems.drive.Drive;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.List;
import java.util.Locale;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;

/**
 * What must hold of where the robot thinks it is every loop, whatever its drive team presses and
 * whatever fails: checked after each loop by {@code Monkey}, each a small named check that says
 * what it found wrong, or nothing.
 *
 * <ul>
 *   <li><b>IMU failover</b>: odometry turns on the gyro while it reads, on the backup IMU while
 *       only it does, and on the wheels while neither does, and the robot says which: an alert for
 *       the backup, and for the wheels an alert and the gyro-lost mode;
 *   <li><b>blind says so</b>: with every camera unplugged, the estimate isn't good enough to score
 *       from once its last image is older than the scoring gate allows, nor tracking tags once it's
 *       older than the estimator's vision timeout;
 *   <li><b>never jumps</b>: while the robot is enabled, the estimate moves in a loop no farther
 *       than its wheels' travel since the loop before; what the camera images it accepted this loop
 *       can move it, each its gate's distance (the square root of its χ²) times how unsure the
 *       estimate was of its position as it was weighed, since a Kalman update moves the estimate,
 *       measured against its own uncertainty, no farther than the measurement is from it; and what
 *       turning its heading beyond its IMU turns the path it replays from the earliest image's
 *       capture by. A recovery, a jump the estimator declares, counts and logs, is one exception; a
 *       loop with an image taken before one it applied already is the other. The estimator gates
 *       that image against the estimate before the later one, and applies the later one again after
 *       it, gated again, so the estimate ends where the two in order would have put it; but what
 *       that moves it from last loop's estimate, which already held the later image, is the earlier
 *       image weighed against that estimate, whose χ² it never works out.
 * </ul>
 *
 * <p>Each judges what the robot reports, not where the simulator really has it: off the field, or
 * fooled by a fault, the estimate may be wrong, and what's checked is that it moves and says as
 * promised.
 */
public final class LocalizationInvariants {
  /** Room for the arithmetic: a centimeter. */
  static final double JUMP_SLACK_METERS = 0.01;

  private final Localization localization;
  private final Drive drive;
  private final Supplier<List<FrameVerdict>> frames;
  private final BooleanSupplier blind;
  private final boolean cameras;
  private final double visionTimeoutSeconds;

  /** How many loops in a row every camera has been unplugged. */
  private int blindLoops;

  /** Each wheel's distance at the newest reading of the loop before; null before any. */
  private double @Nullable [] wheels;

  private @Nullable Pose2d before;
  private int recoveries;

  /** When the newest image it accepted was taken: none yet. */
  private long newestApplied = Long.MIN_VALUE;

  /** The estimate, each of the last loops, by its newest odometry's time. */
  private record Stamped(long nanos, Pose2d pose, double unsure) {}

  /** How many loops of the estimate are kept: 3 s, past a late camera's second. */
  private static final int KEPT_LOOPS = 150;

  private final Deque<Stamped> poses = new ArrayDeque<>();

  /** What turned odometry the loop before, and its yaw then. */
  private YawSource lastSource = YawSource.WHEELS;

  private double lastYaw = Double.NaN;

  /**
   * @param localization the robot's localization
   * @param drive the drivetrain, whose readings localization took this loop
   * @param frames what became of each camera image this loop
   * @param blind whether every camera is unplugged, as the simulated world has it
   * @param cameras whether the robot has cameras
   * @param estimator the estimator's configuration: how long until it's no longer tracking
   */
  public LocalizationInvariants(
      Localization localization,
      Drive drive,
      Supplier<List<FrameVerdict>> frames,
      BooleanSupplier blind,
      boolean cameras,
      EstimatorConfig estimator) {
    this.localization = localization;
    this.drive = drive;
    this.frames = frames;
    this.blind = blind;
    this.cameras = cameras;
    visionTimeoutSeconds = estimator.visionTimeoutSeconds();
  }

  /**
   * Checks the loop just run: the first thing found wrong, led by its check's name ("never jumps:
   * ..."), or null when everything holds.
   *
   * @param enabled whether the robot was enabled this loop
   */
  public @Nullable String broken(boolean enabled) {
    LocalizationEstimate estimate = localization.estimate();
    DrivetrainReadings readings = drive.readings();
    String found = failover(readings, estimate);
    String saying = blindSaysSo(estimate);
    String jump = jumps(readings, estimate, enabled);
    if (found != null) {
      return "IMU failover: " + found;
    }
    if (saying != null) {
      return "blind says so: " + saying;
    }
    return jump == null ? null : "never jumps: " + jump;
  }

  /** Whether an IMU turned odometry this loop: it reads, and sent yaws. */
  private static boolean turns(ImuReadings imu) {
    return imu.usable() && imu.yaws().count() > 0;
  }

  private @Nullable String failover(DrivetrainReadings readings, LocalizationEstimate estimate) {
    if (readings.wheels().isEmpty()) {
      return null; // nothing to turn: it keeps what it last turned on
    }
    boolean gyro = turns(readings.gyro());
    boolean backup = turns(readings.backupImu());
    YawSource expected = gyro ? YawSource.GYRO : backup ? YawSource.BACKUP : YawSource.WHEELS;
    YawSource source = localization.yawSource();
    String state =
        String.format(
            Locale.ROOT,
            "the gyro %s and the backup IMU %s",
            gyro ? "reads" : "doesn't",
            backup ? "reads" : "doesn't");
    if (source != expected) {
      return "odometry turns on " + source + " while " + state + ": expected " + expected;
    }
    if (source == YawSource.BACKUP && !localization.isUsingBackupAlertActive()) {
      return "odometry turns on the backup IMU, and no alert says so";
    }
    if (source == YawSource.WHEELS
        && !readings.gyro().usable()
        && !readings.backupImu().usable()
        && (!localization.isGyroDisconnectedAlertActive()
            || estimate.mode() != LocalizationEstimate.Mode.IMU_FAULT)) {
      return "odometry turns on the wheels while "
          + state
          + ", and it says "
          + estimate.mode()
          + (localization.isGyroDisconnectedAlertActive() ? "" : " with no alert");
    }
    return null;
  }

  private @Nullable String blindSaysSo(LocalizationEstimate estimate) {
    blindLoops = cameras && blind.getAsBoolean() ? blindLoops + 1 : 0;
    // A loop to read the cameras, and a loop for the newest odometry to pass the last image.
    double seconds = (blindLoops - 2) * RobotLoop.PERIOD_SECONDS;
    if (seconds > LocalizationGate.SCORING.maxVisionAgeSeconds() && localization.readyToScore()) {
      return String.format(
          Locale.ROOT,
          "every camera has been unplugged for %.2f s, and the robot says it's ready to score",
          blindLoops * RobotLoop.PERIOD_SECONDS);
    }
    if (seconds > visionTimeoutSeconds && estimate.mode() == LocalizationEstimate.Mode.TRACKING) {
      return String.format(
          Locale.ROOT,
          "every camera has been unplugged for %.2f s, and the estimate says it's tracking tags",
          blindLoops * RobotLoop.PERIOD_SECONDS);
    }
    return null;
  }

  private @Nullable String jumps(
      DrivetrainReadings readings, LocalizationEstimate estimate, boolean enabled) {
    double travel = travel(readings.wheels());
    Pose2d pose = estimate.pose();
    Pose2d last = before;
    long now = estimate.timestampNanos();
    double turned = turnedBeyondImu(readings, pose, last);
    boolean recovered = estimate.health().recoveries() != recoveries;
    double told = 0;
    double reach = travel;
    long earliest = Long.MAX_VALUE;
    long newest = newestApplied;
    for (FrameVerdict frame : frames.get()) {
      if (frame.accepted() && frame.measured().isPresent()) {
        long taken = frame.captureNanos();
        earliest = Math.min(earliest, taken);
        newest = Math.max(newest, taken);
        told += Math.sqrt(frame.measured().get().chiSquared()) * unsureSince(taken);
        if (last != null) {
          reach = Math.max(reach, at(taken).getDistance(last.getTranslation()) + travel);
        }
      }
    }
    // An image taken before one applied already is weighed against the estimate before that one,
    // and moves last loop's estimate by more than its χ² says.
    boolean reweighs = earliest < newestApplied;
    newestApplied = newest;
    before = pose;
    recoveries = estimate.health().recoveries();
    poses.addLast(new Stamped(now, pose, estimate.covariance().largestPositionStdDev()));
    while (poses.size() > KEPT_LOOPS) {
      poses.removeFirst();
    }
    // Disabled, the drive team may place it on its start; and a recovery is a declared jump.
    if (last == null || !enabled || recovered || reweighs) {
      return null;
    }
    double moved = pose.getTranslation().getDistance(last.getTranslation());
    double allowed = travel + told + turned * reach + JUMP_SLACK_METERS;
    if (moved <= allowed) {
      return null;
    }
    return String.format(
        Locale.ROOT,
        "the estimate moved %.3f m this loop, %s to %s, where its wheels traveled %.3f m, the"
            + " images it accepted could move it %.3f m by their gate, and it turned %.2f° beyond"
            + " its IMU, %.3f m from where the earliest was taken: %.3f m at most",
        moved,
        last.getTranslation(),
        pose.getTranslation(),
        travel,
        told,
        Math.toDegrees(turned),
        reach,
        allowed);
  }

  /**
   * How far the estimate turned this loop beyond what the IMU turning odometry turned, in radians:
   * a correction to its heading, which turns the path it replays from an image's capture on, and
   * moves where it ends by the turn times the path's reach. All its turn, without the same IMU both
   * loops.
   */
  private double turnedBeyondImu(DrivetrainReadings readings, Pose2d pose, @Nullable Pose2d last) {
    YawSource source = localization.yawSource();
    ImuReadings imu =
        switch (source) {
          case GYRO -> readings.gyro();
          case BACKUP -> readings.backupImu();
          case WHEELS -> null;
        };
    double yaw = imu != null && imu.usable() ? imu.yawRadians() : Double.NaN;
    double imuTurn =
        source == lastSource && Double.isFinite(yaw) && Double.isFinite(lastYaw)
            ? MathUtil.angleModulus(yaw - lastYaw)
            : Double.NaN;
    lastSource = source;
    lastYaw = yaw;
    if (last == null) {
      return 0;
    }
    double turn = MathUtil.angleModulus(pose.getRotation().minus(last.getRotation()).getRadians());
    return Double.isNaN(imuTurn) ? Math.abs(turn) : Math.abs(MathUtil.angleModulus(turn - imuTurn));
  }

  /**
   * The most the estimate was unsure of its position, its largest standard deviation, from a loop
   * before a time to the loop before this one: what it was as an image taken then was weighed,
   * whatever corrected it since.
   */
  private double unsureSince(long nanos) {
    double most = 0;
    long from = nanos - Math.round(RobotLoop.PERIOD_SECONDS * 1e9);
    for (Stamped each : poses) {
      if (each.nanos() >= from || each == poses.peekLast()) {
        most = Math.max(most, each.unsure());
      }
    }
    return most;
  }

  /** Where the estimate had the robot at a time, as of the loops before: the nearest loop's. */
  private Translation2d at(long nanos) {
    Stamped nearest = null;
    for (Stamped each : poses) {
      if (nearest == null || Math.abs(each.nanos() - nanos) < Math.abs(nearest.nanos() - nanos)) {
        nearest = each;
      }
    }
    return nearest == null ? Translation2d.ZERO : nearest.pose().getTranslation();
  }

  /**
   * The farthest any wheel traveled from the newest reading of the loop before to this loop's
   * newest: the most the wheels can move the robot, whose travel is their average. Nothing for a
   * loop without readings, and nothing from a wheel that didn't report in either.
   */
  private double travel(List<OdometrySample> samples) {
    if (samples.isEmpty()) {
      return 0;
    }
    OdometrySample newest = samples.get(samples.size() - 1);
    double[] now = new double[newest.wheels()];
    double farthest = 0;
    double[] then = wheels;
    for (int i = 0; i < now.length; i++) {
      now[i] = newest.distanceMeters(i);
      if (then != null && i < then.length && Double.isFinite(now[i]) && Double.isFinite(then[i])) {
        farthest = Math.max(farthest, Math.abs(now[i] - then[i]));
      }
    }
    wheels = now;
    return farthest;
  }
}
