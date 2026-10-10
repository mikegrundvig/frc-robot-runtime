package com.michaelgrundvig.frc.localization;

import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.positive;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.ArrayDeque;
import org.wpilib.math.util.MathUtil;

/**
 * Compares two IMUs' yaw, to catch one failing while both still report: a gyro that froze, lost its
 * mount, or saturated in a hit. Over a sliding window, the two should turn the same amount; when
 * they drift apart by more than the tolerance (plus a share of the turning, since two gyros' scales
 * differ a little), one of them is wrong. Which one, this can't say: it raises the question rather
 * than silently picking an answer.
 */
public final class ImuCrossCheck {
  private final long windowNanos;
  private final double toleranceRadians;
  private final double scaleTolerance;
  private final ArrayDeque<Reading> readings = new ArrayDeque<>();
  private double primaryTurned;
  private double backupTurned;
  private double lastPrimary;
  private double lastBackup;

  private record Reading(long time, double primary, double backup) {}

  /**
   * How closely two IMUs must agree.
   *
   * @param windowSeconds how far back the comparison looks
   * @param toleranceDegrees how far apart the two may turn over the window, however little it
   *     turned
   * @param scaleTolerance and this share of the turning on top: two gyros whose scales differ by 1%
   *     disagree by 1% of every turn
   */
  @ConfigRecord
  public record Settings(double windowSeconds, double toleranceDegrees, double scaleTolerance) {
    /**
     * Over a second, within 3° and 5% of the turn: a gyro and an IMU whose scales differ a little.
     */
    public static final Settings DEFAULTS =
        builder().windowSeconds(1.0).toleranceDegrees(3).scaleTolerance(0.05).build();

    public Settings {
      Checks.on(Settings.class)
          .field("windowSeconds", windowSeconds, positive())
          .field("toleranceDegrees", toleranceDegrees, positive())
          .field("scaleTolerance", scaleTolerance, nonNegative())
          .orThrow();
    }

    /** Starts building the settings, one named value at a time. */
    public static ImuCrossCheckSettingsBuilder.WindowSecondsStage builder() {
      return ImuCrossCheckSettingsBuilder.builder();
    }
  }

  public ImuCrossCheck(Settings settings) {
    windowNanos = Timestamps.fromSeconds(settings.windowSeconds());
    toleranceRadians = Math.toRadians(settings.toleranceDegrees());
    scaleTolerance = settings.scaleTolerance();
  }

  /** Both IMUs' yaw, once a loop while both report. */
  public void update(long timestampNanos, double primaryYawRadians, double backupYawRadians) {
    if (!readings.isEmpty()) {
      primaryTurned += MathUtil.angleModulus(primaryYawRadians - lastPrimary);
      backupTurned += MathUtil.angleModulus(backupYawRadians - lastBackup);
    }
    lastPrimary = primaryYawRadians;
    lastBackup = backupYawRadians;
    readings.addLast(new Reading(timestampNanos, primaryTurned, backupTurned));
    while (readings.size() > 2 && readings.getFirst().time() < timestampNanos - windowNanos) {
      readings.removeFirst();
    }
  }

  /** Starts over: one of them stopped reporting. */
  public void reset() {
    readings.clear();
    primaryTurned = 0;
    backupTurned = 0;
  }

  /**
   * How far apart the two turned over the window, in radians: the primary's turn less the backup's.
   */
  public double disagreementRadians() {
    if (readings.size() < 2) {
      return 0;
    }
    Reading first = readings.getFirst();
    Reading last = readings.getLast();
    return (last.primary() - first.primary()) - (last.backup() - first.backup());
  }

  /** How far the primary turned over the window, in radians, either way. */
  public double turnedRadians() {
    if (readings.size() < 2) {
      return 0;
    }
    return Math.abs(readings.getLast().primary() - readings.getFirst().primary());
  }

  /** Whether the two disagree by more than the tolerance allows. */
  public boolean disagrees() {
    return Math.abs(disagreementRadians()) > toleranceRadians + scaleTolerance * turnedRadians();
  }
}
