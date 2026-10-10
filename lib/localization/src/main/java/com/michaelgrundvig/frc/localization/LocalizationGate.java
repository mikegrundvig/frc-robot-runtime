package com.michaelgrundvig.frc.localization;

import static com.michaelgrundvig.frc.world.config.Rules.positive;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.Locale;
import org.jspecify.annotations.Nullable;

/**
 * Whether an estimate is good enough to act on where precision matters: scoring, aiming, a final
 * approach. A precise-looking pose from dead reckoning, or one the cameras just disagreed with, is
 * not one to shoot from. A top team gated its scoring on vision within 0.5 s and agreeing within
 * 0.2 m; Top teams would not shoot while tilted.
 *
 * <p>Commands ask {@link #whyNot} before committing, and show the reason to the drive team when the
 * answer is no.
 *
 * @param maxVisionAgeSeconds the newest accepted camera measurement may be no older than this
 * @param maxDisagreementMeters and the measurements accepted lately may have been no farther than
 *     this from the estimate they corrected
 * @param maxPositionStdDevMeters the position's uncertainty, along its worst direction, may be no
 *     more than this
 * @param maxTiltDegrees the robot may be tilted no more than this
 */
@ConfigRecord
public record LocalizationGate(
    double maxVisionAgeSeconds,
    double maxDisagreementMeters,
    double maxPositionStdDevMeters,
    double maxTiltDegrees) {
  /** For scoring: a top team's thresholds, and level. */
  public static final LocalizationGate SCORING =
      builder()
          .maxVisionAgeSeconds(0.5)
          .maxDisagreementMeters(0.2)
          .maxPositionStdDevMeters(0.15)
          .maxTiltDegrees(5)
          .build();

  public LocalizationGate {
    Checks.on(LocalizationGate.class)
        .field("maxVisionAgeSeconds", maxVisionAgeSeconds, positive())
        .field("maxDisagreementMeters", maxDisagreementMeters, positive())
        .field("maxPositionStdDevMeters", maxPositionStdDevMeters, positive())
        .field("maxTiltDegrees", maxTiltDegrees, positive())
        .orThrow();
  }

  /** Starts building a gate, one named limit at a time. */
  public static LocalizationGateBuilder.MaxVisionAgeSecondsStage builder() {
    return LocalizationGateBuilder.builder();
  }

  /**
   * Why the estimate isn't good enough, for the drive team to read; null when it is.
   *
   * @return the first reason found, the most basic first
   */
  public @Nullable String whyNot(LocalizationEstimate estimate) {
    LocalizationEstimate.Health health = estimate.health();
    switch (estimate.mode()) {
      case TRACKING -> {}
      case INITIALIZING -> {
        return "not placed or seen yet";
      }
      case DEAD_RECKONING -> {
        return "no camera measurement lately";
      }
      case LOST -> {
        return "the cameras disagree with the estimate";
      }
      case IMU_FAULT -> {
        return "the gyro is lost";
      }
    }
    if (!(health.visionAgeSeconds() <= maxVisionAgeSeconds)) {
      return String.format(
          Locale.ROOT, "the last camera measurement is %.1f s old", health.visionAgeSeconds());
    }
    if (!(health.recentDisagreementMeters() <= maxDisagreementMeters)) {
      return Double.isNaN(health.recentDisagreementMeters())
          ? "no camera measurement lately"
          : String.format(
              Locale.ROOT,
              "the cameras and odometry disagree by %.2f m",
              health.recentDisagreementMeters());
    }
    double spread = estimate.covariance().largestPositionStdDev();
    if (!(spread <= maxPositionStdDevMeters)) {
      return String.format(Locale.ROOT, "the position is uncertain by %.2f m", spread);
    }
    double tilt = Math.toDegrees(health.tiltRadians());
    if (!(tilt <= maxTiltDegrees)) {
      return String.format(Locale.ROOT, "the robot is tilted %.0f degrees", tilt);
    }
    if (health.collisionActive()) {
      return "the robot was just hit";
    }
    return null;
  }

  /** Whether the estimate is good enough. */
  public boolean allows(LocalizationEstimate estimate) {
    return whyNot(estimate) == null;
  }
}
