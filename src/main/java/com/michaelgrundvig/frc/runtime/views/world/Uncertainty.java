package com.michaelgrundvig.frc.runtime.views.world;

import com.michaelgrundvig.frc.localization.LocalizationEstimate;
import com.michaelgrundvig.frc.localization.PoseCovariance;
import com.michaelgrundvig.frc.pitwall.scene.Inspection;
import com.michaelgrundvig.frc.pitwall.scene.Layer;
import com.michaelgrundvig.frc.pitwall.scene.SceneBuilder;
import com.michaelgrundvig.frc.pitwall.scene.SceneState;
import com.michaelgrundvig.frc.pitwall.scene.Style;
import java.util.Locale;
import org.jspecify.annotations.Nullable;

/**
 * How sure localization is of where the robot is: an ellipse two standard deviations across its
 * position's uncertainty, about the estimate, dashed since it's a bound and not a thing. Drawn in
 * the color of what's wrong while the estimate is lost or the gyro is.
 */
final class Uncertainty implements Layer {
  /** The ellipse's frame on the page: at the estimate, turned along its longest axis. */
  private static final String FRAME = "uncertainty";

  /** How many standard deviations the ellipse reaches. */
  private static final double SIGMAS = 2;

  /** The largest radius drawn: an estimate not yet placed is uncertain across the whole field. */
  private static final double MAX_RADIUS = 20;

  private final WorldView.Readings readings;

  /** The estimate last drawn, for a click to describe: an immutable record, published whole. */
  private volatile @Nullable LocalizationEstimate drawn;

  Uncertainty(WorldView.Readings readings) {
    this.readings = readings;
  }

  @Override
  public String id() {
    return "uncertainty";
  }

  @Override
  public String title() {
    return "Uncertainty (2σ)";
  }

  @Override
  public void describe(SceneBuilder scene) {
    scene
        .frame(FRAME, SceneBuilder.ROOT)
        .deadband(FRAME, RobotLayer.DEADBAND_METERS, RobotLayer.DEADBAND_RADIANS)
        .value("uncertainty.major", 0.005)
        .value("uncertainty.minor", 0.005);
    scene
        .cylinder("uncertainty.ellipse", FRAME)
        .radii("uncertainty.major", "uncertainty.minor")
        .at(0, 0, 0.03)
        .outline()
        .style(Style.ACCENT.dashed())
        .inspect(this::inspection);
  }

  @Override
  public void update(SceneState state) {
    LocalizationEstimate estimate = readings.estimate();
    drawn = estimate;
    PoseCovariance covariance = estimate.covariance();
    // The covariance's axes: its eigenvectors, the longest first.
    double mean = (covariance.xx() + covariance.yy()) / 2;
    double spread = Math.hypot((covariance.xx() - covariance.yy()) / 2, covariance.xy());
    double angle = 0.5 * Math.atan2(2 * covariance.xy(), covariance.xx() - covariance.yy());
    state.pose(FRAME, estimate.pose().getX(), estimate.pose().getY(), angle);
    state.value("uncertainty.major", radius(mean + spread));
    state.value("uncertainty.minor", radius(mean - spread));
    boolean wrong =
        estimate.mode() == LocalizationEstimate.Mode.LOST
            || estimate.mode() == LocalizationEstimate.Mode.IMU_FAULT;
    state.style("uncertainty.ellipse", wrong ? Style.BAD.dashed() : null);
  }

  private static double radius(double variance) {
    return Math.min(MAX_RADIUS, SIGMAS * Math.sqrt(Math.max(0, variance)));
  }

  /** What a click on the ellipse says: how sure, along which way, and what it rests on. */
  private Inspection inspection() {
    LocalizationEstimate estimate = drawn;
    if (estimate == null) {
      return Inspection.of("Uncertainty").detail("Estimate", "not drawn yet");
    }
    PoseCovariance covariance = estimate.covariance();
    return Inspection.of("Uncertainty of the estimate")
        .detail("Based on", estimate.mode().name().toLowerCase(Locale.ROOT).replace('_', ' '))
        .detail(
            "Position, 1σ",
            String.format(
                Locale.ROOT,
                "%.1f cm along x, %.1f cm along y, %.1f cm the least sure way",
                100 * covariance.xStdDev(),
                100 * covariance.yStdDev(),
                100 * covariance.largestPositionStdDev()))
        .detail(
            "Heading, 1σ",
            covariance.hasHeading()
                ? String.format(Locale.ROOT, "%.2f°", Math.toDegrees(covariance.headingStdDev()))
                : "unknown")
        .detail(
            "Cameras",
            String.format(
                Locale.ROOT,
                "%d accepted, %d rejected; last accepted %s",
                estimate.health().accepted(),
                estimate.health().rejected(),
                Double.isFinite(estimate.health().visionAgeSeconds())
                    ? String.format(Locale.ROOT, "%.1f s ago", estimate.health().visionAgeSeconds())
                    : "never"))
        .detail("Drawn", "two standard deviations across, about the estimate");
  }
}
