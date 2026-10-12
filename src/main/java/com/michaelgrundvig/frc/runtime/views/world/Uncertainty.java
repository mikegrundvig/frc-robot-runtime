package com.michaelgrundvig.frc.runtime.views.world;

import com.michaelgrundvig.frc.localization.LocalizationEstimate;
import com.michaelgrundvig.frc.localization.LocalizerReport;
import com.michaelgrundvig.frc.localization.PoseCovariance;
import com.michaelgrundvig.frc.pitwall.scene.Inspection;
import com.michaelgrundvig.frc.pitwall.scene.Layer;
import com.michaelgrundvig.frc.pitwall.scene.SceneBuilder;
import com.michaelgrundvig.frc.pitwall.scene.SceneState;
import com.michaelgrundvig.frc.pitwall.scene.Style;
import java.util.Locale;
import java.util.Optional;
import org.jspecify.annotations.Nullable;

/**
 * How sure localization is of where the robot is. It is an ellipse around the estimate that reaches
 * two standard deviations of the position's uncertainty each way. The ellipse is dashed, since it's
 * a bound and not a thing, and it is drawn in the color of what's wrong while the estimate is lost
 * or the gyro is.
 */
final class Uncertainty implements Layer {
  /** The ellipse's frame on the page, which sits at the estimate, turned along its longest axis. */
  private static final String FRAME = "uncertainty";

  /** How many standard deviations the ellipse reaches. */
  private static final double SIGMAS = 2;

  /**
   * The largest radius drawn, in meters. An estimate not yet placed is uncertain across the whole
   * field, and beyond.
   */
  private static final double MAX_RADIUS = 20;

  private final WorldView.Readings readings;

  /**
   * The estimate last drawn, for a click to describe, or null before any was. A click reads it on a
   * web server thread, so the loop hands the record over whole, through a volatile field. It is a
   * plain reference because the loop sets it every loop, where an {@code Optional} would be made
   * again each time.
   */
  private volatile @Nullable LocalizerReport drawn;

  /** What a click says in place of something the localizer does not report. */
  private static final String NOT_REPORTED = "not reported by this localizer";

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

  /**
   * Places the ellipse at the estimate and sizes it by the position's covariance.
   *
   * <p>The covariance of the position is the matrix {@code [xx xy; xy yy]}. Its eigenvectors are
   * the ellipse's axes, and its eigenvalues are the variances along them:
   *
   * <pre>
   * mean   = (xx + yy) / 2
   * spread = sqrt(((xx - yy) / 2)² + xy²)
   * variance along the longer axis  = mean + spread
   * variance along the shorter axis = mean - spread
   * angle of the longer axis        = atan2(2 xy, xx - yy) / 2
   * </pre>
   */
  @Override
  public void update(SceneState state) {
    LocalizerReport report = readings.report();
    drawn = report;
    if (report.covariance().isEmpty()) {
      // A localizer that doesn't say how sure it is has no ellipse: none is drawn, of any size.
      state.pose(FRAME, report.pose().getX(), report.pose().getY(), 0);
      state.value("uncertainty.major", 0);
      state.value("uncertainty.minor", 0);
      return;
    }
    PoseCovariance covariance = report.covariance().get();
    double mean = (covariance.xx() + covariance.yy()) / 2;
    double spread = Math.hypot((covariance.xx() - covariance.yy()) / 2, covariance.xy());
    double angle = 0.5 * Math.atan2(2 * covariance.xy(), covariance.xx() - covariance.yy());
    state.pose(FRAME, report.pose().getX(), report.pose().getY(), angle);
    state.value("uncertainty.major", radius(mean + spread));
    state.value("uncertainty.minor", radius(mean - spread));
    // A localizer that doesn't say what its pose rests on is drawn as described: nothing here knows
    // of anything wrong.
    Optional<LocalizationEstimate.Mode> mode = report.mode();
    boolean somethingIsWrong =
        mode.isPresent()
            && (mode.get() == LocalizationEstimate.Mode.LOST
                || mode.get() == LocalizationEstimate.Mode.IMU_FAULT);
    if (somethingIsWrong) {
      state.style("uncertainty.ellipse", Style.BAD.dashed());
    } else {
      // Pitwall takes null for "as it was described".
      state.style("uncertainty.ellipse", null);
    }
  }

  /**
   * How far the ellipse reaches along an axis with this variance: two standard deviations, and no
   * farther than the largest radius drawn. Rounding can leave the shorter axis's variance a hair
   * below zero, which is taken as zero.
   */
  private static double radius(double variance) {
    return Math.min(MAX_RADIUS, SIGMAS * Math.sqrt(Math.max(0, variance)));
  }

  /** What a click on the ellipse says: how sure, along which way, and what it rests on. */
  private Inspection inspection() {
    LocalizerReport report = drawn;
    if (report == null) {
      return Inspection.of("Uncertainty").detail("Estimate", "not drawn yet");
    }
    Inspection said =
        Inspection.of("Uncertainty of the estimate")
            .detail("Based on", report.mode().map(Uncertainty::words).orElse(NOT_REPORTED));
    if (report.covariance().isEmpty()) {
      said =
          said.detail("Position, 1σ", NOT_REPORTED)
              .detail("Heading, 1σ", NOT_REPORTED)
              .detail("Cameras", report.health().map(Uncertainty::cameras).orElse(NOT_REPORTED));
      return said.detail("Drawn", "nothing: the localizer reports no uncertainty");
    }
    PoseCovariance covariance = report.covariance().get();
    return said.detail(
            "Position, 1σ",
            String.format(
                Locale.ROOT,
                "%.1f cm along x, %.1f cm along y, %.1f cm the least sure way",
                100 * covariance.xStdDev(),
                100 * covariance.yStdDev(),
                100 * covariance.largestPositionStdDev()))
        .detail("Heading, 1σ", headingSpread(covariance))
        .detail("Cameras", report.health().map(Uncertainty::cameras).orElse(NOT_REPORTED))
        .detail("Drawn", "two standard deviations across, about the estimate");
  }

  /** A mode in the page's words: "dead reckoning". */
  private static String words(LocalizationEstimate.Mode mode) {
    return mode.name().toLowerCase(Locale.ROOT).replace('_', ' ');
  }

  /** How the cameras' measurements have fared, from the estimate's health. */
  private static String cameras(LocalizationEstimate.Health health) {
    return String.format(
        Locale.ROOT,
        "%d accepted, %d rejected; last accepted %s",
        health.accepted(),
        health.rejected(),
        lastAccepted(health));
  }

  private static String headingSpread(PoseCovariance covariance) {
    if (!covariance.hasHeading()) {
      return "unknown";
    }
    return String.format(Locale.ROOT, "%.2f°", Math.toDegrees(covariance.headingStdDev()));
  }

  /**
   * How long ago the last accepted image was taken. The estimate's health carries an infinite age
   * until one has been accepted.
   */
  private static String lastAccepted(LocalizationEstimate.Health health) {
    if (!Double.isFinite(health.visionAgeSeconds())) {
      return "never";
    }
    return String.format(Locale.ROOT, "%.1f s ago", health.visionAgeSeconds());
  }
}
