package com.michaelgrundvig.frc.localization;

import java.util.List;
import java.util.Objects;

/**
 * What one {@link LocalizationService#update} made of the loop: the estimate, how far each source
 * is trusted, and what became of each camera measurement it weighed. Immutable, so the loop can
 * hand it to a view on another thread whole.
 *
 * @param estimate the estimate, with its uncertainty, mode, and health
 * @param trust how far each source is trusted
 * @param frames what became of each camera image given since the last update: those refused or held
 *     back before they became measurements first, in the order they came; then the measurements, in
 *     the order the estimator decided them (those it couldn't apply, then the rest by capture time)
 */
@OutputRecord
public record LocalizationOutput(
    LocalizationEstimate estimate, Trust trust, List<FrameVerdict> frames) {
  public LocalizationOutput {
    Objects.requireNonNull(estimate, "the estimate");
    Objects.requireNonNull(trust, "the trust");
    frames = List.copyOf(frames);
  }

  /** Builds one by name: no frames, unless said. */
  public static LocalizationOutputBuilder builder() {
    return LocalizationOutputBuilder.builder();
  }
}
