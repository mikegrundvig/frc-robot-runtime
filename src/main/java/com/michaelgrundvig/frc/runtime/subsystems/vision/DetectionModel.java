package com.michaelgrundvig.frc.runtime.subsystems.vision;

import java.util.Arrays;

/**
 * How likely an AprilTag detector is to find a tag, by how big the tag is in the image: the length
 * of its longest side, in pixels (a tag's height in the image, for a tag turned about the
 * vertical). Between the points given the chance is interpolated in a straight line; below the
 * first it's the first's, above the last the last's.
 *
 * <p>The presets are measured, on 1,152 rendered images of tags at 1 to 10 m and 0 to 55 degrees,
 * as a 1600 × 1304 camera with an 80 degree lens sees them, clean (no blur): each distance's share
 * found, at the size a tag square to the camera is at that distance. They say nothing about blur,
 * which hurt every detector there, nor about how fast each runs.
 *
 * @param name for reports
 * @param pixels the sizes, rising
 * @param chances the chance of being found at each size, 0 to 1
 */
public record DetectionModel(String name, double[] pixels, double[] chances) {
  /** WPILib's AprilTag detector at decimate 2, the fastest setting: found to about 3 m there. */
  public static final DetectionModel WPILIB_DECIMATE_2 =
      new DetectionModel(
          "WPILib decimate 2", new double[] {51, 68, 101}, new double[] {0, 0.25, 0.97});

  /** WPILib's AprilTag detector at decimate 1, full resolution: found to about 5 m. */
  public static final DetectionModel WPILIB_DECIMATE_1 =
      new DetectionModel(
          "WPILib decimate 1",
          new double[] {25, 34, 41, 51, 68, 101},
          new double[] {0, 0.28, 0.78, 0.92, 0.97, 1});

  /**
   * NAPA with {@code min_tag_px} 16, Visor's setting: found to 10 m, the farthest measured (20 px).
   * Below that it's unmeasured; it's taken to fall to nothing at 15 px, under the 16 it's set for.
   */
  public static final DetectionModel NAPA =
      new DetectionModel(
          "NAPA", new double[] {15, 20, 25, 34, 41}, new double[] {0, 0.92, 0.94, 0.94, 1});

  public DetectionModel {
    if (pixels.length == 0 || pixels.length != chances.length) {
      throw new IllegalArgumentException("a chance for each size, and at least one");
    }
    for (int i = 0; i < pixels.length; i++) {
      if (i > 0 && !(pixels[i] > pixels[i - 1])) {
        throw new IllegalArgumentException("the sizes rise: " + Arrays.toString(pixels));
      }
      if (!(chances[i] >= 0 && chances[i] <= 1)) {
        throw new IllegalArgumentException("a chance is 0 to 1: " + chances[i]);
      }
    }
    pixels = pixels.clone();
    chances = chances.clone();
  }

  /** The chance a tag this big in the image is found. */
  public double chance(double longestSidePixels) {
    if (longestSidePixels <= pixels[0]) {
      return chances[0];
    }
    for (int i = 1; i < pixels.length; i++) {
      if (longestSidePixels <= pixels[i]) {
        double along = (longestSidePixels - pixels[i - 1]) / (pixels[i] - pixels[i - 1]);
        return chances[i - 1] + along * (chances[i] - chances[i - 1]);
      }
    }
    return chances[chances.length - 1];
  }

  @Override
  public boolean equals(Object other) {
    return other instanceof DetectionModel model
        && name.equals(model.name)
        && Arrays.equals(pixels, model.pixels)
        && Arrays.equals(chances, model.chances);
  }

  @Override
  public int hashCode() {
    return 31 * (31 * name.hashCode() + Arrays.hashCode(pixels)) + Arrays.hashCode(chances);
  }

  @Override
  public String toString() {
    return name;
  }
}
