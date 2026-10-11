package com.michaelgrundvig.frc.runtime.subsystems.vision;

import com.michaelgrundvig.frc.localization.TagDetections;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Turns one image's tags, as a camera's IO logs them, into the detections the vision pipeline
 * takes.
 *
 * <p>The IO's records are plain values that a log can hold, so they say "nothing" with a flag or a
 * number out of range, and a coprocessor can send numbers that aren't numbers. This is where those
 * are caught, before anything is solved from them:
 *
 * <ul>
 *   <li>A tag whose pose isn't a number is left out.
 *   <li>An ambiguity that isn't one counts as a coin toss.
 *   <li>A multi-tag solution that isn't a number, or that used none of the tags kept, is left out.
 * </ul>
 */
final class Detections {
  /** The ambiguity of a tag whose two poses are equally likely. */
  private static final double COIN_TOSS = 1;

  private Detections() {}

  /**
   * One image's tags.
   *
   * @param frame the image
   * @param allTags every tag of every image the camera read this loop, each image's tags following
   *     the image before's
   * @param firstTag where this image's tags start in that list
   * @param camera the camera's name
   */
  static TagDetections inImage(
      CameraIO.Frame frame, CameraIO.SeenTag[] allTags, int firstTag, String camera) {
    List<TagDetections.Tag> tags = new ArrayList<>();
    List<Integer> usedByMultiTag = new ArrayList<>();
    // A list shorter than its images say is read as far as it goes.
    int end = Math.min(firstTag + frame.tagCount(), allTags.length);
    for (int k = firstTag; k < end; k++) {
      CameraIO.SeenTag tag = allTags[k];
      boolean posesAreNumbers = isFinite(tag.bestCameraToTag()) && isFinite(tag.altCameraToTag());
      if (!posesAreNumbers) {
        continue;
      }
      tags.add(detection(tag));
      if (tag.inMultiTag()) {
        usedByMultiTag.add(tag.id());
      }
    }
    // The pipeline's record takes null for an image without a multi-tag solution.
    return new TagDetections(
        frame.timestampNanos(), camera, tags, multiTag(frame, usedByMultiTag).orElse(null));
  }

  private static TagDetections.Tag detection(CameraIO.SeenTag tag) {
    return new TagDetections.Tag(
        tag.id(),
        List.of(
            new Translation2d(tag.corner0X(), tag.corner0Y()),
            new Translation2d(tag.corner1X(), tag.corner1Y()),
            new Translation2d(tag.corner2X(), tag.corner2Y()),
            new Translation2d(tag.corner3X(), tag.corner3Y())),
        tag.bestCameraToTag(),
        tag.altCameraToTag(),
        ambiguity(tag.ambiguity()));
  }

  /**
   * A tag's ambiguity, from 0 (certain) to 1 (a coin toss). PhotonVision reports -1 when it has no
   * ambiguity to give, and that, like anything else that isn't an ambiguity, is taken as unknown: a
   * coin toss.
   */
  private static double ambiguity(double reported) {
    boolean known = reported >= 0;
    if (!known) {
      return COIN_TOSS;
    }
    return Math.min(reported, COIN_TOSS);
  }

  /**
   * The vision software's solution from all of an image's tags at once, if it made one that can be
   * used. A reprojection error below zero is taken as zero.
   */
  private static Optional<TagDetections.MultiTag> multiTag(
      CameraIO.Frame frame, List<Integer> usedByMultiTag) {
    boolean usable =
        frame.hasMultiTag()
            && !usedByMultiTag.isEmpty()
            && isFinite(frame.multiTagFieldToCamera())
            && Double.isFinite(frame.multiTagReprojectionErrorPixels());
    if (!usable) {
      return Optional.empty();
    }
    return Optional.of(
        new TagDetections.MultiTag(
            frame.multiTagFieldToCamera(),
            Math.max(0, frame.multiTagReprojectionErrorPixels()),
            usedByMultiTag));
  }

  /** Whether every number of a transform is a number. */
  private static boolean isFinite(Transform3d transform) {
    Quaternion rotation = transform.getRotation().getQuaternion();
    return Double.isFinite(transform.getX())
        && Double.isFinite(transform.getY())
        && Double.isFinite(transform.getZ())
        && Double.isFinite(rotation.getW())
        && Double.isFinite(rotation.getX())
        && Double.isFinite(rotation.getY())
        && Double.isFinite(rotation.getZ());
  }
}
