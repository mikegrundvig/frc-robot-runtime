package com.michaelgrundvig.frc.runtime.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Transform3d;

/**
 * One camera's door to its vision software: PhotonVision on a coprocessor, PhotonVision's
 * simulation, or nothing (in replay, where the inputs come from the log).
 *
 * <p>The inputs are the raw detections, not solved poses, so a replay can run a changed pipeline
 * against a real match. They are plain values that a log can hold, which is why "nothing" is said
 * here with an empty text or a flag. {@link Detections} turns them into the types the pipeline
 * takes.
 */
public interface CameraIO extends AutoCloseable {
  @AutoLog
  class CameraIOInputs {
    public boolean connected = false;

    /** The results that arrived since the last loop, oldest first. */
    public Frame[] frames = new Frame[] {};

    /** Every tag in those results, frame by frame: each frame's tags follow the previous one's. */
    public SeenTag[] tags = new SeenTag[] {};

    /**
     * How many results were left out because they couldn't be placed in time. Such a result had no
     * capture time, or a bad one.
     */
    public int skippedFrames = 0;

    /**
     * Why the camera couldn't be read at all this loop, such as the coprocessor running a different
     * vision software version than the robot's library. It is empty when the camera was read.
     */
    public String error = "";
  }

  /**
   * One image's result.
   *
   * @param timestampNanos when the image was captured, on the robot's clock
   * @param tagCount how many of the tags that follow are this image's
   * @param hasMultiTag whether the vision software solved all the tags at once
   * @param multiTagFieldToCamera that solution, which is the camera's pose on the field. It is a
   *     zero transform when there is no solution.
   * @param multiTagReprojectionErrorPixels how well that solution fits its tags, or zero when there
   *     is no solution
   */
  record Frame(
      long timestampNanos,
      int tagCount,
      boolean hasMultiTag,
      Transform3d multiTagFieldToCamera,
      double multiTagReprojectionErrorPixels) {}

  /**
   * One tag in an image: its four corners in pixels, the two poses a single tag allows, how
   * ambiguous they are, and whether the multi-tag solution used it.
   *
   * @param bestCameraToTag the likelier of the two poses, from the camera to the tag
   * @param altCameraToTag the other of the two
   * @param ambiguity how close the two are to equally likely, from 0 (certain) to 1 (a coin toss).
   *     PhotonVision sends -1 when it has none to give.
   * @param inMultiTag whether the multi-tag solution used this tag
   */
  record SeenTag(
      int id,
      double corner0X,
      double corner0Y,
      double corner1X,
      double corner1Y,
      double corner2X,
      double corner2Y,
      double corner3X,
      double corner3Y,
      Transform3d bestCameraToTag,
      Transform3d altCameraToTag,
      double ambiguity,
      boolean inMultiTag) {}

  default void updateInputs(CameraIOInputs inputs) {}

  @Override
  default void close() {}
}
