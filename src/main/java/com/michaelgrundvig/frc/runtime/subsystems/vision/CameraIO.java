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
     * The numbers of the tags in those results that came without poses, frame by frame like {@link
     * #tags}: Visor sends a tag's corners alone from a camera with no lens calibration, or for a
     * tag seen too edge-on to solve. Empty from PhotonVision, which sends only tags it solved.
     */
    public int[] unsolvedTags = new int[] {};

    /**
     * How many results were left out because they couldn't be placed in time. Such a result had no
     * capture time, or a bad one.
     */
    public int skippedFrames = 0;

    /**
     * The exposure the camera's latest image was taken at, in milliseconds, as Visor read it back
     * from the camera; -1 where it isn't known: the camera chose it (its automatic exposure on), or
     * its software doesn't say (PhotonVision).
     */
    public double exposureMs = -1;

    /**
     * The camera's gain control as Visor read it back, in its own units; -1 where it isn't known.
     */
    public int gain = -1;

    /** Whether the camera's own automatic exposure was on for its latest image (Visor's). */
    public boolean autoExposure = false;

    /**
     * The latest image's median brightness, 0 (black) to 255 (white); -1 until one says (Visor).
     */
    public int brightnessMedian = -1;

    /** Its 99th percentile, about a tag's white where one is in view; -1 until one says. */
    public int brightnessHigh = -1;

    /** The share of its pixels at or near full white, 0 to 1; -1 until one says. */
    public double clippedShare = -1;

    /**
     * The newest frame's number, as Visor counts the camera's frames from its start: a gap is
     * frames the network lost, and a number lower than the last a Visor that restarted. -1 until a
     * frame says (and from PhotonVision, which doesn't).
     */
    public long sequence = -1;

    /**
     * Visor's count of the camera's frames it never sent, from its start, as the newest frame said:
     * replaced by a newer one before detection got to them, or broken. It wraps past 2^32 - 1. -1
     * until a frame says (and from PhotonVision).
     */
    public long dropped = -1;

    /**
     * How long Visor took to find each frame's tags and solve their poses, in milliseconds: one for
     * each frame that arrived since the last loop, those left out included, oldest first. Empty
     * from PhotonVision, which doesn't say.
     */
    public double[] detectMs = new double[] {};

    /**
     * The address of the coprocessor the camera's newest frame came from, where Visor serves its
     * live view: empty until a frame arrives, and in the simulator, whose frames come from no board
     * (and from PhotonVision).
     */
    public String visorAddress = "";

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
   * @param unsolvedTagCount how many of the unsolved tags that follow are this image's
   * @param hasMultiTag whether the vision software solved all the tags at once
   * @param multiTagFieldToCamera that solution, which is the camera's pose on the field. It is a
   *     zero transform when there is no solution.
   * @param multiTagReprojectionErrorPixels how well that solution fits its tags, or zero when there
   *     is no solution
   */
  record Frame(
      long timestampNanos,
      int tagCount,
      int unsolvedTagCount,
      boolean hasMultiTag,
      Transform3d multiTagFieldToCamera,
      double multiTagReprojectionErrorPixels) {
    /** An image whose every tag was solved. */
    public Frame(
        long timestampNanos,
        int tagCount,
        boolean hasMultiTag,
        Transform3d multiTagFieldToCamera,
        double multiTagReprojectionErrorPixels) {
      this(
          timestampNanos,
          tagCount,
          0,
          hasMultiTag,
          multiTagFieldToCamera,
          multiTagReprojectionErrorPixels);
    }
  }

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
