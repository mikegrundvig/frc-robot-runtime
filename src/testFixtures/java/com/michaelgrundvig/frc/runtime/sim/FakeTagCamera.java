package com.michaelgrundvig.frc.runtime.sim;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import com.michaelgrundvig.frc.localization.TagDetections;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Tag;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * A camera that reports raw AprilTag detections, as a coprocessor does, for the drivetrain's
 * localization tests: lighter than PhotonVision's simulated cameras ({@code VisionSim}), and with
 * nothing drawn. It looks from the robot's true pose, tilted as the robot is, through its
 * configured mount, and sees each tag within its range and field of view that faces it, the tag's
 * pose from the camera off by a seeded draw that grows with the distance. With two tags or more,
 * the coprocessor's multi-tag solution comes too. Each frame arrives a latency after it was taken.
 */
public final class FakeTagCamera {
  /** How sure the multi-tag solution is: well within what a heading may be taken from. */
  private static final double MULTI_TAG_REPROJECTION_PIXELS = 0.4;

  /** How ambiguous each tag's own solve is: clear enough for a single tag to be used. */
  private static final double AMBIGUITY = 0.1;

  private static final List<Translation2d> CORNERS =
      List.of(
          new Translation2d(600, 380),
          new Translation2d(680, 380),
          new Translation2d(680, 420),
          new Translation2d(600, 420));

  private final DriveSimulation robot;
  private final CameraConfig camera;
  private final FieldModel field;
  private final double stdDevPerMeter;
  private final long latencyNanos;
  private final long frameNanos;
  private final Random noise;
  private final ArrayDeque<TagDetections> inFlight = new ArrayDeque<>();
  private long lastFrameNanos = Long.MIN_VALUE;

  /**
   * @param stdDevPerMeter how far off each tag's position from the camera is, per meter away
   */
  public FakeTagCamera(
      DriveSimulation robot,
      CameraConfig camera,
      FieldModel field,
      double stdDevPerMeter,
      double latencySeconds,
      double framesPerSecond,
      long seed) {
    this.robot = robot;
    this.camera = camera;
    this.field = field;
    this.stdDevPerMeter = stdDevPerMeter;
    latencyNanos = Math.round(latencySeconds * 1e9);
    frameNanos = Math.round(1e9 / framesPerSecond);
    noise = new Random(seed);
  }

  /** Takes a picture if one is due, and returns the frames that have arrived by now. */
  public List<TagDetections> poll(long nowNanos) {
    if (lastFrameNanos == Long.MIN_VALUE || nowNanos - lastFrameNanos >= frameNanos) {
      lastFrameNanos = nowNanos;
      inFlight.addLast(look(nowNanos));
    }
    List<TagDetections> arrived = new ArrayList<>();
    while (!inFlight.isEmpty() && inFlight.getFirst().timestampNanos() + latencyNanos <= nowNanos) {
      arrived.add(inFlight.removeFirst());
    }
    return arrived;
  }

  /** What the camera sees now. */
  private TagDetections look(long nowNanos) {
    Pose3d cameraPose = robot.truePose3d().transformBy(camera.robotToCamera());
    double halfView = camera.sensor().diagonalFieldOfView().in(Radians) / 2;
    double range = camera.sensor().sightRange().in(Meters);
    List<TagDetections.Tag> tags = new ArrayList<>();
    for (Tag tag : field.tags().values()) {
      Transform3d view = new Transform3d(cameraPose, tag.pose());
      double distance = view.getTranslation().getNorm();
      boolean inView =
          view.getX() > 0
              && distance <= range
              && Math.atan2(Math.hypot(view.getY(), view.getZ()), view.getX()) <= halfView;
      boolean facing = new Transform3d(tag.pose(), cameraPose).getX() > 0;
      if (!inView || !facing) {
        continue;
      }
      double spread = stdDevPerMeter * distance;
      Transform3d seen =
          new Transform3d(
              view.getTranslation()
                  .plus(
                      new Translation3d(
                          spread * noise.nextGaussian(),
                          spread * noise.nextGaussian(),
                          spread * noise.nextGaussian())),
              view.getRotation());
      tags.add(new TagDetections.Tag(tag.id(), CORNERS, seen, Transform3d.ZERO, AMBIGUITY));
    }
    TagDetections.MultiTag multiTag = null;
    if (tags.size() >= 2) {
      double spread = stdDevPerMeter;
      multiTag =
          new TagDetections.MultiTag(
              new Transform3d(
                  cameraPose
                      .getTranslation()
                      .plus(
                          new Translation3d(
                              spread * noise.nextGaussian(), spread * noise.nextGaussian(), 0)),
                  cameraPose.getRotation()),
              MULTI_TAG_REPROJECTION_PIXELS,
              tags.stream().map(TagDetections.Tag::id).toList());
    }
    return new TagDetections(nowNanos, camera.name(), tags, multiTag);
  }
}
