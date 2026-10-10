package com.michaelgrundvig.frc.runtime.subsystems.vision;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Milliseconds;

import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.IntToLongFunction;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.interpolation.TimeInterpolatableBuffer;
import org.wpilib.system.RobotController;

/**
 * The simulator's cameras: PhotonVision's own simulation, rendering each camera's view of the tags
 * from where the robot really is (never from the estimate), with the camera's resolution, field of
 * view, frame rate, latency, and a little lens-calibration error. Its results reach the robot code
 * through the same {@link CameraIOPhotonVision} a real camera uses, each image stamped with when it
 * was taken ({@link #update}).
 *
 * <p>A camera can be knocked on its mount ({@link #mount}) or made later than its configuration
 * says ({@link #addLatency}), and a tag moved off where the field was built ({@link #moveTag}),
 * while the simulation runs; the robot code still believes its configuration and the field's
 * layout.
 */
public final class VisionSim implements AutoCloseable {
  /**
   * How far each tag corner lands from where a perfect camera would put it, in pixels, on average
   * and its spread: a little lens-calibration error. PhotonVision's own example values.
   */
  public static final double CORNER_NOISE_MEAN_PIXELS = 0.25;

  public static final double CORNER_NOISE_STD_DEV_PIXELS = 0.08;

  /** How much each image's latency varies, as a standard deviation: PhotonVision's example. */
  public static final double LATENCY_STD_DEV_MILLIS = 5;

  /**
   * How a simulated camera errs from a perfect one: its tag corners' noise, in pixels, and its
   * latency's spread.
   *
   * @param cornerMeanPixels how far each corner lands from where a perfect camera puts it, on
   *     average
   * @param cornerStdDevPixels and that distance's spread
   * @param latencyStdDevMillis how much each image's latency varies about the configured one
   */
  public record Noise(
      double cornerMeanPixels, double cornerStdDevPixels, double latencyStdDevMillis) {
    /** PhotonVision's example camera: corners a quarter pixel off, latency varying by 5 ms. */
    public static final Noise TYPICAL =
        new Noise(CORNER_NOISE_MEAN_PIXELS, CORNER_NOISE_STD_DEV_PIXELS, LATENCY_STD_DEV_MILLIS);

    /** A perfect camera: every corner where the lens puts it, every image exactly as late. */
    public static final Noise NONE = new Noise(0, 0, 0);
  }

  /** How far back the robot's true poses are kept, for the images taken that long ago. */
  private static final double HISTORY_SECONDS = 1.5;

  private final VisionSystemSim world = new VisionSystemSim("main");
  private final Field layout;
  private final IntToLongFunction cameraSeeds;
  private final Noise noise;
  private final List<PhotonCameraSim> cameras = new ArrayList<>();

  /**
   * When this world began, on the robot's clock, in nanoseconds. Its truth is kept by the time
   * since: the clock itself counts from wherever the computer's monotonic clock stood when the
   * simulation started (days of uptime, 10^14 ns and more), where a double's step is a tenth of a
   * nanosecond, so poses kept by the clock's own seconds would interpolate a little differently
   * from run to run, and an image's corners with them.
   */
  private final long startNanos = RobotController.getMonotonicTime();

  /** Where the robot really was, loop by loop, by seconds since the world began. */
  private final TimeInterpolatableBuffer<Pose3d> truths =
      TimeInterpolatableBuffer.createBuffer(HISTORY_SECONDS);

  /**
   * A camera as added: PhotonVision's simulation of it, its configured mount and latency, and where
   * it's mounted now, which its images are taken from.
   */
  private record Added(
      PhotonCameraSim simulated,
      Transform3d configured,
      double latencyMillis,
      Transform3d mountError,
      Transform3d mount) {}

  /**
   * Where each camera on a joint was carried, by name, over the history kept, for the moment each
   * image is taken: on the configured mount's place, before its mount error.
   */
  private final Map<String, TimeInterpolatableBuffer<Pose3d>> carried = new LinkedHashMap<>();

  private final Map<String, Added> byName = new LinkedHashMap<>();

  /** How far each camera's clock is off the robot's, by name, in nanoseconds: none, unless set. */
  private final Map<String, Long> clockOffsets = new LinkedHashMap<>();

  /** The field's tags as the cameras see them, by ID, and where each was as the world was built. */
  private final Map<Integer, VisionTargetSim> tags = new LinkedHashMap<>();

  private final Map<Integer, Pose3d> builtTags = new LinkedHashMap<>();

  /** The tag moved from where it was built, if one is: its ID. */
  private int movedTag = NONE;

  private static final int NONE = Integer.MIN_VALUE;

  /**
   * @param tagErrors by tag ID, how far each tag really sits from where the field's layout says, in
   *     the field's frame, as the world is built: the cameras see it there, while their multi-tag
   *     solves, as the coprocessor's do, believe the layout ({@link #moveTag} moves one further)
   * @param cameraSeeds where each camera's noise starts, by its place in the order the cameras are
   *     added, from 0: each draws its own, so a run repeats exactly, and a run with another seed
   *     sees other noise. The simulator gives each camera its own stream of the world's seed
   *     ({@code Seeds.camera}).
   */
  public VisionSim(
      FieldModel field, Map<Integer, Translation3d> tagErrors, IntToLongFunction cameraSeeds) {
    this(field, tagErrors, cameraSeeds, Noise.TYPICAL);
  }

  /**
   * As above, its cameras erring as {@code noise} says: {@link Noise#NONE} for perfect ones, whose
   * images a test can check against a pinhole camera's.
   */
  public VisionSim(
      FieldModel field,
      Map<Integer, Translation3d> tagErrors,
      IntToLongFunction cameraSeeds,
      Noise noise) {
    this.noise = noise;
    layout = field.toField();
    List<FieldTag> real = new ArrayList<>();
    for (FieldTag tag : layout.getTags()) {
      Translation3d off = tagErrors.getOrDefault(tag.getID(), Translation3d.ZERO);
      Pose3d pose = tag.getPose();
      Pose3d built = new Pose3d(pose.getTranslation().plus(off), pose.getRotation());
      real.add(new FieldTag(tag.getID(), built));
      builtTags.put(tag.getID(), built);
    }
    world.addAprilTags(
        new Field(
            layout.name,
            layout.season,
            layout.game,
            null,
            layout.length,
            layout.width,
            layout.program,
            real));
    for (VisionTargetSim target : world.getVisionTargets("apriltag")) {
      tags.put(target.fiducialID, target);
    }
    this.cameraSeeds = cameraSeeds;
  }

  /** The field's tags' IDs, as its layout lists them. */
  public List<Integer> tagIds() {
    return List.copyOf(builtTags.keySet());
  }

  /**
   * Moves one tag this far from where it was as the world was built, in the field's frame, from now
   * on, putting any other moved tag back: a tag knocked at an event. The robot code, and the
   * cameras' multi-tag solves, still believe the layout.
   *
   * @param by zero puts it back
   * @throws IllegalArgumentException if the field has no tag with this ID
   */
  public void moveTag(int id, Translation3d by) {
    VisionTargetSim target = tags.get(id);
    Pose3d built = builtTags.get(id);
    if (target == null || built == null) {
      throw new IllegalArgumentException("the field has no tag " + id);
    }
    putTagsBack();
    target.setPose(new Pose3d(built.getTranslation().plus(by), built.getRotation()));
    movedTag = by.equals(Translation3d.ZERO) ? NONE : id;
  }

  /** Puts every tag back where it was as the world was built. */
  public void putTagsBack() {
    for (Map.Entry<Integer, Pose3d> built : builtTags.entrySet()) {
      VisionTargetSim target = tags.get(built.getKey());
      if (target != null) {
        target.setPose(built.getValue());
      }
    }
    movedTag = NONE;
  }

  /** Whether a tag is moved from where it was as the world was built. */
  public boolean isTagMoved() {
    return movedTag != NONE;
  }

  /** Where a tag really is now, as the cameras see it; empty if the field has no such tag. */
  public Optional<Pose3d> tagPose(int id) {
    VisionTargetSim target = tags.get(id);
    return target == null ? Optional.empty() : Optional.of(target.getPose());
  }

  /** Adds a camera, mounted exactly as configured, and returns the IO the robot code reads. */
  public CameraIO camera(CameraConfig config) {
    return camera(config, Transform3d.ZERO);
  }

  /**
   * Adds a camera mounted off its configuration (knocked 4 in low, turned 2 degrees), and returns
   * the IO the robot code reads it through, which still believes the configuration.
   *
   * @param mountError in the robot's frame: added to the configured position, and turning the
   *     configured orientation
   */
  public CameraIO camera(CameraConfig config, Transform3d mountError) {
    CameraIOPhotonVision io = new CameraIOPhotonVision(config.name());
    // Seeded, one seed per camera, so a simulated run repeats exactly.
    PhotonCameraSim simulated =
        new PhotonCameraSim(
            io.camera(),
            properties(config.sensor(), cameraSeeds.applyAsLong(cameras.size()), noise),
            layout);
    // Tags farther than the camera's sight range are not detected: small in the image, and noisy.
    simulated.setMaxSightRange(config.sensor().sightRange().in(Meters));
    // No video: PhotonVision renders a raw and a processed frame of every image by default, native
    // memory freed only when the collector gets to it (a study's process grew past 4 GB). Nothing
    // here views them, and what the camera detects doesn't depend on them.
    simulated.enableRawStream(false);
    simulated.enableProcessedStream(false);
    Transform3d configured = config.robotToCamera();
    Transform3d mount = mounted(configured, mountError);
    world.addCamera(simulated, mount);
    cameras.add(simulated);
    byName.put(
        config.name(),
        new Added(
            simulated, configured, config.sensor().latency().in(Milliseconds), mountError, mount));
    return io;
  }

  /**
   * Mounts a camera off its configuration from now on, as {@link #camera(CameraConfig,
   * Transform3d)} does when it's added: images rendered from now on see from there, an image
   * already taken and still on its way included.
   *
   * @param mountError in the robot's frame, from the configured mount; zero puts it back
   * @throws IllegalArgumentException if no camera has this name
   */
  public void mount(String camera, Transform3d mountError) {
    Added added = added(camera);
    Transform3d mount = mounted(added.configured(), mountError);
    world.adjustCamera(added.simulated(), mount);
    byName.put(
        camera,
        new Added(added.simulated(), added.configured(), added.latencyMillis(), mountError, mount));
  }

  /**
   * Where a camera on a joint is now, in the robot's frame, as the joint has carried it: images
   * taken from now on see from there, with its mount error. Call once per loop, before {@link
   * #update}.
   *
   * @throws IllegalArgumentException if no camera has this name
   */
  public void carry(String camera, Transform3d mount) {
    added(camera);
    carried
        .computeIfAbsent(camera, name -> TimeInterpolatableBuffer.createBuffer(HISTORY_SECONDS))
        .addSample(
            sinceStart(RobotController.getMonotonicTime()),
            new Pose3d(mount.getTranslation(), mount.getRotation()));
  }

  /**
   * Makes a camera this much later than its configuration says, from now on: its images arrive
   * later, still carrying when they were taken. PhotonVision's simulation then runs it at no more
   * frames a second than its latency allows.
   *
   * @param extraMillis zero for its configured latency
   * @throws IllegalArgumentException if no camera has this name
   */
  public void addLatency(String camera, double extraMillis) {
    Added added = added(camera);
    added.simulated().prop.setAvgLatencyMs(added.latencyMillis() + extraMillis);
  }

  /**
   * Sets a camera's clock this far off the robot's, from now on: each image is stamped this much
   * later than it was taken (earlier, if negative), as a coprocessor whose time sync is off stamps
   * it, though it arrives when it would have. Unlike {@link #addLatency}, the stamps are wrong.
   *
   * @param millis zero for a clock that agrees with the robot's
   * @throws IllegalArgumentException if no camera has this name
   */
  public void offsetClock(String camera, double millis) {
    added(camera);
    clockOffsets.put(camera, Math.round(millis * 1e6));
  }

  private Added added(String camera) {
    Added added = byName.get(camera);
    if (added == null) {
      throw new IllegalArgumentException("no simulated camera is named " + camera);
    }
    return added;
  }

  /**
   * Where a camera really is, mounted off its configuration: the error added to the configured
   * position, and turning the configured orientation. The camera placement study mounts its cameras
   * off the same way.
   */
  public static Transform3d mounted(Transform3d configured, Transform3d mountError) {
    return new Transform3d(
        configured.getTranslation().plus(mountError.getTranslation()),
        configured.getRotation().rotateBy(mountError.getRotation()));
  }

  /**
   * A camera as PhotonVision's simulation models it: the sensor's resolution, field of view, frame
   * rate, and latency, and a little lens-calibration error, which moves each tag corner about a
   * quarter pixel. The camera placement study takes its images with the same model, and starts from
   * the same errors.
   *
   * @param seed where the noise starts, so a run repeats exactly
   */
  public static SimCameraProperties properties(CameraConfig.Sensor sensor, long seed) {
    return properties(sensor, seed, Noise.TYPICAL);
  }

  /** As above, erring as {@code noise} says. */
  static SimCameraProperties properties(CameraConfig.Sensor sensor, long seed, Noise noise) {
    return new SimCameraProperties()
        .setCalibration(
            sensor.widthPixels(),
            sensor.heightPixels(),
            Rotation2d.fromDegrees(sensor.diagonalFieldOfView().in(Degrees)))
        .setCalibError(noise.cornerMeanPixels(), noise.cornerStdDevPixels())
        .setFPS(sensor.frameRate().in(Hertz))
        .setAvgLatencyMs(sensor.latency().in(Milliseconds))
        .setLatencyStdDevMs(noise.latencyStdDevMillis())
        .setRandomSeed(seed);
  }

  /**
   * Renders every camera's view from where the robot is, tilted as it is. Call once per loop,
   * before reading.
   *
   * <p>An image falls due on the camera's frame rate, and is taken its latency before then, from
   * where the robot was at that moment; it's stamped with that moment, as a real camera's time sync
   * stamps it. PhotonVision's {@code VisionSystemSim.update} takes it the same way but stamps it as
   * of the loop that renders it, up to a loop (20 ms) later. While the robot turns, every image
   * then shows it where it was a little before its stamp, turned a little less, the same way image
   * after image: just what a heading error does to a position solved with the gyro's heading. An
   * estimator that lets every fix correct the heading took it for one, and grew sure of a heading
   * 0.7° off after the gyro-scale procedure (docs/testing.md).
   */
  public void update(Pose3d truth) {
    truths.addSample(sinceStart(RobotController.getMonotonicTime()), truth);
    List<VisionTargetSim> targets = List.copyOf(world.getVisionTargets());
    for (Map.Entry<String, Added> added : byName.entrySet()) {
      PhotonCameraSim camera = added.getValue().simulated();
      Optional<Long> due = camera.consumeNextEntryTime();
      if (due.isEmpty()) {
        continue;
      }
      long arrivedNanos = due.get();
      double latencyMillis = camera.prop.estLatencyMs();
      long takenNanos = arrivedNanos - Math.round(latencyMillis * 1e6);
      Pose3d robot = truths.getSample(sinceStart(takenNanos)).orElse(truth);
      Pose3d from = robot.plus(mountAt(added.getKey(), added.getValue(), takenNanos));
      PhotonPipelineResult image = camera.process(latencyMillis, from, targets);
      image.metadata.captureTimestampNanos =
          takenNanos + clockOffsets.getOrDefault(added.getKey(), 0L);
      image.metadata.publishTimestampNanos = arrivedNanos;
      camera.submitProcessedFrame(image, arrivedNanos);
    }
    // Every image due is taken, so PhotonVision's own update only keeps its record of the robot.
    world.update(truth);
  }

  /**
   * Where a camera was on the robot when an image was taken: carried by its joint, if it rides one.
   */
  private Transform3d mountAt(String camera, Added added, long takenNanos) {
    TimeInterpolatableBuffer<Pose3d> joint = carried.get(camera);
    if (joint == null) {
      return added.mount();
    }
    return joint
        .getSample(sinceStart(takenNanos))
        .map(
            at ->
                mounted(new Transform3d(at.getTranslation(), at.getRotation()), added.mountError()))
        .orElse(added.mount());
  }

  /** How long after this world began a moment on the robot's clock was, in seconds. */
  private double sinceStart(long nanos) {
    return (nanos - startNanos) / 1e9;
  }

  @Override
  public void close() {
    world.clearCameras();
    for (PhotonCameraSim camera : cameras) {
      camera.close();
    }
  }
}
