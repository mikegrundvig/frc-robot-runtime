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
import java.util.OptionalInt;
import java.util.function.IntToLongFunction;
import org.photonvision.PhotonCamera;
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
import org.wpilib.vision.camera.VideoSink;
import org.wpilib.vision.stream.CameraServer;

/**
 * The simulator's cameras. This is PhotonVision's own simulation, rendering each camera's view of
 * the tags from where the robot really is (never from the estimate), with the camera's resolution,
 * field of view, frame rate, latency, and a little lens-calibration error. Its results reach the
 * robot code through the same {@link CameraIOPhotonVision} a real camera uses, each image stamped
 * with when it was taken ({@link #update}).
 *
 * <p>While the simulation runs, a camera can be knocked on its mount ({@link #mount}), made later
 * than its configuration says ({@link #addLatency}), or have its clock set off the robot's ({@link
 * #offsetClock}), and a tag can be moved off where the field was built ({@link #moveTag}). The
 * robot code still believes its configuration and the field's layout.
 */
public final class VisionSim implements AutoCloseable {
  /**
   * How far each tag corner lands from where a perfect camera would put it, in pixels, on average.
   * It stands for a little lens-calibration error. The value is PhotonVision's own example.
   */
  public static final double CORNER_NOISE_MEAN_PIXELS = 0.25;

  /** The spread of that distance, in pixels. PhotonVision's example again. */
  public static final double CORNER_NOISE_STD_DEV_PIXELS = 0.08;

  /** How much each image's latency varies, as a standard deviation. PhotonVision's example. */
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

  private static final double NANOS_PER_MILLI = 1e6;
  private static final double NANOS_PER_SECOND = 1e9;

  private final VisionSystemSim world = new VisionSystemSim("main");
  private final Field layout;
  private final IntToLongFunction cameraSeeds;
  private final Noise noise;
  private final List<PhotonCameraSim> cameras = new ArrayList<>();

  /**
   * When this world began, on the robot's clock, in nanoseconds. The robot's true poses are kept by
   * the time since then ({@link #sinceStart}), not by the clock's own time.
   *
   * <p>The clock counts from wherever the computer's monotonic clock stood when the simulation
   * started. That is days of uptime, 10^14 ns and more, where a double of seconds steps by a tenth
   * of a nanosecond. Poses kept by the clock's own seconds would interpolate a little differently
   * from run to run, and an image's corners with them.
   */
  private final long startNanos = RobotController.getMonotonicTime();

  /** Where the robot really was, loop by loop, by seconds since the world began. */
  private final TimeInterpolatableBuffer<Pose3d> truths =
      TimeInterpolatableBuffer.createBuffer(HISTORY_SECONDS);

  /**
   * PhotonVision's simulated camera, which stops publishing when it is closed.
   *
   * <p>PhotonVision's own {@code close} lets go of its video and leaves its NetworkTables topics
   * published, the camera's results among them, so NetworkTables keeps the last image the camera
   * sent for as long as the process runs. A test run builds one robot after another in one process.
   * The next robot's camera of the same name then found that image waiting, and took it for its own
   * first one. It was stamped on the earlier robot's clock, and between tests the simulator's clock
   * runs on the computer's, so how long ago that seemed was different on every run.
   *
   * <p>With nothing publishing a topic any more, NetworkTables forgets its value, and the next
   * camera starts with none.
   *
   * <p>It also does not serve video. PhotonVision's camera starts two video servers, a raw stream
   * and a processed one, on the next free ports from 1181 up, listening on every address of the
   * computer. The streams are switched off here ({@link #camera}), so nobody could watch them, and
   * every simulated robot a test run built took two more ports, past three hundred in a run. The
   * servers are closed as soon as the camera is made.
   */
  private static final class OwnTopicsCameraSim extends PhotonCameraSim {
    OwnTopicsCameraSim(PhotonCamera camera, SimCameraProperties properties, Field layout) {
      super(camera, properties, layout);
      closeVideoServers(camera.getName());
    }

    @Override
    public void close() {
      super.close();
      ts.removeEntries();
    }
  }

  /**
   * Closes the two video servers PhotonVision's simulation of a camera starts, which are named for
   * the camera and its stream. {@link VisionSim} does so for the cameras it adds; a test that makes
   * a {@code PhotonCameraSim} of its own does it for that.
   */
  public static void closeVideoServers(String camera) {
    for (String stream : List.of("-raw", "-processed")) {
      String name = "serve_" + camera + stream;
      VideoSink server = CameraServer.getServer(name);
      CameraServer.removeServer(name);
      server.close();
    }
  }

  /**
   * A camera as added.
   *
   * @param simulated PhotonVision's simulation of it
   * @param configured where its configuration mounts it on the robot
   * @param latencyMillis its configured latency
   * @param mountError how far it is knocked from its configured mount, in the robot's frame
   * @param mount where it is mounted now, which its images are taken from
   */
  private record Added(
      PhotonCameraSim simulated,
      Transform3d configured,
      double latencyMillis,
      Transform3d mountError,
      Transform3d mount) {}

  /**
   * Where each camera on a joint was carried, by name, over the history kept, so that an image can
   * be taken from where the camera was at that moment. A place here stands in for the configured
   * mount: the camera's mount error is added to it.
   */
  private final Map<String, TimeInterpolatableBuffer<Pose3d>> carried = new LinkedHashMap<>();

  private final Map<String, Added> byName = new LinkedHashMap<>();

  /**
   * How far each camera's clock is off the robot's, by name, in nanoseconds. A camera that isn't
   * here has a clock that agrees with the robot's.
   */
  private final Map<String, Long> clockOffsets = new LinkedHashMap<>();

  /** The field's tags as the cameras see them, by ID, and where each was as the world was built. */
  private final Map<Integer, VisionTargetSim> tags = new LinkedHashMap<>();

  private final Map<Integer, Pose3d> builtTags = new LinkedHashMap<>();

  /** The ID of the tag that is moved from where it was built, if one is. */
  private OptionalInt movedTag = OptionalInt.empty();

  /**
   * @param tagErrors by tag ID, how far each tag really sits from where the field's layout says, in
   *     the field's frame, as the world is built. The cameras see the tag there, while their
   *     multi-tag solves believe the layout, as a coprocessor's do. ({@link #moveTag} moves one
   *     further.)
   * @param cameraSeeds where each camera's noise starts, by its place in the order the cameras are
   *     added, from 0. Each camera draws its own noise, so a run repeats exactly, and a run with
   *     another seed sees other noise. The simulator gives each camera its own stream of the
   *     world's seed ({@code Seeds.camera}).
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
    this.cameraSeeds = cameraSeeds;
    layout = field.toField();
    List<FieldTag> asBuilt = new ArrayList<>();
    for (FieldTag tag : layout.getTags()) {
      Translation3d off = tagErrors.getOrDefault(tag.getID(), Translation3d.ZERO);
      Pose3d inTheLayout = tag.getPose();
      Pose3d built = new Pose3d(inTheLayout.getTranslation().plus(off), inTheLayout.getRotation());
      asBuilt.add(new FieldTag(tag.getID(), built));
      builtTags.put(tag.getID(), built);
    }
    world.addAprilTags(fieldWith(asBuilt));
    for (VisionTargetSim target : world.getVisionTargets("apriltag")) {
      tags.put(target.fiducialID, target);
    }
  }

  /** The layout's field with its tags where they were built. WPILib's field takes no image here. */
  private Field fieldWith(List<FieldTag> tagsAsBuilt) {
    return new Field(
        layout.name,
        layout.season,
        layout.game,
        null,
        layout.length,
        layout.width,
        layout.program,
        tagsAsBuilt);
  }

  /** The field's tags' IDs, as its layout lists them. */
  public List<Integer> tagIds() {
    return List.copyOf(builtTags.keySet());
  }

  /**
   * Moves one tag this far from where it was as the world was built, in the field's frame, from now
   * on, as a tag knocked at an event is. Any other moved tag is put back first. The robot code, and
   * the cameras' multi-tag solves, still believe the layout.
   *
   * @param by how far to move it. Zero puts it back.
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
    boolean putBack = by.equals(Translation3d.ZERO);
    if (!putBack) {
      movedTag = OptionalInt.of(id);
    }
  }

  /** Puts every tag back where it was as the world was built. */
  public void putTagsBack() {
    for (Map.Entry<Integer, Pose3d> built : builtTags.entrySet()) {
      VisionTargetSim target = tags.get(built.getKey());
      if (target != null) {
        target.setPose(built.getValue());
      }
    }
    movedTag = OptionalInt.empty();
  }

  /** Whether a tag is moved from where it was as the world was built. */
  public boolean isTagMoved() {
    return movedTag.isPresent();
  }

  /** Where a tag really is now, as the cameras see it, or empty if the field has no such tag. */
  public Optional<Pose3d> tagPose(int id) {
    return Optional.ofNullable(tags.get(id)).map(VisionTargetSim::getPose);
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
        new OwnTopicsCameraSim(
            io.camera(),
            properties(config.sensor(), cameraSeeds.applyAsLong(cameras.size()), noise),
            layout);
    // Tags farther than the camera's sight range are not detected. They are small in the image,
    // and noisy.
    simulated.setMaxSightRange(config.sensor().sightRange().in(Meters));
    // No video. By default PhotonVision renders a raw and a processed frame of every image, in
    // native memory that is freed only when the collector gets to it (one study's process grew past
    // 4 GB). Nothing here views them, and what the camera detects doesn't depend on them.
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
   * Transform3d)} does when it's added. Images rendered from now on see from there, and that
   * includes an image already taken and still on its way.
   *
   * @param mountError in the robot's frame, from the configured mount. Zero puts it back.
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
   * Where a camera on a joint is now, in the robot's frame, as the joint has carried it. Images
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
   * Makes a camera this much later than its configuration says, from now on. Its images arrive
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
   * Sets a camera's clock this far off the robot's, from now on. Each image is stamped this much
   * later than it was taken (earlier, if negative), as a coprocessor whose time sync is off stamps
   * it, though it arrives when it would have. Unlike {@link #addLatency}, the stamps are wrong.
   *
   * @param millis zero for a clock that agrees with the robot's
   * @throws IllegalArgumentException if no camera has this name
   */
  public void offsetClock(String camera, double millis) {
    added(camera);
    clockOffsets.put(camera, Math.round(millis * NANOS_PER_MILLI));
  }

  private Added added(String camera) {
    Added added = byName.get(camera);
    if (added == null) {
      throw new IllegalArgumentException("no simulated camera is named " + camera);
    }
    return added;
  }

  /**
   * Where a camera really is, mounted off its configuration. The error's translation is added to
   * the configured position, and its rotation turns the configured orientation. The camera
   * placement study mounts its cameras off the same way.
   */
  public static Transform3d mounted(Transform3d configured, Transform3d mountError) {
    return new Transform3d(
        configured.getTranslation().plus(mountError.getTranslation()),
        configured.getRotation().rotateBy(mountError.getRotation()));
  }

  /**
   * A camera as PhotonVision's simulation models it. The model has the sensor's resolution, field
   * of view, frame rate, and latency, and a little lens-calibration error, which moves each tag
   * corner about a quarter pixel. The camera placement study takes its images with the same model,
   * and starts from the same errors.
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
   * <p>An image falls due on the camera's frame rate. It is taken its latency before then, from
   * where the robot was at that moment, and it is stamped with that moment, as a real camera's time
   * sync stamps it.
   *
   * <p>PhotonVision's own {@code VisionSystemSim.update} takes an image the same way but stamps it
   * as of the loop that renders it, up to a loop (20 ms) later. While the robot turns, every image
   * then shows it where it was a little before its stamp, turned a little less, the same way image
   * after image. That is just what a heading error does to a position solved with the gyro's
   * heading. An estimator that lets every fix correct the heading took it for one, and grew sure of
   * a heading 0.7° off after the gyro-scale procedure (docs/testing.md). So the images are taken
   * and stamped here instead.
   */
  public void update(Pose3d truth) {
    truths.addSample(sinceStart(RobotController.getMonotonicTime()), truth);
    List<VisionTargetSim> targets = List.copyOf(world.getVisionTargets());
    for (Map.Entry<String, Added> each : byName.entrySet()) {
      String name = each.getKey();
      Added added = each.getValue();
      Optional<Long> dueNanos = added.simulated().consumeNextEntryTime();
      if (dueNanos.isPresent()) {
        takeImage(name, added, dueNanos.get(), truth, targets);
      }
    }
    // Every image that was due has been taken above, so PhotonVision's own update finds none to
    // take. It only keeps its record of where the robot is.
    world.update(truth);
  }

  /**
   * Takes one camera's image and hands it to the robot code.
   *
   * @param arrivedNanos when the image reaches the robot code
   * @param truthNow where the robot really is this loop, which stands in when the image was taken
   *     longer ago than the history kept
   */
  private void takeImage(
      String name, Added added, long arrivedNanos, Pose3d truthNow, List<VisionTargetSim> targets) {
    PhotonCameraSim camera = added.simulated();
    // The camera draws a latency for this image, which varies a little from image to image.
    double latencyMillis = camera.prop.estLatencyMs();
    long takenNanos = arrivedNanos - Math.round(latencyMillis * NANOS_PER_MILLI);
    Pose3d robotThen = truths.getSample(sinceStart(takenNanos)).orElse(truthNow);
    Pose3d cameraThen = robotThen.plus(mountAt(name, added, takenNanos));
    PhotonPipelineResult image = camera.process(latencyMillis, cameraThen, targets);
    // A camera whose clock is off the robot's stamps its image as its own clock had it.
    image.metadata.captureTimestampNanos = takenNanos + clockOffsets.getOrDefault(name, 0L);
    image.metadata.publishTimestampNanos = arrivedNanos;
    camera.submitProcessedFrame(image, arrivedNanos);
  }

  /**
   * Where a camera was on the robot when an image was taken. A camera on a joint was where its
   * joint had carried it. A moment before the joint's history starts falls back to the mount the
   * camera was added with.
   */
  private Transform3d mountAt(String camera, Added added, long takenNanos) {
    TimeInterpolatableBuffer<Pose3d> carriedByItsJoint = carried.get(camera);
    if (carriedByItsJoint == null) {
      return added.mount();
    }
    Optional<Pose3d> carriedTo = carriedByItsJoint.getSample(sinceStart(takenNanos));
    if (carriedTo.isEmpty()) {
      return added.mount();
    }
    Transform3d configuredThen =
        new Transform3d(carriedTo.get().getTranslation(), carriedTo.get().getRotation());
    return mounted(configuredThen, added.mountError());
  }

  /** How long after this world began a moment on the robot's clock was, in seconds. */
  private double sinceStart(long nanos) {
    return (nanos - startNanos) / NANOS_PER_SECOND;
  }

  @Override
  public void close() {
    world.clearCameras();
    for (PhotonCameraSim camera : cameras) {
      camera.close();
    }
  }
}
