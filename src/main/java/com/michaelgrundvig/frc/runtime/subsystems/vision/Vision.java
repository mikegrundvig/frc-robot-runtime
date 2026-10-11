package com.michaelgrundvig.frc.runtime.subsystems.vision;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;

import com.michaelgrundvig.frc.localization.CaptureContext;
import com.michaelgrundvig.frc.localization.FrameVerdict;
import com.michaelgrundvig.frc.localization.LocalizationEstimate;
import com.michaelgrundvig.frc.localization.LocalizationService;
import com.michaelgrundvig.frc.localization.MountCheck;
import com.michaelgrundvig.frc.localization.TagAllowlist;
import com.michaelgrundvig.frc.localization.TagDetections;
import com.michaelgrundvig.frc.localization.Timestamps;
import com.michaelgrundvig.frc.localization.Verdict;
import com.michaelgrundvig.frc.localization.VisionConfig;
import com.michaelgrundvig.frc.localization.VisionMeasurement;
import com.michaelgrundvig.frc.localization.VisionPipeline;
import com.michaelgrundvig.frc.localization.VisionResult;
import com.michaelgrundvig.frc.localization.calibration.CameraMountCalibration;
import com.michaelgrundvig.frc.localization.calibration.HeadingAverage;
import com.michaelgrundvig.frc.pitwall.Limits;
import com.michaelgrundvig.frc.pitwall.LiveConfig;
import com.michaelgrundvig.frc.world.field.Alliance;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Place;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalLong;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.tunable.TunableBase;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.TunableRegistry;
import org.wpilib.util.Alert;

/**
 * The robot's cameras. Each loop this reads every camera's results and hands each image to
 * localization ({@link LocalizationService#camera}), which solves it through the {@link
 * VisionPipeline} as the robot was when the image was taken.
 *
 * <p>The order in a loop matters. Call {@link #periodic} after the drivetrain's odometry, because
 * an image is solved with the robot's heading and tilt at the moment it was taken, and odometry is
 * what knows them. Call it before localization's update, because the update is what applies the
 * loop's images. Then call {@link #record} with what the update made of each image.
 *
 * <p>For the pit and the log, each camera says whether it's connected, what became of every image
 * and why, and where it puts each tag it sees (placed through the estimated pose and the camera's
 * configured mount) with how far that lands from the tag's real place. A camera whose tags land
 * consistently off is mounted differently than its configuration says, because it was bumped at an
 * event, say. Past a tolerance it raises an alert to recalibrate it, and until then its
 * measurements count for less, by how far its tags land off across the floor (see {@link
 * MountCheck}).
 *
 * <p>Each image's latency, from when it was taken to the loop that reads it, is logged too. An
 * image that arrives before it was taken, or much later than its camera's usual latency, has the
 * wrong capture time, which happens when the camera's clock and the robot's are out of step. It is
 * left out and counted, and an alert stays up for two seconds after the last one.
 *
 * <p>How far images are trusted is the robot's configuration ({@code RobotConfig.vision}), whose
 * main thresholds and uncertainties Pitwall tunes. Two switches are WPILib tunables, for the drive
 * team as much as the pit. One per camera leaves that camera out (a camera fogged over, knocked
 * loose, or blinded by stage lights). The other says how long to hold the cameras back when
 * autonomous starts.
 *
 * <p>A task lining up on a target can trust only that target's tags while it runs ({@link
 * #trustingOnly}), as one top team's auto-align did.
 */
public final class Vision implements AutoCloseable {
  /** The share of measurements accepted is shown to this many percent. */
  private static final double ACCEPTED_STEP_PERCENT = 5;

  /** The robot's pose is good enough to calibrate a camera against when it is within 5 cm. */
  private static final double CALIBRATION_POSE_STD_DEV_METERS = 0.05;

  /**
   * A camera's mount is judged by the latest 500 tags it saw. That is eight seconds of seeing two
   * tags in every image at 30 images a second, and enough for noise and the estimate's own errors
   * to average out. A knock shows in proportion: one that puts tags 10 cm off is past a 5 cm
   * tolerance after 250 tags.
   */
  private static final int MOUNT_SIGHTINGS = 500;

  /**
   * A camera's mount is judged only while another camera's measurement agreed with the estimate
   * this lately. The other cameras hold the pose it is judged against, and odometry carries the
   * pose between their measurements. It is two seconds because cameras facing other ways see tags
   * at other moments, and because a camera's partner facing the same way can be pulled off with it.
   */
  private static final long OTHERS_HOLD_NANOS = 2_000_000_000L;

  /**
   * A camera is calibrated only from images taken after the robot has stood still this long. Such
   * an image has no blur, and it doesn't matter exactly when it was taken.
   */
  private static final long STILL_NANOS = 500_000_000L;

  /** Slower than this, the robot counts as standing still: 2 cm and about a degree a second. */
  private static final double STILL_METERS_PER_SECOND = 0.02;

  private static final double STILL_RADIANS_PER_SECOND = 0.02;

  /** The longest the cameras can be held back at autonomous's start: longer than any autonomous. */
  private static final double MAX_HOLD_SECONDS = 30;

  private final List<Camera> cameras;
  private final LocalizationService localization;

  /** The field whose tags the cameras see. A venue selected in the pit replaces it. */
  private FieldModel field;

  /** How far images are trusted, as tuned, and the value read from it this loop. */
  private final Supplier<VisionConfig> settings;

  private VisionConfig current;

  /** The robot's alliance, once known. A task lining up on a place trusts that alliance's tags. */
  private final Supplier<Optional<Alliance>> alliance;

  /** The time of the last loop in which the robot was seen moving, or empty if it never was. */
  private OptionalLong lastMovedNanos = OptionalLong.empty();

  /** When the previous loop read the cameras, which is what each image read now arrived after. */
  private OptionalLong previousLoopNanos = OptionalLong.empty();

  // ---- Calibrations that collect from the cameras ----

  /** Where the headings the cameras measure go, while a calibration collects them. */
  private Optional<HeadingAverage> headings = Optional.empty();

  /** The camera whose mount is being calibrated, and where its sightings go, while one is. */
  private Optional<MountCalibration> mountCalibration = Optional.empty();

  /**
   * One camera's mount being calibrated.
   *
   * @param camera the camera, whose measurements stay out of the estimator meanwhile
   * @param sightings the calibration its tag sightings go into
   */
  private record MountCalibration(Camera camera, CameraMountCalibration sightings) {}

  // ---- Holding the cameras back as autonomous starts ----

  /**
   * How long to hold the cameras' measurements back when autonomous starts, trusting the placed
   * start while the robot launches. One top team held theirs for 2 s. It is off by default, because
   * the estimator gates every measurement against the placed start anyway.
   */
  private final TunableDouble autoStartHoldSeconds =
      TunableDouble.createConfig(0.0, Limits.between(0, MAX_HOLD_SECONDS).config());

  /** Images taken before this time are held back. It is empty while nothing is held. */
  private OptionalLong heldUntilNanos = OptionalLong.empty();

  private final List<String> tunablePaths = new ArrayList<>();

  // ---- Trusting only some tags ----

  /** The tasks holding the cameras to their tags, oldest first. The latest one is in effect. */
  private final List<TagLease> leases = new ArrayList<>();

  private final Alert trustingFewerTags =
      new Alert("Vision", "trustingFewerTags", "The cameras trust only some tags", Alert.Level.LOW);

  // ---- The pit's line ----

  /** The share of measurements accepted as last shown, or empty before there was one. */
  private OptionalDouble shownAcceptedPercent = OptionalDouble.empty();

  /**
   * Vision's configuration as the pit tunes it: the main thresholds and uncertainties, each within
   * the range that's safe to dial in. Publish it, and give vision its {@link LiveConfig#get}.
   */
  public static LiveConfig<VisionConfig> tuning(VisionConfig committed) {
    return LiveConfig.of("vision", committed)
        .number("maxAmbiguity", Limits.between(0, 1))
        .number("maxHeightErrorMeters", Limits.between(0, 1))
        .number("linearStdDevBaseline", Limits.between(0.001, 1)) // meters
        .number("angularStdDevBaseline", Limits.between(0.001, 1)) // radians
        .number("distanceExponent", Limits.between(0, 4))
        .number("mountToleranceMeters", Limits.between(0.01, 0.5))
        .number("latencyMarginSeconds", Limits.between(0.01, 1));
  }

  /**
   * @param cameras each camera's configuration, in the same order as its IO
   * @param ios each camera's door to its vision software
   * @param localization what solves each image, as the robot was when it was taken, and weighs it
   * @param field the field, whose tags the cameras see
   * @param settings how far images are trusted, as tuned, which is read once a loop
   * @param alliance the robot's alliance, once the Driver Station or the field says
   */
  public Vision(
      List<CameraConfig> cameras,
      CameraIO[] ios,
      LocalizationService localization,
      FieldModel field,
      Supplier<VisionConfig> settings,
      Supplier<Optional<Alliance>> alliance) {
    if (cameras.size() != ios.length) {
      throw new IllegalArgumentException("one IO for each camera");
    }
    List<Camera> each = new ArrayList<>();
    for (int i = 0; i < ios.length; i++) {
      each.add(new Camera(cameras.get(i), ios[i], MOUNT_SIGHTINGS));
    }
    this.cameras = List.copyOf(each);
    this.localization = localization;
    this.field = field;
    this.settings = settings;
    this.alliance = alliance;
    current = settings.get();
    publish("Vision/autoStartHoldSeconds", autoStartHoldSeconds);
  }

  private void publish(String path, TunableBase tunable) {
    TunableRegistry.publish(path, tunable);
    tunablePaths.add(path);
  }

  // ---- Calibrations that collect from the cameras ----

  /**
   * From now on, every heading the cameras measure goes into this average, until stopped. It is a
   * heading reference for calibrating the gyro. Only a measurement that has a heading of its own
   * counts, which is a multi-tag solution, and the procedure takes them standing still.
   */
  public void collectHeadings(HeadingAverage into) {
    headings = Optional.of(into);
  }

  public void stopCollectingHeadings() {
    headings = Optional.empty();
  }

  /**
   * From now on, a camera's tag sightings go into this calibration of its mount, until stopped.
   * They are taken while the robot stands still and the other cameras hold its pose within 5 cm.
   * The camera's own measurements stay out of the estimator meanwhile, so that the pose it is
   * calibrated against comes from the others.
   */
  public void calibrateMount(String camera, CameraMountCalibration into) {
    mountCalibration = Optional.of(new MountCalibration(camera(camera), into));
  }

  public void stopCalibratingMount() {
    mountCalibration = Optional.empty();
  }

  /** Whether a calibration is collecting from the cameras: headings, or one camera's mount. */
  public boolean isCalibrating() {
    return mountCalibration.isPresent() || headings.isPresent();
  }

  private boolean isBeingCalibrated(Camera camera) {
    return mountCalibration.isPresent() && mountCalibration.get().camera() == camera;
  }

  // ---- Trusting only some tags ----

  /**
   * A task lining up on a place trusts only the tags the place is found from, while it runs. Lining
   * up on the hub, the cameras measure the robot from the hub's tags alone.
   *
   * <p>Which tags those are depends on the alliance, and a binding is made before the alliance is
   * known, so they are found when the task starts. With no alliance known by then, the task trusts
   * every tag, and an alert says so.
   *
   * @throws IllegalArgumentException if the place is written as a pose, which has no tags to trust
   */
  public Command trustingOnly(Place place, Command task) {
    if (place.tags(field, Alliance.BLUE).isEmpty()) {
      throw new IllegalArgumentException(
          place.name() + " is written as a pose, not found from tags: it has no tags to trust");
    }
    return trusting(
        task,
        () -> leaseOn(place),
        task.name() + " (trusting " + possessive(place.name()) + " tags)");
  }

  /** Holds the cameras to a place's tags for the robot's alliance, as a task starts. */
  private TagLease leaseOn(Place place) {
    Optional<Alliance> ours = alliance.get();
    if (ours.isEmpty()) {
      return hold(
          Optional.empty(), place.name() + ": the alliance isn't known, so every tag is trusted");
    }
    return trustOnly(TagAllowlist.of(place.name(), place.tags(field, ours.get())));
  }

  /**
   * A task that trusts only some tags while it runs. The cameras go back to what they trusted
   * before when it ends, however it ends.
   */
  public Command trustingOnly(TagAllowlist tags, Command task) {
    return trusting(
        task, () -> trustOnly(tags), task.name() + " (trusting " + tags.describe() + ")");
  }

  /**
   * The task, holding the cameras to its tags while it runs.
   *
   * <p>It requires what the task requires, at the task's priority. So a task that interrupts it
   * ends it before starting, which means the one ending lets go of its tags before the one starting
   * takes its own. A task that is refused never touches the tags.
   */
  private Command trusting(Command task, Supplier<TagLease> take, String name) {
    HeldLease held = new HeldLease();
    return Command.requiring(task.requirements())
        .executing(
            coroutine -> {
              held.lease = Optional.of(take.get());
              coroutine.await(task);
              held.release();
            })
        // A canceled command never resumes, so nothing after the await runs (not even a finally).
        .whenCanceled(held::release)
        .withPriority(task.priority())
        .named(name);
  }

  /** One run's hold on the cameras' tags, from the task's start until it ends or is canceled. */
  private static final class HeldLease {
    private Optional<TagLease> lease = Optional.empty();

    void release() {
      lease.ifPresent(TagLease::release);
      lease = Optional.empty();
    }
  }

  /**
   * From now on, the cameras measure the robot only from these tags, until the lease is released.
   * Every tag is still logged, and still checks its camera's mount.
   *
   * <p>{@link #trustingOnly} takes and releases a lease around a task, which is safer than doing it
   * by hand: a task canceled partway never reaches its own last line.
   */
  public TagLease trustOnly(TagAllowlist tags) {
    return hold(Optional.of(tags), "The cameras trust only " + tags.describe());
  }

  private TagLease hold(Optional<TagAllowlist> tags, String says) {
    TagLease lease = new TagLease(tags, says);
    leases.add(lease);
    return lease;
  }

  /**
   * One task's hold on which tags the cameras trust. The latest lease still held is in effect.
   * Releasing one gives the cameras back to the one before, or to every tag.
   */
  public final class TagLease {
    /** The tags trusted, or empty for a lease that trusts every tag. */
    private final Optional<TagAllowlist> tags;

    /** What the alert says while this lease is in effect. */
    private final String says;

    private TagLease(Optional<TagAllowlist> tags, String says) {
      this.tags = tags;
      this.says = says;
    }

    /**
     * Lets go. Releasing it again, or after a later task took the cameras, changes nothing more.
     */
    public void release() {
      leases.remove(this);
    }
  }

  /** What the alert about trusting fewer tags says, or nothing while it's down. */
  String trustingFewerTagsAlertText() {
    if (!trustingFewerTags.get()) {
      return "";
    }
    return trustingFewerTags.getText();
  }

  /** The tags the cameras trust now, or empty when they trust every tag. */
  private Optional<TagAllowlist> trustedTags() {
    if (leases.isEmpty()) {
      return Optional.empty();
    }
    return leases.getLast().tags;
  }

  /** Logs which tags are trusted, and keeps the alert up while a task holds the cameras to some. */
  private void showWhichTagsAreTrusted() {
    Logger.recordOutput("Vision/OnlyTags", trustedTags().map(TagAllowlist::describe).orElse(""));
    boolean held = !leases.isEmpty();
    if (held) {
      trustingFewerTags.setText(leases.getLast().says);
    }
    trustingFewerTags.set(held);
  }

  // ---- Holding the cameras back as autonomous starts ----

  /**
   * Autonomous just started: hold the cameras' measurements back for the tuned time, if any. They
   * are still read and logged, marked as held.
   */
  public void autonomousStarted(long nowNanos) {
    // A tuned value that isn't a sensible time holds nothing, and never throws in the loop.
    double seconds = autoStartHoldSeconds.get();
    boolean sensible = seconds > 0 && Double.isFinite(seconds);
    if (!sensible) {
      heldUntilNanos = OptionalLong.empty();
      return;
    }
    long holdNanos = Timestamps.fromSeconds(Math.min(seconds, MAX_HOLD_SECONDS));
    heldUntilNanos = OptionalLong.of(nowNanos + holdNanos);
  }

  /**
   * Why an image is solved and logged but never applied, or empty when it is applied. A camera
   * being calibrated is judged against the pose the others hold, so its own images stay out. An
   * image taken just after autonomous started is held while the placed start is trusted.
   */
  private Optional<Verdict> whyHeldBack(Camera camera, long takenNanos) {
    if (isBeingCalibrated(camera)) {
      return Optional.of(Verdict.CALIBRATING);
    }
    if (heldUntilNanos.isPresent() && takenNanos < heldUntilNanos.getAsLong()) {
      return Optional.of(Verdict.HELD);
    }
    return Optional.empty();
  }

  // ---- One loop ----

  /**
   * Reads every camera and hands localization each usable image.
   *
   * @param nowNanos the robot's clock this loop, which how late each image arrived is measured to
   */
  public void periodic(long nowNanos) {
    LocalizationEstimate estimate = localization.estimate();
    noteWhetherMoving(estimate);
    current = settings.get();
    localization.vision(current);
    showWhichTagsAreTrusted();
    PoseKnowledge known = howWellThePoseIsKnown(estimate);
    for (Camera camera : cameras) {
      takeItsImages(camera, known, nowNanos);
    }
    previousLoopNanos = OptionalLong.of(nowNanos);
  }

  private void noteWhetherMoving(LocalizationEstimate estimate) {
    boolean moving =
        estimate.fieldVelocity().getNorm() > STILL_METERS_PER_SECOND
            || Math.abs(estimate.yawRateRadiansPerSecond()) > STILL_RADIANS_PER_SECOND;
    if (moving) {
      lastMovedNanos = OptionalLong.of(estimate.timestampNanos());
    }
  }

  /**
   * Whether the robot had stood still for long enough when an image was taken, counted from the
   * last loop it was seen moving. A robot that hasn't been seen moving since the program started
   * has no such loop to count from, and doesn't count as standing still.
   */
  private boolean hadStoodStillAt(long takenNanos) {
    return lastMovedNanos.isPresent() && takenNanos - lastMovedNanos.getAsLong() >= STILL_NANOS;
  }

  /**
   * How well the robot's pose is known this loop, which is what each camera's mount is judged
   * against.
   *
   * @param positionStdDevMeters how sure the estimate's position is, the least sure way
   * @param headingStdDevRadians how sure the estimate's heading is
   * @param tracking whether recent camera measurements agree with odometry
   * @param goodEnoughToCalibrateBy whether the pose is held well enough to calibrate a camera's
   *     mount against
   */
  private record PoseKnowledge(
      double positionStdDevMeters,
      double headingStdDevRadians,
      boolean tracking,
      boolean goodEnoughToCalibrateBy) {}

  private static PoseKnowledge howWellThePoseIsKnown(LocalizationEstimate estimate) {
    boolean tracking = estimate.mode() == LocalizationEstimate.Mode.TRACKING;
    double positionStdDevMeters = estimate.covariance().largestPositionStdDev();
    return new PoseKnowledge(
        positionStdDevMeters,
        estimate.covariance().headingStdDev(),
        tracking,
        tracking && positionStdDevMeters <= CALIBRATION_POSE_STD_DEV_METERS);
  }

  /**
   * Reads one camera and takes each of its images in turn.
   *
   * <p>The camera's mount is judged before its images are taken, so that this loop's measurements
   * are doubted by what its earlier tags showed, and localization is told that doubt first.
   */
  private void takeItsImages(Camera camera, PoseKnowledge known, long nowNanos) {
    camera.read();
    camera.checkMount(current.mountToleranceMeters());
    localization.mountDoubt(camera.name(), camera.mountDoubtMeters(), camera.mountDistanceMeters());
    // The IO logs every image's tags in one flat list, each image's following the one before's.
    int nextTag = 0;
    for (CameraIO.Frame frame : camera.inputs.frames) {
      int firstTag = nextTag;
      nextTag += frame.tagCount();
      // One image's bad data costs that image, never the robot program. Whatever a camera sends
      // that the checks miss ends here, logged as the image's outcome.
      try {
        takeImage(camera, frame, firstTag, known, nowNanos);
      } catch (RuntimeException thrown) {
        camera.outcome("unusable data from the camera: " + thrown.getMessage());
      }
    }
    camera.logLoop(nowNanos);
  }

  // ---- One image ----

  /**
   * One image. Unless it is left out, it is solved into a measurement for localization, and its
   * tags are placed on the field for the camera's residual, the check of its mount, and any
   * calibration that is collecting them.
   */
  private void takeImage(
      Camera camera, CameraIO.Frame frame, int firstTag, PoseKnowledge known, long nowNanos) {
    long takenNanos = frame.timestampNanos();
    TagDetections detections =
        Detections.inImage(frame, camera.inputs.tags, firstTag, camera.name());
    List<Pose3d> tagsOnTheField = onTheField(detections);
    if (!camera.enabled.get()) {
      camera.leftOut(tagsOnTheField, "camera turned off");
      return;
    }
    if (!camera.isNew(takenNanos)) {
      camera.outcome("the same image again");
      return;
    }
    // An image that arrived before it was taken, or far later than its camera's images do, has
    // the wrong capture time. Solved as of that time, it would put the robot where it wasn't.
    boolean inTime =
        camera.arrivedInTime(
            takenNanos,
            nowNanos,
            previousLoopNanos,
            current.latencyMarginSeconds(),
            tagsOnTheField);
    if (!inTime) {
      return;
    }
    Optional<Pose2d> poseThen = localization.poseAt(takenNanos);
    CaptureContext capture = localization.capture(takenNanos);
    Optional<VisionMeasurement> measurement =
        handToLocalization(camera, detections, tagsOnTheField);
    collectItsHeading(measurement);
    // A camera on a joint is neither judged by its mount nor calibrated. Where it is is its
    // joint's to say. And without the robot's pose at the time, its tags can't be placed.
    boolean rigidlyMounted = camera.config.joint().isEmpty();
    if (poseThen.isEmpty() || !rigidlyMounted) {
      return;
    }
    placeItsTags(camera, detections, robotAsItWas(poseThen.get(), capture), capture, known);
  }

  /**
   * Hands an image to localization, which solves it, and records what became of it. An image that
   * is held back is solved and logged all the same, and never applied.
   *
   * @return the measurement the image was solved into, or empty if the pipeline refused it
   */
  private Optional<VisionMeasurement> handToLocalization(
      Camera camera, TagDetections detections, List<Pose3d> tagsOnTheField) {
    // Localization takes null for "every tag".
    @Nullable TagAllowlist onlyTags = trustedTags().orElse(null);
    Optional<Verdict> heldBack = whyHeldBack(camera, detections.timestampNanos());
    VisionResult result;
    if (heldBack.isPresent()) {
      result = localization.hold(detections, onlyTags, heldBack.get());
    } else {
      result = localization.camera(detections, onlyTags);
    }
    Optional<VisionMeasurement> measurement = Optional.ofNullable(result.measurement());
    if (measurement.isEmpty()) {
      camera.leftOut(tagsOnTheField, result.outcome());
    } else if (heldBack.isPresent()) {
      camera.leftOut(tagsOnTheField, heldBack.get().words() + ": " + result.outcome());
    } else {
      camera.outcome(result.outcome());
    }
    return measurement;
  }

  /** Gives a measurement's heading to the calibration collecting headings, if one is. */
  private void collectItsHeading(Optional<VisionMeasurement> measurement) {
    if (headings.isEmpty() || measurement.isEmpty()) {
      return;
    }
    if (measurement.get().covariance().hasHeading()) {
      headings.get().add(measurement.get().pose().getRotation());
    }
  }

  /**
   * The robot as it was when an image was taken, in three dimensions. On raised terrain the robot
   * stands as high as the ground under it, and it is tilted as its IMU said then.
   */
  private Pose3d robotAsItWas(Pose2d poseThen, CaptureContext capture) {
    return new Pose3d(
        poseThen.getX(),
        poseThen.getY(),
        field.terrain().heightAt(poseThen.getTranslation()),
        new Rotation3d(
            capture.rollRadians(), capture.pitchRadians(), poseThen.getRotation().getRadians()));
  }

  /**
   * Places each tag an image saw on the field, through where the robot was and the camera's
   * configured mount. Each goes into the camera's residual, into the check of its mount when the
   * pose is known well enough for that, and into the mount calibration collecting this camera's
   * sightings, if one is.
   */
  private void placeItsTags(
      Camera camera,
      TagDetections detections,
      Pose3d robotThen,
      CaptureContext capture,
      PoseKnowledge known) {
    long takenNanos = detections.timestampNanos();
    Optional<CameraMountCalibration> calibrating = Optional.empty();
    if (isBeingCalibrated(camera)
        && known.goodEnoughToCalibrateBy()
        && hadStoodStillAt(takenNanos)) {
      calibrating = Optional.of(mountCalibration.get().sightings());
    }
    boolean poseIsHeld = othersHoldThePose(camera, capture, known, takenNanos);
    for (TagDetections.Tag tag : detections.tags()) {
      Pose3d seen =
          robotThen.transformBy(camera.config.robotToCamera()).transformBy(tag.bestCameraToTag());
      camera.saw(seen);
      Optional<Pose3d> real = field.tag(tag.id());
      if (real.isEmpty()) {
        continue;
      }
      camera.landed(seen, real.get());
      double distanceMeters = tag.bestCameraToTag().getTranslation().getNorm();
      if (poseIsHeld && isSureEnoughAt(distanceMeters, known)) {
        camera.judgeMountBy(robotThen, seen, real.get());
      }
      if (calibrating.isPresent()) {
        calibrating.get().add(robotThen, tag.bestCameraToTag(), tag.altCameraToTag(), real.get());
      }
    }
  }

  /**
   * Whether a camera's mount can be judged against the pose now, because the other cameras hold it.
   * That takes all of: the estimate tracking, the robot turning slowly, no calibration under way,
   * and another camera's measurement accepted lately.
   *
   * <p>The last one matters after the heading has been disturbed (by the gyro-scale procedure's
   * spins, say). Then the estimate can stand still centimeters off while it's sure to millimeters.
   * The cameras, agreeing with each other but not with it, are what show that.
   */
  private boolean othersHoldThePose(
      Camera camera, CaptureContext capture, PoseKnowledge known, long takenNanos) {
    boolean turningSlowly =
        capture.recentYawRateRadiansPerSecond() <= current.maxYawRateForHeading();
    return known.tracking()
        && turningSlowly
        && !isCalibrating()
        && anotherAgreedLately(camera, takenNanos);
  }

  /**
   * Whether the estimator accepted another camera's measurement within two seconds before a moment
   * (or since), one that agreed with the estimate within the mount tolerance.
   */
  private boolean anotherAgreedLately(Camera camera, long takenNanos) {
    for (Camera other : cameras) {
      if (other != camera && other.agreedSince(takenNanos - OTHERS_HOLD_NANOS)) {
        return true;
      }
    }
    return false;
  }

  /**
   * Whether the pose is sure enough to judge a mount by a tag at this distance.
   *
   * <p>A heading off by an angle puts every tag that angle times its distance off, the same way
   * each time, which is just what a camera turned on its mount looks like. So where the estimate
   * puts a tag is uncertain by its position's spread and by that, which add as independent errors
   * do:
   *
   * <pre>spread at the tag = sqrt(position spread² + (distance × heading spread)²)</pre>
   *
   * <p>The tag counts only while that is within the mount tolerance.
   */
  private boolean isSureEnoughAt(double distanceMeters, PoseKnowledge known) {
    double spreadAtTheTagMeters =
        Math.hypot(known.positionStdDevMeters(), distanceMeters * known.headingStdDevRadians());
    return spreadAtTheTagMeters <= current.mountToleranceMeters();
  }

  /**
   * An image's tags where the field's layout has them. A tag the field doesn't have is left out.
   */
  private List<Pose3d> onTheField(TagDetections detections) {
    List<Pose3d> tags = new ArrayList<>(detections.tags().size());
    for (TagDetections.Tag tag : detections.tags()) {
      field.tag(tag.id()).ifPresent(tags::add);
    }
    return tags;
  }

  // ---- After localization's update ----

  /** Counts and logs what localization's update made of each camera's measurements. */
  public void record(List<FrameVerdict> frames) {
    for (Camera camera : cameras) {
      camera.record(frames, current.mountToleranceMeters(), field::tag);
    }
  }

  /**
   * The venue the pit selected, its tags where a survey found them. Mount checks judge against it
   * from now on. Only while disabled, as {@code RobotContainer} calls it.
   */
  public void venue(FieldModel venue) {
    field = venue;
  }

  // ---- For the pit ----

  /**
   * One line for the pit: which cameras are connected, how many of their measurements are accepted,
   * to 5%, and how many images were left out because they arrived out of step.
   *
   * <p>The images left out are counted exactly, since each is news, and the page hears of the line
   * at most once a second.
   */
  public String summary() {
    int connected = 0;
    int accepts = 0;
    int rejects = 0;
    int outOfStep = 0;
    for (Camera camera : cameras) {
      if (camera.isConnected()) {
        connected++;
      }
      accepts += camera.accepted();
      rejects += camera.rejected();
      outOfStep += camera.leftOutOfStep();
    }
    String line =
        connected + " of " + cameras.size() + " cameras connected, " + accepted(accepts, rejects);
    if (outOfStep > 0) {
      line += ", " + outOfStep + " images left out of step";
    }
    return line;
  }

  /**
   * The share of measurements accepted, in words. The share shown moves only once the real one has
   * moved 5% from it, so the line doesn't change with every measurement, or flicker at a rounding
   * edge.
   */
  private String accepted(int accepts, int rejects) {
    int measurements = accepts + rejects;
    if (measurements == 0) {
      return "no measurements yet";
    }
    double percent = 100.0 * accepts / measurements;
    boolean movedAStep =
        shownAcceptedPercent.isEmpty()
            || Math.abs(percent - shownAcceptedPercent.getAsDouble()) >= ACCEPTED_STEP_PERCENT;
    if (movedAStep) {
      double toTheStep = Math.round(percent / ACCEPTED_STEP_PERCENT) * ACCEPTED_STEP_PERCENT;
      shownAcceptedPercent = OptionalDouble.of(toTheStep);
    }
    return String.format(
        Locale.ROOT, "%.0f%% of measurements accepted", shownAcceptedPercent.getAsDouble());
  }

  /**
   * How far, on average since the robot started, a camera's tags land from where they really are,
   * placed through the estimated pose and the camera's configured mount. It is the pre-event check
   * that each camera is still mounted as configured. A few centimeters is normal, and one camera
   * well above the others has moved.
   *
   * @return NaN until the camera has seen a tag
   */
  public double meanTagResidualMeters(String camera) {
    return camera(camera).meanTagResidualMeters();
  }

  /**
   * How far off, on average and lately, a camera's tags land: the latest 500 it saw while the
   * robot's pose was known. Past the tolerance ({@code VisionConfig.mountToleranceMeters}), the
   * camera is out of calibration.
   *
   * @return NaN until the camera has seen 500 tags
   */
  public double mountOffsetMeters(String camera) {
    return camera(camera).mountOffsetMeters();
  }

  /**
   * How much less a camera is trusted for its mount: the uncertainty, in meters, added to each of
   * its measurements. Zero for a camera whose tags land within the tolerance.
   */
  public double mountDoubtMeters(String camera) {
    return camera(camera).mountDoubtMeters();
  }

  private Camera camera(String name) {
    for (Camera camera : cameras) {
      if (camera.name().equals(name)) {
        return camera;
      }
    }
    throw new IllegalArgumentException("no camera named " + name);
  }

  /** How long autonomous's start holds the cameras back, in seconds. Zero is off. */
  TunableDouble autoStartHoldSeconds() {
    return autoStartHoldSeconds;
  }

  /** A camera, in the order of the cameras' configuration: its switch, alerts, and tallies. */
  Camera camera(int index) {
    return cameras.get(index);
  }

  @Override
  public void close() {
    for (Camera camera : cameras) {
      camera.close();
    }
    for (String path : tunablePaths) {
      TunableRegistry.remove(path);
    }
    trustingFewerTags.close();
  }
}
