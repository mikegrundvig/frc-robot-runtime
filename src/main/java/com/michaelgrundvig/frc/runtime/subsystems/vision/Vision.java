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
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.tunable.TunableBase;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.TunableRegistry;
import org.wpilib.util.Alert;

/**
 * The robot's cameras: reads each one's results, and hands each image to localization ({@link
 * LocalizationService#camera}), which solves it through the {@link VisionPipeline} as the robot was
 * when it was taken. Call {@link #periodic} after the drivetrain's odometry and before
 * localization's update, then {@link #record} with what the update made of each image.
 *
 * <p>For the pit and the log, per camera: whether it's connected, what became of every image and
 * why, and each tag as the camera sees it (placed through the estimated pose and the camera's
 * configured mount) with how far that lands from the tag's real place. A camera whose tags land
 * consistently off is mounted differently than its configuration says: bumped at an event, say.
 * Past a tolerance it raises an alert to recalibrate it, and until then its measurements count for
 * less, by how far its tags land off across the floor (see {@link MountCheck}).
 *
 * <p>Each image's latency, from when it was taken to the loop that reads it, is logged too. One
 * that arrives before it was taken, or much later than its camera's usual latency, has the wrong
 * capture time (the camera's clock and the robot's out of step): it's left out, counted, and an
 * alert stays up for two seconds after the last.
 *
 * <p>How far images are trusted is the robot's configuration ({@code RobotConfig.vision}), whose
 * main thresholds and uncertainties Pitwall tunes. Two switches are WPILib tunables, for the drive
 * team as much as the pit: one per camera to leave it out (a camera fogged over, knocked loose, or
 * blinded by stage lights), and how long to hold the cameras back when autonomous starts.
 *
 * <p>A task lining up on a target can trust only that target's tags while it runs ({@link
 * #trustingOnly}), as a top team's auto-align did.
 */
public final class Vision implements AutoCloseable {
  /** The share of measurements accepted is shown to this many percent. */
  private static final double ACCEPTED_STEP_PERCENT = 5;

  /** The share of measurements accepted as last shown: NaN before there was one. */
  private double shownAcceptedPercent = Double.NaN;

  private final List<Camera> cameras;
  private final LocalizationService localization;
  private FieldModel field;
  private final Supplier<VisionConfig> settings;

  /** The robot's pose is good enough to calibrate a camera against: within 5 cm. */
  private static final double CALIBRATION_POSE_STD_DEV_METERS = 0.05;

  /**
   * A camera's mount is judged by the latest 500 tags it saw: eight seconds of seeing two tags in
   * every image at 30 images a second, and enough for noise and the estimate's own errors to
   * average out. A knock shows in proportion: one that puts tags 10 cm off is past a 5 cm tolerance
   * after 250 tags.
   */
  private static final int MOUNT_SIGHTINGS = 500;

  /**
   * A camera's mount is judged only while another camera's measurement agreed with the estimate
   * this lately: the others hold the pose it's judged against, and odometry carries it between. Two
   * seconds, since cameras facing other ways see tags at other moments, and a camera's partner
   * facing the same way can be pulled off with it.
   */
  private static final long OTHERS_HOLD_NANOS = 2_000_000_000L;

  /**
   * A camera is calibrated only from images taken standing still, this long: no blur, and no
   * question of exactly when the image was taken.
   */
  private static final long STILL_NANOS = 500_000_000L;

  /** Slower than this, the robot counts as standing still: 2 cm and about a degree a second. */
  private static final double STILL_METERS_PER_SECOND = 0.02;

  private static final double STILL_RADIANS_PER_SECOND = 0.02;

  private long movingUntil = Long.MIN_VALUE;

  // Calibration: headings the cameras measure, and one camera's sightings for its mount.
  private @Nullable HeadingAverage headings;
  private @Nullable CameraMountCalibration mountCalibration;
  private @Nullable Camera calibrating;

  /**
   * How long to hold the cameras' measurements back when autonomous starts, trusting the placed
   * start while the robot launches: a top team held theirs 2 s. Off by default: the estimator gates
   * every measurement against the placed start anyway.
   */
  private final TunableDouble autoStartHoldSeconds =
      TunableDouble.createConfig(0.0, Limits.between(0, MAX_HOLD_SECONDS).config());

  /** Longer than any autonomous period has been. */
  private static final double MAX_HOLD_SECONDS = 30;

  private long holdUntilNanos = Long.MIN_VALUE;
  private final List<String> tunablePaths = new ArrayList<>();
  private VisionConfig current;

  /** When the previous loop read the cameras: what each image read now arrived after. */
  private long previousLoopNanos = Long.MIN_VALUE;

  /** The robot's alliance, once known: a task lining up on a place trusts that alliance's tags. */
  private final Supplier<Optional<Alliance>> alliance;

  /** The tasks holding the cameras to their tags, oldest first: the latest held is in effect. */
  private final List<TagLease> leases = new ArrayList<>();

  private final Alert trustingFewerTags =
      new Alert("Vision", "trustingFewerTags", "The cameras trust only some tags", Alert.Level.LOW);

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
   * @param settings how far images are trusted, as tuned: read once a loop
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

  /**
   * From now on, every heading the cameras measure (multi-tag solutions, standing still) goes into
   * this, until stopped: a heading reference for calibrating the gyro.
   */
  public void collectHeadings(HeadingAverage into) {
    headings = into;
  }

  public void stopCollectingHeadings() {
    headings = null;
  }

  /**
   * From now on, a camera's tag sightings go into this calibration of its mount, taken while the
   * robot stands still and the other cameras hold its pose within 5 cm; its measurements stay out
   * of the estimator, so the pose comes from the others. Until stopped.
   */
  public void calibrateMount(String camera, CameraMountCalibration into) {
    calibrating = camera(camera);
    mountCalibration = into;
  }

  public void stopCalibratingMount() {
    calibrating = null;
    mountCalibration = null;
  }

  /** Whether a calibration is collecting from the cameras: headings, or one camera's mount. */
  public boolean isCalibrating() {
    return calibrating != null || headings != null;
  }

  /**
   * A task lining up on a place trusts only the tags the place is found from, while it runs: lining
   * up on the hub, the cameras measure the robot from the hub's tags alone. Which tags depends on
   * the alliance, and a binding is made before the alliance is known, so they're found when the
   * task starts. With no alliance known by then, it trusts every tag, and says so.
   *
   * @throws IllegalArgumentException if the place is written as a pose: it has no tags to trust
   */
  public Command trustingOnly(Place place, Command task) {
    if (place.tags(field, Alliance.BLUE).isEmpty()) {
      throw new IllegalArgumentException(
          place.name() + " is written as a pose, not found from tags: it has no tags to trust");
    }
    return trusting(
        task,
        () ->
            alliance
                .get()
                .map(ours -> trustOnly(TagAllowlist.of(place.name(), place.tags(field, ours))))
                .orElseGet(
                    () ->
                        hold(
                            null,
                            place.name() + ": the alliance isn't known, so every tag is trusted")),
        task.name() + " (trusting " + possessive(place.name()) + " tags)");
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
   * The task, holding the cameras to its tags while it runs. It requires what the task requires, at
   * the task's priority: a task that interrupts it ends it before starting, so the one ending lets
   * go of its tags before the one starting takes its own, and a task refused never touches them.
   */
  private Command trusting(Command task, Supplier<TagLease> take, String name) {
    Holding holding = new Holding();
    return Command.requiring(task.requirements())
        .executing(
            coroutine -> {
              holding.lease = take.get();
              coroutine.await(task);
              holding.release();
            })
        // A canceled command never resumes, so nothing after the await runs (not even a finally).
        .whenCanceled(holding::release)
        .withPriority(task.priority())
        .named(name);
  }

  /** One run's hold on the cameras' tags. */
  private static final class Holding {
    private @Nullable TagLease lease;

    void release() {
      TagLease held = lease;
      if (held != null) {
        held.release();
        lease = null;
      }
    }
  }

  /**
   * From now on, the cameras measure the robot only from these tags, until the lease is released.
   * Every tag is still logged, and still checks its camera's mount. {@link #trustingOnly} does both
   * around a task, which is safer: a task canceled partway never reaches its own last line.
   */
  public TagLease trustOnly(TagAllowlist tags) {
    return hold(tags, "The cameras trust only " + tags.describe());
  }

  private TagLease hold(@Nullable TagAllowlist tags, String says) {
    TagLease lease = new TagLease(tags, says);
    leases.add(lease);
    return lease;
  }

  /**
   * One task's hold on which tags the cameras trust. The latest lease still held is in effect;
   * releasing one gives the cameras back to the one before, or to every tag.
   */
  public final class TagLease {
    private final @Nullable TagAllowlist tags;
    private final String says;

    private TagLease(@Nullable TagAllowlist tags, String says) {
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
    return trustingFewerTags.get() ? trustingFewerTags.getText() : "";
  }

  /** The tags the cameras trust now; null for every tag. */
  private @Nullable TagAllowlist onlyTags() {
    return leases.isEmpty() ? null : leases.getLast().tags;
  }

  /**
   * Autonomous just started: hold the cameras' measurements back for the tuned time, if any. They
   * are still read and logged, marked held.
   */
  public void autonomousStarted(long nowNanos) {
    // A tuned value that isn't a sensible time holds nothing, and never throws in the loop.
    double seconds = autoStartHoldSeconds.get();
    holdUntilNanos =
        seconds > 0 && Double.isFinite(seconds)
            ? nowNanos + Timestamps.fromSeconds(Math.min(seconds, MAX_HOLD_SECONDS))
            : Long.MIN_VALUE;
  }

  /**
   * Reads every camera and hands localization each usable image. Each is solved with the robot
   * tilted as it was when the image was taken, since a tag seen from a tilted camera as if level
   * lands off by the tilt times its distance.
   *
   * @param nowNanos the robot's clock this loop: how late each image arrived is measured to it
   */
  public void periodic(long nowNanos) {
    LocalizationEstimate estimate = localization.estimate();
    if (estimate.fieldVelocity().getNorm() > STILL_METERS_PER_SECOND
        || Math.abs(estimate.yawRateRadiansPerSecond()) > STILL_RADIANS_PER_SECOND) {
      movingUntil = estimate.timestampNanos();
    }
    current = settings.get();
    localization.vision(current);
    TagAllowlist trusted = onlyTags();
    Logger.recordOutput("Vision/OnlyTags", trusted != null ? trusted.describe() : "");
    if (!leases.isEmpty()) {
      trustingFewerTags.setText(leases.getLast().says);
    }
    trustingFewerTags.set(!leases.isEmpty());
    Motion motion =
        new Motion(
            estimate.covariance().largestPositionStdDev(),
            estimate.covariance().headingStdDev(),
            estimate.mode() == LocalizationEstimate.Mode.TRACKING,
            poseKnown(estimate));

    for (Camera camera : cameras) {
      camera.read();
      camera.checkMount(current.mountToleranceMeters());
      localization.mountDoubt(
          camera.name(), camera.mountDoubtMeters(), camera.mountDistanceMeters());
      int next = 0;
      for (CameraIO.Frame frame : camera.inputs.frames) {
        int first = next;
        next += frame.tagCount();
        // One image's bad data costs that image, never the robot program: whatever a camera sends
        // that the checks miss ends here, logged as the image's outcome.
        try {
          processFrame(camera, frame, first, motion, nowNanos);
        } catch (RuntimeException e) {
          camera.outcome("unusable data from the camera: " + e.getMessage());
        }
      }
      camera.logLoop(nowNanos);
    }
    previousLoopNanos = nowNanos;
  }

  /**
   * How well the robot's pose is known this loop: what each camera's mount is judged against.
   *
   * @param positionStdDev how sure the estimate's position is, meters, the least sure way
   * @param headingStdDev how sure the estimate's heading is, radians
   * @param tracking whether recent camera measurements agree with odometry
   * @param poseKnown whether the pose is held well enough to calibrate a camera against
   */
  private record Motion(
      double positionStdDev, double headingStdDev, boolean tracking, boolean poseKnown) {}

  /**
   * One image: solved into a measurement for localization (unless it's held back), and its tags
   * placed on the field, for the camera's residual and any calibration collecting them.
   */
  private void processFrame(
      Camera camera, CameraIO.Frame frame, int first, Motion motion, long nowNanos) {
    TagDetections detections = detections(frame, camera.inputs.tags, first, camera.name());
    List<Pose3d> seen = onTheField(detections);
    if (!camera.enabled.get()) {
      camera.leftOut(seen, "camera turned off");
      return;
    }
    if (!camera.isNew(frame.timestampNanos())) {
      camera.outcome("the same image again");
      return;
    }
    // An image that arrived before it was taken, or far later than its camera's images do, has
    // the wrong capture time: solved as of that time, it would put the robot where it wasn't.
    if (!camera.arrivedInTime(
        frame.timestampNanos(),
        nowNanos,
        previousLoopNanos,
        current.latencyMarginSeconds(),
        seen)) {
      return;
    }
    Pose2d then = localization.poseAt(frame.timestampNanos()).orElse(null);
    CaptureContext capture = localization.capture(frame.timestampNanos());
    TagAllowlist trusted = onlyTags();
    // A camera being calibrated, or autonomous just started: solved and logged, never applied.
    Verdict held =
        camera == calibrating
            ? Verdict.CALIBRATING
            : frame.timestampNanos() < holdUntilNanos ? Verdict.HELD : null;
    VisionResult result =
        held != null
            ? localization.hold(detections, trusted, held)
            : localization.camera(detections, trusted);
    VisionMeasurement measurement = result.measurement();
    if (measurement == null) {
      camera.leftOut(seen, result.outcome());
    } else if (held != null) {
      camera.leftOut(seen, held.words() + ": " + result.outcome());
    } else {
      camera.outcome(result.outcome());
    }
    HeadingAverage collecting = headings;
    if (collecting != null && measurement != null && measurement.covariance().hasHeading()) {
      collecting.add(measurement.pose().getRotation());
    }
    // A camera on a joint isn't judged by its mount, nor calibrated: where it is is its joint's.
    if (then != null && camera.config.joint().isEmpty()) {
      // On raised terrain, the robot stands as high as the ground under it.
      Pose3d robotThen =
          new Pose3d(
              then.getX(),
              then.getY(),
              field.terrain().heightAt(then.getTranslation()),
              new Rotation3d(
                  capture.rollRadians(), capture.pitchRadians(), then.getRotation().getRadians()));
      CameraMountCalibration sightings =
          camera == calibrating
                  && motion.poseKnown()
                  && frame.timestampNanos() - movingUntil >= STILL_NANOS
              ? mountCalibration
              : null;
      // A camera's mount is judged only against a pose the other cameras hold: the robot turning
      // slowly, the estimate tracking, no calibration under way, and another camera's
      // measurement accepted lately. Each tag also needs the heading sure enough at its distance
      // (recordSightings): a heading off by an angle puts every tag that angle times its distance
      // off, the same way each time, which is just what a camera turned on its mount looks like.
      boolean judging =
          motion.tracking()
              && capture.recentYawRateRadiansPerSecond() <= current.maxYawRateForHeading()
              && !isCalibrating()
              && othersHold(camera, frame.timestampNanos());
      recordSightings(camera, robotThen, detections, judging ? motion : null, sightings);
    }
  }

  /**
   * Whether another camera holds the pose: the estimator accepted one of its measurements within
   * two seconds of a moment, and it agreed with the estimate within the mount tolerance. After the
   * heading has been disturbed (the gyro-scale procedure's spins, say), the estimate can stand
   * still centimeters off while it's sure to millimeters; the cameras, agreeing with each other but
   * not with it, are what show it.
   */
  private boolean othersHold(Camera camera, long timestampNanos) {
    for (Camera other : cameras) {
      if (other != camera && other.lastAgreedNanos() >= timestampNanos - OTHERS_HOLD_NANOS) {
        return true;
      }
    }
    return false;
  }

  /**
   * Places each tag an image saw on the field, through where the robot was and the camera's
   * configured mount, for the camera's residual and the check of its mount; and gives it to the
   * mount calibration collecting this camera's sightings, if one is.
   *
   * @param judging how the robot moved, if its pose is held well enough to judge the mount by; null
   *     if not
   */
  private void recordSightings(
      Camera camera,
      Pose3d robotThen,
      TagDetections detections,
      @Nullable Motion judging,
      @Nullable CameraMountCalibration sightings) {
    for (TagDetections.Tag tag : detections.tags()) {
      Pose3d seen =
          robotThen.transformBy(camera.config.robotToCamera()).transformBy(tag.bestCameraToTag());
      Pose3d real = field.tag(tag.id()).orElse(null);
      double distance = tag.bestCameraToTag().getTranslation().getNorm();
      boolean judge =
          judging != null
              && Math.hypot(judging.positionStdDev(), distance * judging.headingStdDev())
                  <= current.mountToleranceMeters();
      camera.saw(robotThen, seen, real, judge);
      if (real != null && sightings != null) {
        sightings.add(robotThen, tag.bestCameraToTag(), tag.altCameraToTag(), real);
      }
    }
  }

  /** Whether the other cameras hold the robot's pose well enough to calibrate one against. */
  private static boolean poseKnown(LocalizationEstimate estimate) {
    return estimate.mode() == LocalizationEstimate.Mode.TRACKING
        && estimate.covariance().largestPositionStdDev() <= CALIBRATION_POSE_STD_DEV_METERS;
  }

  /** Counts and logs what localization's update made of each camera's measurements. */
  public void record(List<FrameVerdict> frames) {
    for (Camera camera : cameras) {
      camera.record(frames, current.mountToleranceMeters(), field::tag);
    }
  }

  /**
   * The venue the pit selected, its tags where a survey found them: mount checks judge against it
   * from now on. Only while disabled, as {@code RobotContainer} calls it.
   */
  public void venue(FieldModel venue) {
    field = venue;
  }

  /**
   * An image's tags where the field's layout has them; a tag the field doesn't have is left out.
   */
  private List<Pose3d> onTheField(TagDetections detections) {
    List<Pose3d> tags = new ArrayList<>(detections.tags().size());
    for (TagDetections.Tag tag : detections.tags()) {
      field.tag(tag.id()).ifPresent(tags::add);
    }
    return tags;
  }

  /**
   * One line for the pit: which cameras are connected, how many of their measurements are accepted,
   * to 5%, and how many images were left out because they arrived out of step. The share shown
   * moves only once the real one has moved 5% from it, so the line doesn't change with every
   * measurement (or flicker at a rounding edge); the images left out are counted exactly, since
   * each is news, and the page hears of the line at most once a second.
   */
  public String summary() {
    int connected = 0;
    int accepts = 0;
    int rejects = 0;
    int outOfStep = 0;
    for (Camera camera : cameras) {
      connected += camera.isConnected() ? 1 : 0;
      accepts += camera.accepted();
      rejects += camera.rejected();
      outOfStep += camera.leftOutOfStep();
    }
    String fate = "no measurements yet";
    if (accepts + rejects > 0) {
      double percent = 100.0 * accepts / (accepts + rejects);
      if (!(Math.abs(percent - shownAcceptedPercent) < ACCEPTED_STEP_PERCENT)) {
        shownAcceptedPercent = Math.round(percent / ACCEPTED_STEP_PERCENT) * ACCEPTED_STEP_PERCENT;
      }
      fate = String.format(Locale.ROOT, "%.0f%% of measurements accepted", shownAcceptedPercent);
    }
    return connected
        + " of "
        + cameras.size()
        + " cameras connected, "
        + fate
        + (outOfStep > 0 ? ", " + outOfStep + " images left out of step" : "");
  }

  /**
   * How far, on average since the robot started, a camera's tags land from where they really are,
   * placed through the estimated pose and the camera's configured mount: the pre-event check that
   * each camera is still mounted as configured. A few centimeters is normal; one camera well above
   * the others has moved.
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

  /** How long autonomous's start holds the cameras back, in seconds: zero is off. */
  TunableDouble autoStartHoldSeconds() {
    return autoStartHoldSeconds;
  }

  /** A camera, in the order of the cameras' configuration: its switch, alerts, and tallies. */
  Camera camera(int index) {
    return cameras.get(index);
  }

  /**
   * One frame's tags, from the flat list the IO logs, as the pipeline takes them. A tag whose pose
   * isn't a number is left out, and so is a multi-tag solution that isn't; an ambiguity that isn't
   * one counts as a coin toss.
   */
  private static TagDetections detections(
      CameraIO.Frame frame, CameraIO.SeenTag[] tags, int first, String camera) {
    List<TagDetections.Tag> seen = new ArrayList<>();
    List<Integer> usedByMultiTag = new ArrayList<>();
    for (int k = first; k < Math.min(first + frame.tagCount(), tags.length); k++) {
      CameraIO.SeenTag tag = tags[k];
      if (!isFinite(tag.bestCameraToTag()) || !isFinite(tag.altCameraToTag())) {
        continue;
      }
      seen.add(
          new TagDetections.Tag(
              tag.id(),
              List.of(
                  new Translation2d(tag.corner0X(), tag.corner0Y()),
                  new Translation2d(tag.corner1X(), tag.corner1Y()),
                  new Translation2d(tag.corner2X(), tag.corner2Y()),
                  new Translation2d(tag.corner3X(), tag.corner3Y())),
              tag.bestCameraToTag(),
              tag.altCameraToTag(),
              // PhotonVision reports -1 when it has no ambiguity to give: treat that as unknown.
              !(tag.ambiguity() >= 0) ? 1 : Math.min(tag.ambiguity(), 1)));
      if (tag.inMultiTag()) {
        usedByMultiTag.add(tag.id());
      }
    }
    TagDetections.MultiTag multiTag = null;
    if (frame.hasMultiTag()
        && !usedByMultiTag.isEmpty()
        && isFinite(frame.multiTagFieldToCamera())
        && Double.isFinite(frame.multiTagReprojectionErrorPixels())) {
      multiTag =
          new TagDetections.MultiTag(
              frame.multiTagFieldToCamera(),
              Math.max(0, frame.multiTagReprojectionErrorPixels()),
              usedByMultiTag);
    }
    return new TagDetections(frame.timestampNanos(), camera, seen, multiTag);
  }

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
