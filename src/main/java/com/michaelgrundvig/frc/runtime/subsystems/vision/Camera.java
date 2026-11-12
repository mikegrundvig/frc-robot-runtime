package com.michaelgrundvig.frc.runtime.subsystems.vision;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.localization.DefaultLocalizer;
import com.michaelgrundvig.frc.localization.FrameVerdict;
import com.michaelgrundvig.frc.localization.MountCheck;
import com.michaelgrundvig.frc.localization.TagVerdict;
import com.michaelgrundvig.frc.localization.Timestamps;
import com.michaelgrundvig.frc.localization.Verdict;
import com.michaelgrundvig.frc.runtime.lib.Said;
import com.michaelgrundvig.frc.runtime.lib.health.CameraHealth;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.OptionalLong;
import java.util.function.IntFunction;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.tunable.TunableBoolean;
import org.wpilib.tunable.TunableRegistry;
import org.wpilib.util.Alert;

/**
 * One of the robot's cameras, as {@link Vision} runs it. It holds the camera's IO and what it read
 * this loop, its alerts, the switch that leaves it out, its tallies, and the check that it still
 * sits where its configuration says. What became of this loop's images, and which tags it saw, are
 * logged under {@code Vision/<name>}.
 *
 * <p>A loop of a camera goes in this order, which {@link Vision} keeps:
 *
 * <ol>
 *   <li>{@link #read} reads the camera, judges its health ({@link CameraHealthLog}), and starts the
 *       loop's record.
 *   <li>{@link #checkMount} judges the mount by the tags seen in earlier loops, so that this loop's
 *       measurements can be doubted for it.
 *   <li>Each image is taken in turn: it is left out ({@link #leftOut}) or given to localization
 *       ({@link #verdict}), and its tags are recorded ({@link #saw}, {@link #landed}, {@link
 *       #judgeMountBy}).
 *   <li>{@link #logLoop} logs the loop's images.
 *   <li>{@link #record}, after localization's update, logs what the update made of them.
 * </ol>
 *
 * <p>A tag the camera saw is logged two ways, because the log answers two questions with it.
 *
 * <ul>
 *   <li>{@code SeenTags} is where the camera put the tag, through the estimated pose and the
 *       camera's configured mount. Its gap from the field's tag is the camera's error, which the
 *       mount check judges.
 *   <li>{@code AcceptedTags} and {@code RejectedTags} are the field's own tags, sorted by what
 *       became of the images that saw them. They show what each measurement rested on, and
 *       AdvantageScope draws a line from the robot to each.
 * </ul>
 *
 * <p>The two can't be one list. An image taken while the estimate didn't know where the robot was
 * has no placed tag, and it is still accepted or refused.
 *
 * <p>Every image the camera hands over comes to one {@link Verdict}, whatever decided it: the robot
 * (turned off, the same image again, out of step, no capture time, unreadable data), the pipeline,
 * or the estimator. Each is counted, since the robot started, under {@code
 * Vision/<name>/Images/<VERDICT>}, so how many images were thrown out, and why, is read straight
 * off the log. A camera on Visor, on a robot, with no lens calibration places nothing, and raises
 * an alert that says so from the start.
 */
final class Camera implements AutoCloseable {
  /** The out-of-step alert stays up this long after the last image out of step. */
  private static final long OUT_OF_STEP_HOLD_NANOS = 2_000_000_000L;

  /** How long the no-capture-time alert stays up after the last image without one. */
  private static final long NO_CAPTURE_TIME_HOLD_NANOS = 2_000_000_000L;

  /** The out-of-calibration alert, once up, comes down at this share of the tolerance. */
  private static final double ALERT_CLEARS_AT = 0.8;

  final CameraConfig config;
  private final CameraIO io;
  final CameraIOInputsAutoLogged inputs = new CameraIOInputsAutoLogged();

  /** Off leaves the camera out: fogged over, knocked loose, or blinded by stage lights. */
  final TunableBoolean enabled = TunableBoolean.create(true);

  // ---- The log's names ----

  /** The camera's place in the log, {@code Vision/<name>}. */
  private final String key;

  // Each value's name in the log. They are made once, because a name made every loop is a new
  // string every loop.
  private final String verdictsKey;
  private final String leftOutOfStepKey;
  private final String latencySecondsKey;
  private final String seenTagsKey;
  private final String mountOffsetMetersKey;
  private final String mountDoubtMetersKey;
  private final String tagResidualMetersKey;
  private final String meanTagResidualMetersKey;
  private final String measurementVerdictsKey;
  private final String acceptedKey;
  private final String rejectedKey;
  private final String givenKey;
  private final String acceptedTagsKey;
  private final String rejectedTagsKey;
  private final String rejectedVerdictsKey;
  private final String learnedTurnDegreesKey;
  private final String lateSecondsKey;
  private final String watchDoubtMetersKey;

  /** What became of each of its tags, logged as facts for Pitwall's World view. */
  private final SightingLog sightings;

  /** Its frame rate, drops and detection time, logged under {@code Vision/<name>/Health}. */
  private final CameraHealthLog health;

  // ---- The alerts ----

  private final Alert disconnected;
  private final Alert unreadable;
  private final Alert outOfCalibration;
  private final Alert outOfStep;
  private final Alert noCaptureTime;
  private final Alert noLens;

  // ---- Images that arrive out of step ----

  /** The out-of-step alert is up until this time. It is empty while no image has been late. */
  private OptionalLong outOfStepAlertHeldUntil = OptionalLong.empty();

  private OptionalLong noCaptureTimeHeldUntil = OptionalLong.empty();

  /** How many images were left out for arriving out of step, since the robot started. */
  private int leftOutOfStep;

  // ---- The mount ----

  /** Whether the camera still sits where its configuration says, from where its tags land. */
  private final MountCheck mount;

  /** How far this loop's measurements may be off from where the mount moved, in meters. */
  private double mountDoubt;

  /** Whether the camera counted as out of calibration the last time its mount was judged. */
  private boolean flaggedOutOfCalibration;

  /**
   * What the out-of-calibration alert says now: how far off the tags land, in whole centimeters,
   * and whether the camera counts for less. The centimeters are empty until the alert first says
   * something.
   */
  private OptionalInt shownCentimeters = OptionalInt.empty();

  private boolean shownTrustedLess;

  // ---- The tallies, since the robot started ----

  /** When the newest image the camera handed over was taken. A camera can hand one over twice. */
  private OptionalLong newestImageNanos = OptionalLong.empty();

  /**
   * When the latest image was taken that the estimator accepted from this camera and that agreed
   * with the estimate closely. While that is recent, this camera holds the pose that the other
   * cameras' mounts are judged against.
   */
  private OptionalLong lastAgreedNanos = OptionalLong.empty();

  private int accepted;
  private int rejected;

  // ---- What became of its images, since the robot started ----

  private static final Verdict[] VERDICTS = Verdict.values();

  /** How many of its images came to each verdict, by the verdict's ordinal. */
  private final long[] images = new long[VERDICTS.length];

  /** Each count as last logged: -1 until it's logged, so the first loop logs them all. */
  private final long[] imagesLogged = new long[VERDICTS.length];

  /** Each count's name in the log, {@code Vision/<name>/Images/<VERDICT>}. */
  private final String[] imagesKeys = new String[VERDICTS.length];

  /** What became of its latest image, in words, as its verdict said it; empty before the first. */
  private @Nullable String latestVerdict;

  /**
   * Whether the image being taken has a verdict yet, so one that then throws isn't counted twice.
   */
  private boolean imageHasAVerdict;

  /** Whether the robot's localizer says what became of each measurement it was given. */
  enum MeasurementVerdicts {
    REPORTED,
    NOT_REPORTED
  }

  /**
   * How many of this camera's measurements went to a localizer that does not report what it does
   * with them ({@code Verdict.GIVEN}): neither accepted nor rejected, as far as anything here
   * knows.
   */
  private int given;

  /** The distances from each tag seen to where the tag really is, added up, and how many. */
  private double residualSumMeters;

  private int residualCount;

  // ---- This loop's record ----

  /** What became of each of this loop's images, in words. */
  private final List<String> verdicts = new ArrayList<>();

  /** How long after it was taken each of this loop's images was read, in seconds. */
  private final List<Double> latencies = new ArrayList<>();

  /** This loop's tags, where the camera put them on the field. */
  private final List<Pose3d> seenTags = new ArrayList<>();

  private double loopResidualSumMeters;
  private int loopResidualCount;

  /**
   * This loop's tags where the field's layout has them, sorted by what became of the images that
   * saw them, and why each refused one was refused, in the same order as the refused tags.
   */
  private final List<Pose3d> acceptedTags = new ArrayList<>();

  private final List<Pose3d> rejectedTags = new ArrayList<>();
  private final List<String> rejectedVerdicts = new ArrayList<>();

  /** The arrays those three were last logged in, which are used again while their lengths hold. */
  private Pose3d[] acceptedArray = {};

  private Pose3d[] rejectedArray = {};
  private String[] rejectedVerdictsArray = {};

  /**
   * @param mountSightings how many of its latest tags the camera's mount is judged by
   */
  Camera(CameraConfig config, CameraIO io, int mountSightings) {
    this.config = config;
    this.io = io;
    mount = new MountCheck(mountSightings);
    String name = config.name();
    key = "Vision/" + name;
    verdictsKey = key + "/Verdicts";
    leftOutOfStepKey = key + "/LeftOutOfStep";
    latencySecondsKey = key + "/LatencySeconds";
    seenTagsKey = key + "/SeenTags";
    mountOffsetMetersKey = key + "/MountOffsetMeters";
    mountDoubtMetersKey = key + "/MountDoubtMeters";
    tagResidualMetersKey = key + "/TagResidualMeters";
    meanTagResidualMetersKey = key + "/MeanTagResidualMeters";
    measurementVerdictsKey = key + "/MeasurementVerdicts";
    acceptedKey = key + "/Accepted";
    rejectedKey = key + "/Rejected";
    givenKey = key + "/Given";
    acceptedTagsKey = key + "/AcceptedTags";
    rejectedTagsKey = key + "/RejectedTags";
    rejectedVerdictsKey = key + "/RejectedVerdicts";
    learnedTurnDegreesKey = key + "/LearnedTurnDegrees";
    lateSecondsKey = key + "/LateSeconds";
    watchDoubtMetersKey = key + "/WatchDoubtMeters";
    for (Verdict verdict : VERDICTS) {
      imagesKeys[verdict.ordinal()] = key + "/Images/" + verdict.name();
    }
    Arrays.fill(imagesLogged, -1);
    sightings = new SightingLog(config.name(), key);
    health = new CameraHealthLog(config);
    disconnected =
        new Alert(
            "Vision", name + "Disconnected", "Camera " + name + " disconnected", Alert.Level.HIGH);
    unreadable =
        new Alert(
            "Vision", name + "Unreadable", "Camera " + name + " can't be read", Alert.Level.HIGH);
    outOfCalibration =
        new Alert(
            "Vision",
            name + "OutOfCalibration",
            "Camera " + name + " is out of calibration",
            Alert.Level.MEDIUM);
    outOfStep =
        new Alert(
            "Vision",
            name + "OutOfStep",
            "Camera " + possessive(name) + " images arrive out of step",
            Alert.Level.HIGH);
    noCaptureTime =
        new Alert("Vision", name + "NoCaptureTime", noCaptureTimeText(config), Alert.Level.HIGH);
    Optional<String> withoutALens = withoutALens(config);
    noLens =
        new Alert(
            "Vision",
            name + "NoLens",
            "Camera " + name + " " + withoutALens.orElse("has no lens calibration"),
            Alert.Level.HIGH);
    noLens.set(withoutALens.isPresent());
    TunableRegistry.publish(key + "/enabled", enabled);
  }

  String name() {
    return config.name();
  }

  /**
   * What's wrong with a camera that runs on Visor, on a robot (its device given), with no lens
   * calibration: Visor then sends its tags' corners and no poses, so it places nothing. Empty for
   * any other camera, and for one the simulator alone runs, which needs no lens.
   */
  static Optional<String> withoutALens(CameraConfig config) {
    if (!(config.software() instanceof CameraConfig.Software.Visor visor)) {
      return Optional.empty();
    }
    if (visor.device().isEmpty() || visor.lens().isPresent()) {
      return Optional.empty();
    }
    return Optional.of(
        "has no lens calibration: Visor sends its tags' corners and no poses, so it places"
            + " nothing. Calibrate it with Visor's tools/calibrate.py and give it the .lens(...)"
            + " that prints (docs/visor.md)");
  }

  // ---- Reading the camera ----

  /**
   * Reads the camera, shows whether it can be reached, judges its health, and starts this loop's
   * record.
   */
  void read(long nowNanos) {
    readInputs();
    Logger.processInputs(key, inputs);
    showWhetherItCanBeRead();
    health.take(inputs, nowNanos);
    startLoopRecord();
  }

  /**
   * Reads the camera's IO. Nothing the IO throws gets past here.
   *
   * <p>PhotonLib throws on purpose when the coprocessor runs a different PhotonVision version than
   * the robot's library, and again every few seconds. Let through, that would end the robot program
   * each time. Instead the camera reads as unreadable, with the reason, and the robot drives on its
   * other sensors.
   *
   * <p>The error is cleared before the read, so an IO that says why it can't be read (a Visor
   * version mismatch) is heard.
   */
  private void readInputs() {
    inputs.error = "";
    try {
      io.updateInputs(inputs);
    } catch (RuntimeException thrown) {
      inputs.connected = false;
      inputs.frames = new CameraIO.Frame[] {};
      inputs.tags = new CameraIO.SeenTag[] {};
      inputs.unsolvedTags = new int[] {};
      inputs.skippedFrames = 0;
      inputs.detectMs = new double[] {};
      inputs.error = Said.of(thrown);
    }
  }

  /**
   * Raises the alert that fits. A camera that can't be read says why, and isn't also said to be
   * disconnected, which would be the smaller part of the story.
   */
  private void showWhetherItCanBeRead() {
    // The inputs carry the reason as text, empty when the camera was read, because an AdvantageKit
    // input is a plain value that a log can hold.
    boolean readable = inputs.error.isEmpty();
    disconnected.set(!inputs.connected && readable);
    if (!readable) {
      unreadable.setText("Camera " + name() + " can't be read: " + inputs.error);
    }
    unreadable.set(!readable);
  }

  private void startLoopRecord() {
    verdicts.clear();
    latencies.clear();
    seenTags.clear();
    acceptedTags.clear();
    rejectedTags.clear();
    rejectedVerdicts.clear();
    loopResidualSumMeters = 0;
    loopResidualCount = 0;
    if (inputs.skippedFrames > 0) {
      verdict(inputs.skippedFrames + " image(s) without a capture time, left out");
      images[Verdict.NO_CAPTURE_TIME.ordinal()] += inputs.skippedFrames;
    }
  }

  // ---- One image ----

  /**
   * Whether an image is new. A camera can hand over the same image twice, and an image no newer
   * than the newest already taken is taken as one of those. Only an image that arrived in time
   * ({@link #taken}) moves the newest on: one stamped in the future must not make the good images
   * after it look old.
   */
  boolean isNew(long takenNanos) {
    return newestImageNanos.isEmpty() || takenNanos > newestImageNanos.getAsLong();
  }

  /** An image that's new and arrived in time was taken: images no newer than it are seen before. */
  void taken(long takenNanos) {
    newestImageNanos = OptionalLong.of(takenNanos);
  }

  /**
   * Whether an image arrived within the camera's band: no earlier than it was taken, and no later
   * than the camera's usual latency plus a margin. An image outside the band is left out, counted,
   * and an alert stays up for two seconds after the last one.
   *
   * <p>How late the image arrived is logged either way.
   *
   * @param takenNanos when the image was taken, by its stamp
   * @param nowNanos this loop's time
   * @param previousLoopNanos the previous loop's time, or empty on the first loop
   * @param marginSeconds how much later than the camera's usual latency is still in time
   * @param tags the image's tags, where the field's layout has them, which are refused with it
   */
  boolean arrivedInTime(
      long takenNanos,
      long nowNanos,
      OptionalLong previousLoopNanos,
      double marginSeconds,
      List<Pose3d> tags) {
    latencies.add(Timestamps.toSeconds(nowNanos - takenNanos));
    Optional<String> outOfStepBecause =
        whyOutOfStep(takenNanos, nowNanos, previousLoopNanos, marginSeconds);
    if (outOfStepBecause.isEmpty()) {
      return true;
    }
    leaveOutOfStep(tags, outOfStepBecause.get(), nowNanos);
    return false;
  }

  /**
   * Why an image's capture time can't be right, or empty when it can.
   *
   * <p>An image stamped after this loop's time arrived before it was taken, which nothing does.
   *
   * <p>An image is late when it arrived more than the camera's usual latency plus the margin after
   * it was taken. An image read this loop arrived at some time after the previous loop read the
   * camera, so it is certainly late only if it was late already then. Measuring to the previous
   * loop means that a loop which ran long doesn't make the camera's images late. The first loop has
   * no loop before it, so nothing it reads counts as late.
   */
  private Optional<String> whyOutOfStep(
      long takenNanos, long nowNanos, OptionalLong previousLoopNanos, double marginSeconds) {
    if (takenNanos > nowNanos) {
      double earlySeconds = Timestamps.toSeconds(takenNanos - nowNanos);
      return Optional.of(
          String.format(Locale.ROOT, "arrived %.0f ms before it was taken", earlySeconds * 1000));
    }
    if (previousLoopNanos.isEmpty()) {
      return Optional.empty();
    }
    double latestSeconds = config.latencyOrDefault().in(Seconds) + marginSeconds;
    double lateAtLeastSeconds = Timestamps.toSeconds(previousLoopNanos.getAsLong() - takenNanos);
    if (lateAtLeastSeconds > latestSeconds) {
      return Optional.of(
          String.format(
              Locale.ROOT,
              "arrived at least %.0f ms after it was taken, more than %.0f ms",
              lateAtLeastSeconds * 1000,
              latestSeconds * 1000));
    }
    return Optional.empty();
  }

  /** Leaves an image out for arriving out of step, counts it, and holds the alert up. */
  private void leaveOutOfStep(List<Pose3d> tags, String why, long nowNanos) {
    leftOut(tags, why);
    leftOutOfStep++;
    // The alert says why once, as it goes up. Its text isn't changed with every image while it
    // stays up, which would make it unreadable.
    if (!outOfStepAlertIsHeldAt(nowNanos)) {
      outOfStep.setText(
          "Camera "
              + possessive(name())
              + " images are out of step, and left out: one "
              + why
              + ". Are its clock and the robot's in step (time sync)? If it's just slower,"
              + " measure its latency");
    }
    outOfStepAlertHeldUntil = OptionalLong.of(nowNanos + OUT_OF_STEP_HOLD_NANOS);
  }

  private boolean outOfStepAlertIsHeldAt(long nowNanos) {
    return outOfStepAlertHeldUntil.isPresent() && nowNanos < outOfStepAlertHeldUntil.getAsLong();
  }

  /** What became of one of this loop's images. */
  void verdict(String words) {
    verdicts.add(words);
    latestVerdict = words;
  }

  /** An image is about to be taken: it has no verdict yet. */
  void imageStarts() {
    imageHasAVerdict = false;
  }

  /**
   * The image being taken has its verdict, and is counted by it once localization's update says it
   * ({@link #record}).
   */
  void imageHasAVerdict() {
    imageHasAVerdict = true;
  }

  /**
   * The image being taken couldn't be read: its data threw. Its verdict says why, and it's counted
   * as unreadable, unless it already had its verdict before it threw.
   */
  void unreadable(String why) {
    verdict("unusable data from the camera: " + why);
    if (!imageHasAVerdict) {
      images[Verdict.UNREADABLE.ordinal()]++;
    }
  }

  /**
   * One of this loop's images was left out before the estimator saw it. Why is its verdict, and its
   * tags are refused for the same reason.
   *
   * @param tags the image's tags, where the field's layout has them
   */
  void leftOut(List<Pose3d> tags, String why) {
    imageHasAVerdict();
    verdict(why);
    for (Pose3d tag : tags) {
      rejectedTags.add(tag);
      rejectedVerdicts.add(why);
    }
  }

  /**
   * A tag this loop's image saw, where the camera puts it on the field: through the estimated pose
   * and the camera's configured mount.
   */
  void saw(Pose3d seen) {
    seenTags.add(seen);
  }

  /**
   * How far a tag the camera saw landed from where the field has it. The distance goes into the
   * camera's tag residual, this loop's and since the robot started.
   */
  void landed(Pose3d seen, Pose3d real) {
    loopResidualSumMeters += seen.getTranslation().getDistance(real.getTranslation());
    loopResidualCount++;
  }

  /**
   * Counts a tag toward the judgement of the camera's mount. {@link Vision} calls it only for a tag
   * seen while the robot's pose was known well enough to judge a mount by.
   *
   * @param robot where the robot was, raised and tilted as it was
   */
  void judgeMountBy(Pose3d robot, Pose3d seen, Pose3d real) {
    mount.add(robot, seen, real);
  }

  // ---- The mount ----

  /**
   * Judges the camera's mount by the tags it has seen so far: whether it's out of calibration, and
   * how much less this loop's measurements are trusted for it. Once a loop, after {@link #read}.
   *
   * @param toleranceMeters how far a calibrated camera's tags land off, at most, on average
   */
  void checkMount(double toleranceMeters) {
    mountDoubt = mount.doubtMeters(toleranceMeters);
    flaggedOutOfCalibration = isOutOfCalibration(toleranceMeters);
    if (flaggedOutOfCalibration) {
      sayHowFarOff();
    }
    outOfCalibration.set(flaggedOutOfCalibration);
  }

  /**
   * Whether the camera's tags land too far off. The alert goes up past the tolerance, and comes
   * down only once the offset is back under 0.8 of it, so an offset that hovers at the tolerance
   * doesn't flicker the alert on and off. A mount that hasn't seen enough tags to be judged is not
   * out of calibration.
   */
  private boolean isOutOfCalibration(double toleranceMeters) {
    double limitMeters = toleranceMeters;
    if (flaggedOutOfCalibration) {
      limitMeters = ALERT_CLEARS_AT * toleranceMeters;
    }
    return mount.judged() && mount.offsetMeters() > limitMeters;
  }

  /**
   * Puts how far off the tags land into the alert, in whole centimeters, and whether the camera
   * counts for less. The text is written again only when one of the two changes, so that it holds
   * still while the offset wanders within a centimeter.
   */
  private void sayHowFarOff() {
    int centimeters = (int) Math.round(mount.offsetMeters() * 100);
    boolean trustedLess = mountDoubt > 0;
    boolean saidAlready =
        shownCentimeters.isPresent()
            && shownCentimeters.getAsInt() == centimeters
            && shownTrustedLess == trustedLess;
    if (saidAlready) {
      return;
    }
    String countsForLess = "";
    if (trustedLess) {
      countsForLess = ", so it counts for less";
    }
    outOfCalibration.setText(
        String.format(
            Locale.ROOT,
            "Camera %s is out of calibration: its tags land %d cm off%s. Recalibrate its mount"
                + " (Calibrate the %s camera's mount)",
            name(),
            centimeters,
            countsForLess,
            name()));
    shownCentimeters = OptionalInt.of(centimeters);
    shownTrustedLess = trustedLess;
  }

  /** How far this loop's measurements may be off from where the camera's mount moved, in meters. */
  double mountDoubtMeters() {
    return mountDoubt;
  }

  /**
   * How far away the tags its mount was judged by were, on average. Not a number until the mount is
   * judged, which is how localization takes it ({@code LocalizationService.mountDoubt}).
   */
  double mountDistanceMeters() {
    return mount.distanceMeters();
  }

  /**
   * How far off, on average, the camera's latest tags land. Not a number until the camera has seen
   * enough tags to judge its mount by, as the log and {@link Vision#mountOffsetMeters} have it.
   */
  double mountOffsetMeters() {
    return mount.offsetMeters();
  }

  // ---- Logging the loop ----

  /**
   * Logs this loop's images and tags, the tag residual, and how far off the camera's mount looks,
   * and keeps the out-of-step alert up while it is held.
   */
  void logLoop(long nowNanos) {
    // A camera that is turned off or gone has an alert or a switch that says more, so the
    // out-of-step alert comes down at once.
    if (!enabled.get() || !inputs.connected) {
      outOfStepAlertHeldUntil = OptionalLong.empty();
    }
    outOfStep.set(outOfStepAlertIsHeldAt(nowNanos));
    showWhetherImagesHaveCaptureTimes(nowNanos);
    Logger.recordOutput(verdictsKey, verdicts.toArray(String[]::new));
    Logger.recordOutput(leftOutOfStepKey, leftOutOfStep);
    Logger.recordOutput(
        latencySecondsKey, latencies.stream().mapToDouble(Double::doubleValue).toArray());
    Logger.recordOutput(seenTagsKey, seenTags.toArray(Pose3d[]::new));
    Logger.recordOutput(mountOffsetMetersKey, mount.offsetMeters());
    Logger.recordOutput(mountDoubtMetersKey, mountDoubt);
    logTagResidual();
  }

  /**
   * Keeps the no-capture-time alert up while images keep arriving without one, and for a while
   * after: a camera whose coprocessor's clock isn't synced to the robot's sends every image so, and
   * all of them are left out. A camera that's off or gone says that instead.
   */
  private void showWhetherImagesHaveCaptureTimes(long nowNanos) {
    if (inputs.skippedFrames > 0) {
      noCaptureTimeHeldUntil = OptionalLong.of(nowNanos + NO_CAPTURE_TIME_HOLD_NANOS);
    }
    if (!enabled.get() || !inputs.connected) {
      noCaptureTimeHeldUntil = OptionalLong.empty();
    }
    noCaptureTime.set(
        noCaptureTimeHeldUntil.isPresent() && nowNanos < noCaptureTimeHeldUntil.getAsLong());
  }

  /** What the no-capture-time alert says: on Visor, where to look. */
  private static String noCaptureTimeText(CameraConfig config) {
    String said =
        "Camera " + possessive(config.name()) + " images arrive without a capture time, left out";
    if (config.software() instanceof CameraConfig.Software.Visor) {
      return said
          + ": its coprocessor's clock isn't synced to the robot's (chronyc tracking on the"
          + " coprocessor, docs/visor.md)";
    }
    return said;
  }

  /**
   * Logs how far this loop's tags landed from where they really are, on average, and the same since
   * the robot started. A loop without a tag the field has logs neither, so each stays as it was.
   */
  private void logTagResidual() {
    if (loopResidualCount == 0) {
      return;
    }
    Logger.recordOutput(tagResidualMetersKey, loopResidualSumMeters / loopResidualCount);
    residualSumMeters += loopResidualSumMeters;
    residualCount += loopResidualCount;
    Logger.recordOutput(meanTagResidualMetersKey, meanTagResidualMeters());
  }

  /**
   * Counts and logs what localization's update made of this camera's measurements, and this loop's
   * tags by what became of them: {@code AcceptedTags} and {@code RejectedTags}, where the field's
   * layout has them, and beside each refused one why ({@code RejectedVerdicts}, in the same order).
   *
   * <p>An image that was refused before it became a measurement is passed over here. It was logged
   * as it was refused ({@link #leftOut}).
   *
   * @param agreeMeters an accepted measurement this close to the estimate agrees with it
   * @param field where the field's layout has each tag, by its ID
   * @param measurementVerdicts whether the localizer says what became of each measurement
   */
  void record(
      List<FrameVerdict> frames,
      double agreeMeters,
      IntFunction<Optional<Pose3d>> field,
      MeasurementVerdicts measurementVerdicts) {
    List<String> weighed = new ArrayList<>();
    for (FrameVerdict frame : frames) {
      boolean mine = frame.camera().equals(name());
      if (!mine) {
        continue;
      }
      images[frame.verdict().ordinal()]++;
      Optional<FrameVerdict.Measured> measured = frame.measured();
      if (measured.isEmpty()) {
        continue;
      }
      weighed.add(frame.described());
      sortItsTags(frame, field);
      tally(frame, measured.get(), agreeMeters);
    }
    Logger.recordOutput(measurementVerdictsKey, weighed.toArray(String[]::new));
    if (measurementVerdicts == MeasurementVerdicts.NOT_REPORTED) {
      // No count of accepted or rejected is known, so neither is logged as if it were.
      Logger.recordOutput(givenKey, given);
    } else {
      Logger.recordOutput(acceptedKey, accepted);
      Logger.recordOutput(rejectedKey, rejected);
    }
    acceptedArray = acceptedTags.toArray(sized(acceptedArray, acceptedTags.size()));
    rejectedArray = rejectedTags.toArray(sized(rejectedArray, rejectedTags.size()));
    rejectedVerdictsArray =
        rejectedVerdicts.toArray(sized(rejectedVerdictsArray, rejectedVerdicts.size()));
    Logger.recordOutput(acceptedTagsKey, acceptedArray);
    Logger.recordOutput(rejectedTagsKey, rejectedArray);
    Logger.recordOutput(rejectedVerdictsKey, rejectedVerdictsArray);
    sightings.log(frames);
    logImages();
  }

  /**
   * Logs what the default localizer's estimator has learned of this camera: how far it takes the
   * camera to be turned on its mount ({@code LearnedTurnDegrees}), how late it takes its images to
   * be stamped ({@code LateSeconds}), and how far its watch has the camera's measurements in doubt
   * for landing to one side of the others' ({@code WatchDoubtMeters}), which is why a camera can
   * count for less than another. NaN for a camera it hasn't learned of yet.
   */
  void logWhatTheEstimatorLearned(DefaultLocalizer estimator) {
    Logger.recordOutput(learnedTurnDegreesKey, estimator.cameraTurnDegrees(name()));
    Logger.recordOutput(lateSecondsKey, estimator.cameraLateSeconds(name()));
    Logger.recordOutput(watchDoubtMetersKey, estimator.cameraWatchDoubtMeters(name()));
  }

  /**
   * Logs how many of the camera's images came to each verdict, under {@code
   * Vision/<name>/Images/<VERDICT>}: each count when it changes, and all of them the first time.
   * They're plain counts, so what was thrown out and why is read straight off the log.
   */
  private void logImages() {
    for (int verdict = 0; verdict < images.length; verdict++) {
      if (images[verdict] != imagesLogged[verdict]) {
        Logger.recordOutput(imagesKeys[verdict], images[verdict]);
        imagesLogged[verdict] = images[verdict];
      }
    }
  }

  /**
   * Sorts a measurement's tags into the accepted and the refused, each where the field's layout has
   * it. A tag the field doesn't have has no place to draw a line to, and is left out.
   */
  private void sortItsTags(FrameVerdict frame, IntFunction<Optional<Pose3d>> field) {
    for (TagVerdict seen : frame.tags()) {
      Optional<Pose3d> onTheField = field.apply(seen.tag());
      if (onTheField.isEmpty()) {
        continue;
      }
      if (seen.verdict() == Verdict.GIVEN) {
        // Nothing says whether it moved the estimate, so it is drawn as neither.
        continue;
      }
      if (seen.accepted()) {
        acceptedTags.add(onTheField.get());
      } else {
        rejectedTags.add(onTheField.get());
        rejectedVerdicts.add(seen.described());
      }
    }
  }

  /**
   * Counts a measurement as accepted or rejected. An accepted one that was close to the estimate
   * also marks this camera as agreeing with it, as of when its image was taken.
   */
  private void tally(FrameVerdict frame, FrameVerdict.Measured measured, double agreeMeters) {
    if (frame.verdict() == Verdict.GIVEN) {
      given++;
      return;
    }
    if (!frame.accepted()) {
      rejected++;
      return;
    }
    accepted++;
    if (measured.innovationMeters() <= agreeMeters) {
      long latest = frame.captureNanos();
      if (lastAgreedNanos.isPresent()) {
        latest = Math.max(lastAgreedNanos.getAsLong(), latest);
      }
      lastAgreedNanos = OptionalLong.of(latest);
    }
  }

  /**
   * The array to fill with this many elements: the one given while it has the length, and a new one
   * otherwise. AdvantageKit copies what it logs, so an array can be filled again next loop.
   */
  private static <T> T[] sized(T[] array, int length) {
    if (array.length == length) {
      return array;
    }
    return Arrays.copyOf(array, length);
  }

  // ---- What the camera has seen, for Vision, the pit and the tests ----

  /** What became of each of this loop's images, in words, as {@code Verdicts} logs them. */
  List<String> verdicts() {
    return verdicts;
  }

  /** This loop's tags that went into accepted measurements, where the field's layout has them. */
  List<Pose3d> acceptedTags() {
    return acceptedTags;
  }

  /** This loop's refused tags, where the field's layout has them. */
  List<Pose3d> rejectedTags() {
    return rejectedTags;
  }

  /** Why each of this loop's refused tags was refused, in the same order. */
  List<String> rejectedVerdicts() {
    return rejectedVerdicts;
  }

  /**
   * How far, on average since the robot started, this camera's tags land from where they really
   * are. Not a number until the camera has seen a tag, as {@link Vision#meanTagResidualMeters} has
   * it.
   */
  double meanTagResidualMeters() {
    if (residualCount == 0) {
      return Double.NaN;
    }
    return residualSumMeters / residualCount;
  }

  /**
   * Whether the estimator accepted a measurement from this camera, agreeing closely with the
   * estimate, whose image was taken at this time or later.
   */
  boolean agreedSince(long nanos) {
    return lastAgreedNanos.isPresent() && lastAgreedNanos.getAsLong() >= nanos;
  }

  boolean isConnected() {
    return inputs.connected;
  }

  /** Its frame rate, drops and detection time lately, and its warnings this session. */
  CameraHealth health() {
    return health.health();
  }

  int accepted() {
    return accepted;
  }

  /** What became of its latest image, in words, as its verdict said it; empty before the first. */
  Optional<String> latestVerdict() {
    return Optional.ofNullable(latestVerdict);
  }

  /**
   * How its images with tags fared, since the robot started, in words: "used 0 of 412 images with
   * tags (398: a tag's height is off)", the reason most of the rest weren't used with its count.
   */
  String imagesUsedWords() {
    String said = "used " + imagesUsed() + " of " + imagesWithTags() + " images with tags";
    Optional<Verdict> leading = leadingRefusal();
    if (leading.isEmpty()) {
      return said;
    }
    return said + " (" + images(leading.get()) + ": " + leading.get().words() + ")";
  }

  /** How many of its images came to a verdict, since the robot started. */
  long images(Verdict verdict) {
    return images[verdict.ordinal()];
  }

  /**
   * How many of its images saw any tag, solved or not, since the robot started: every image but
   * those that saw none.
   */
  long imagesWithTags() {
    long all = 0;
    for (long count : images) {
      all += count;
    }
    return all - images[Verdict.NO_TAGS.ordinal()];
  }

  /** How many of its images the localizer used: accepted, or given to one that doesn't say. */
  long imagesUsed() {
    return images[Verdict.ACCEPTED.ordinal()] + images[Verdict.GIVEN.ordinal()];
  }

  /**
   * Why most of its images with tags weren't used, or empty while every one was. The first verdict
   * in localization's list wins a tie.
   */
  Optional<Verdict> leadingRefusal() {
    Verdict leading = null;
    for (Verdict verdict : VERDICTS) {
      boolean used = verdict == Verdict.ACCEPTED || verdict == Verdict.GIVEN;
      if (used || verdict == Verdict.NO_TAGS || images[verdict.ordinal()] == 0) {
        continue;
      }
      if (leading == null || images[verdict.ordinal()] > images[leading.ordinal()]) {
        leading = verdict;
      }
    }
    return Optional.ofNullable(leading);
  }

  int rejected() {
    return rejected;
  }

  /** How many of its measurements went to a localizer that doesn't report what became of them. */
  int given() {
    return given;
  }

  /** How many of the camera's images were left out for arriving out of step. */
  int leftOutOfStep() {
    return leftOutOfStep;
  }

  boolean isDisconnectedAlertActive() {
    return disconnected.get();
  }

  /** The no-capture-time alert, while it's up. */
  Optional<String> noCaptureTimeAlert() {
    return noCaptureTime.get() ? Optional.of(noCaptureTime.getText()) : Optional.empty();
  }

  boolean isUnreadableAlertActive() {
    return unreadable.get();
  }

  /** The no-lens-calibration alert, while it's up. */
  Optional<String> noLensAlert() {
    return noLens.get() ? Optional.of(noLens.getText()) : Optional.empty();
  }

  boolean isOutOfCalibrationAlertActive() {
    return outOfCalibration.get();
  }

  String outOfCalibrationAlertText() {
    return outOfCalibration.getText();
  }

  boolean isOutOfStepAlertActive() {
    return outOfStep.get();
  }

  String outOfStepAlertText() {
    return outOfStep.getText();
  }

  @Override
  public void close() {
    io.close();
    disconnected.close();
    unreadable.close();
    outOfCalibration.close();
    outOfStep.close();
    noCaptureTime.close();
    noLens.close();
    TunableRegistry.remove(key + "/enabled");
  }
}
