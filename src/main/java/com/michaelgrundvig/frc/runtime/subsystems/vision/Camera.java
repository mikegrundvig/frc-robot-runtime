package com.michaelgrundvig.frc.runtime.subsystems.vision;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.localization.FrameVerdict;
import com.michaelgrundvig.frc.localization.MountCheck;
import com.michaelgrundvig.frc.localization.TagVerdict;
import com.michaelgrundvig.frc.localization.Timestamps;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.OptionalLong;
import java.util.function.IntFunction;
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
 *   <li>{@link #read} reads the camera and starts the loop's record.
 *   <li>{@link #checkMount} judges the mount by the tags seen in earlier loops, so that this loop's
 *       measurements can be doubted for it.
 *   <li>Each image is taken in turn: it is left out ({@link #leftOut}) or given to localization
 *       ({@link #outcome}), and its tags are recorded ({@link #saw}, {@link #landed}, {@link
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
 */
final class Camera implements AutoCloseable {
  /** The out-of-step alert stays up this long after the last image out of step. */
  private static final long OUT_OF_STEP_HOLD_NANOS = 2_000_000_000L;

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
  private final String outcomesKey;
  private final String leftOutOfStepKey;
  private final String latencySecondsKey;
  private final String seenTagsKey;
  private final String mountOffsetMetersKey;
  private final String mountDoubtMetersKey;
  private final String tagResidualMetersKey;
  private final String meanTagResidualMetersKey;
  private final String decisionsKey;
  private final String acceptedKey;
  private final String rejectedKey;
  private final String acceptedTagsKey;
  private final String rejectedTagsKey;
  private final String rejectedReasonsKey;

  // ---- The alerts ----

  private final Alert disconnected;
  private final Alert unreadable;
  private final Alert outOfCalibration;
  private final Alert outOfStep;

  // ---- Images that arrive out of step ----

  /** The out-of-step alert is up until this time. It is empty while no image has been late. */
  private OptionalLong outOfStepAlertHeldUntil = OptionalLong.empty();

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

  /** The distances from each tag seen to where the tag really is, added up, and how many. */
  private double residualSumMeters;

  private int residualCount;

  // ---- This loop's record ----

  /** What became of each of this loop's images, in words. */
  private final List<String> outcomes = new ArrayList<>();

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
  private final List<String> rejectedReasons = new ArrayList<>();

  /** The arrays those three were last logged in, which are used again while their lengths hold. */
  private Pose3d[] acceptedArray = {};

  private Pose3d[] rejectedArray = {};
  private String[] reasonsArray = {};

  /**
   * @param mountSightings how many of its latest tags the camera's mount is judged by
   */
  Camera(CameraConfig config, CameraIO io, int mountSightings) {
    this.config = config;
    this.io = io;
    mount = new MountCheck(mountSightings);
    String name = config.name();
    key = "Vision/" + name;
    outcomesKey = key + "/Outcomes";
    leftOutOfStepKey = key + "/LeftOutOfStep";
    latencySecondsKey = key + "/LatencySeconds";
    seenTagsKey = key + "/SeenTags";
    mountOffsetMetersKey = key + "/MountOffsetMeters";
    mountDoubtMetersKey = key + "/MountDoubtMeters";
    tagResidualMetersKey = key + "/TagResidualMeters";
    meanTagResidualMetersKey = key + "/MeanTagResidualMeters";
    decisionsKey = key + "/Decisions";
    acceptedKey = key + "/Accepted";
    rejectedKey = key + "/Rejected";
    acceptedTagsKey = key + "/AcceptedTags";
    rejectedTagsKey = key + "/RejectedTags";
    rejectedReasonsKey = key + "/RejectedReasons";
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
    TunableRegistry.publish(key + "/enabled", enabled);
  }

  String name() {
    return config.name();
  }

  // ---- Reading the camera ----

  /** Reads the camera, shows whether it can be reached, and starts this loop's record. */
  void read() {
    readInputs();
    Logger.processInputs(key, inputs);
    showWhetherItCanBeRead();
    startLoopRecord();
  }

  /**
   * Reads the camera's IO. Nothing the IO throws gets past here.
   *
   * <p>PhotonLib throws on purpose when the coprocessor runs a different PhotonVision version than
   * the robot's library, and again every few seconds. Let through, that would end the robot program
   * each time. Instead the camera reads as unreadable, with the reason, and the robot drives on its
   * other sensors.
   */
  private void readInputs() {
    try {
      io.updateInputs(inputs);
      inputs.error = "";
    } catch (RuntimeException thrown) {
      inputs.connected = false;
      inputs.frames = new CameraIO.Frame[] {};
      inputs.tags = new CameraIO.SeenTag[] {};
      inputs.skippedFrames = 0;
      inputs.error = said(thrown);
    }
  }

  /** What an exception says, or the whole of it when it says nothing. */
  private static String said(RuntimeException thrown) {
    String message = thrown.getMessage();
    if (message == null) {
      return thrown.toString();
    }
    return message.strip();
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
    outcomes.clear();
    latencies.clear();
    seenTags.clear();
    acceptedTags.clear();
    rejectedTags.clear();
    rejectedReasons.clear();
    loopResidualSumMeters = 0;
    loopResidualCount = 0;
    if (inputs.skippedFrames > 0) {
      outcomes.add(inputs.skippedFrames + " image(s) without a capture time, left out");
    }
  }

  // ---- One image ----

  /**
   * Whether an image is new. A camera can hand over the same image twice, and an image no newer
   * than the newest already seen is taken as one of those.
   */
  boolean isNew(long takenNanos) {
    boolean seenBefore = newestImageNanos.isPresent() && takenNanos <= newestImageNanos.getAsLong();
    if (seenBefore) {
      return false;
    }
    newestImageNanos = OptionalLong.of(takenNanos);
    return true;
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
    double latestSeconds = config.sensor().latency().in(Seconds) + marginSeconds;
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
  void outcome(String outcome) {
    outcomes.add(outcome);
  }

  /**
   * One of this loop's images was left out before the estimator saw it. Why is its outcome, and its
   * tags are refused for the same reason.
   *
   * @param tags the image's tags, where the field's layout has them
   */
  void leftOut(List<Pose3d> tags, String why) {
    outcome(why);
    for (Pose3d tag : tags) {
      rejectedTags.add(tag);
      rejectedReasons.add(why);
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
    Logger.recordOutput(outcomesKey, outcomes.toArray(String[]::new));
    Logger.recordOutput(leftOutOfStepKey, leftOutOfStep);
    Logger.recordOutput(
        latencySecondsKey, latencies.stream().mapToDouble(Double::doubleValue).toArray());
    Logger.recordOutput(seenTagsKey, seenTags.toArray(Pose3d[]::new));
    Logger.recordOutput(mountOffsetMetersKey, mount.offsetMeters());
    Logger.recordOutput(mountDoubtMetersKey, mountDoubt);
    logTagResidual();
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
   * layout has them, and beside each refused one why ({@code RejectedReasons}, in the same order).
   *
   * <p>An image that was refused before it became a measurement is passed over here. It was logged
   * as it was refused ({@link #leftOut}).
   *
   * @param agreeMeters an accepted measurement this close to the estimate agrees with it
   * @param field where the field's layout has each tag, by its ID
   */
  void record(List<FrameVerdict> frames, double agreeMeters, IntFunction<Optional<Pose3d>> field) {
    List<String> decisions = new ArrayList<>();
    for (FrameVerdict frame : frames) {
      boolean mine = frame.camera().equals(name());
      Optional<FrameVerdict.Measured> measured = frame.measured();
      if (!mine || measured.isEmpty()) {
        continue;
      }
      decisions.add(frame.reason());
      sortItsTags(frame, field);
      tally(frame, measured.get(), agreeMeters);
    }
    Logger.recordOutput(decisionsKey, decisions.toArray(String[]::new));
    Logger.recordOutput(acceptedKey, accepted);
    Logger.recordOutput(rejectedKey, rejected);
    acceptedArray = acceptedTags.toArray(sized(acceptedArray, acceptedTags.size()));
    rejectedArray = rejectedTags.toArray(sized(rejectedArray, rejectedTags.size()));
    reasonsArray = rejectedReasons.toArray(sized(reasonsArray, rejectedReasons.size()));
    Logger.recordOutput(acceptedTagsKey, acceptedArray);
    Logger.recordOutput(rejectedTagsKey, rejectedArray);
    Logger.recordOutput(rejectedReasonsKey, reasonsArray);
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
      if (seen.accepted()) {
        acceptedTags.add(onTheField.get());
      } else {
        rejectedTags.add(onTheField.get());
        rejectedReasons.add(seen.reason());
      }
    }
  }

  /**
   * Counts a measurement as accepted or rejected. An accepted one that was close to the estimate
   * also marks this camera as agreeing with it, as of when its image was taken.
   */
  private void tally(FrameVerdict frame, FrameVerdict.Measured measured, double agreeMeters) {
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

  /** This loop's tags that went into accepted measurements, where the field's layout has them. */
  List<Pose3d> acceptedTags() {
    return acceptedTags;
  }

  /** This loop's refused tags, where the field's layout has them. */
  List<Pose3d> rejectedTags() {
    return rejectedTags;
  }

  /** Why each of this loop's refused tags was refused, in the same order. */
  List<String> rejectedReasons() {
    return rejectedReasons;
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

  int accepted() {
    return accepted;
  }

  int rejected() {
    return rejected;
  }

  /** How many of the camera's images were left out for arriving out of step. */
  int leftOutOfStep() {
    return leftOutOfStep;
  }

  boolean isDisconnectedAlertActive() {
    return disconnected.get();
  }

  boolean isUnreadableAlertActive() {
    return unreadable.get();
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
    TunableRegistry.remove(key + "/enabled");
  }
}
