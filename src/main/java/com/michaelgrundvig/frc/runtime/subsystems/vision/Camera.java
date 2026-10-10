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
import java.util.function.IntFunction;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.tunable.TunableBoolean;
import org.wpilib.tunable.TunableRegistry;
import org.wpilib.util.Alert;

/**
 * One of the robot's cameras, as {@link Vision} runs it: its IO and what it read this loop, its
 * alerts, the switch that leaves it out, its tallies, and the check that it still sits where its
 * configuration says. What became of this loop's images and which tags it saw are logged under
 * {@code Vision/<name>}.
 *
 * <p>A tag it saw is logged two ways, for two questions. {@code SeenTags} is where the camera put
 * the tag, through the estimate and its configured mount: its gap from the field's tag is the
 * camera's error, which the mount check judges. {@code AcceptedTags} and {@code RejectedTags} are
 * the field's own tags, by what became of the images that saw them: what each measurement rested
 * on, drawn by AdvantageScope as a line from the robot to each. The two can't be one list: an image
 * seen while the estimate didn't know where the robot was has no placed tag, and is still accepted
 * or refused.
 */
final class Camera implements AutoCloseable {
  final CameraConfig config;
  private final CameraIO io;
  final CameraIOInputsAutoLogged inputs = new CameraIOInputsAutoLogged();
  private final String key;

  // Each value's name in the log, made once rather than every loop.
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
  private final Alert disconnected;
  private final Alert unreadable;
  private final Alert outOfCalibration;
  private final Alert outOfStep;

  /** The out-of-step alert stays up this long after the last image out of step. */
  private static final long OUT_OF_STEP_HOLD_NANOS = 2_000_000_000L;

  private long outOfStepUntil = Long.MIN_VALUE;
  private int leftOutOfStep;

  /** Whether the camera still sits where its configuration says, from where its tags land. */
  private final MountCheck mount;

  /** How far this loop's measurements may be off from where the mount moved, in meters. */
  private double mountDoubt;

  /** The out-of-calibration alert, once up, comes down at this share of the tolerance. */
  private static final double ALERT_CLEARS_AT = 0.8;

  private boolean flaggedOutOfCalibration;

  // What the out-of-calibration alert says now.
  private int shownCentimeters = -1;
  private boolean shownTrustedLess;

  /** Off leaves the camera out: fogged over, knocked loose, or blinded by stage lights. */
  final TunableBoolean enabled = TunableBoolean.create(true);

  private long lastFrame = Long.MIN_VALUE;

  /**
   * When the latest image was taken that the estimator accepted from this camera, agreeing with the
   * estimate closely: this camera holds the pose others are judged against.
   */
  private long lastAgreed = Long.MIN_VALUE;

  private int accepted;
  private int rejected;
  private double residualSum;
  private int residualCount;

  // This loop: what became of each image, how late each arrived, and the tags seen, placed on the
  // field.
  private final List<String> outcomes = new ArrayList<>();
  private final List<Double> latencies = new ArrayList<>();
  private final List<Pose3d> seenTags = new ArrayList<>();
  private double loopResidualSum;
  private int loopResidualCount;

  // This loop's tags where the field's layout has them, by what became of the images that saw
  // them, and why each refused one was: AdvantageScope draws a line from the robot to each.
  private final List<Pose3d> acceptedTags = new ArrayList<>();
  private final List<Pose3d> rejectedTags = new ArrayList<>();
  private final List<String> rejectedReasons = new ArrayList<>();

  // The arrays they were last logged in, reused while their lengths hold.
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

  /**
   * Reads the camera and starts this loop's record. Nothing its IO throws gets past here: PhotonLib
   * throws on purpose when the coprocessor runs a different PhotonVision version than the robot's
   * library, and again every few seconds, which would end the robot program each time. Instead the
   * camera reads as unreadable, with the reason, and the robot drives on its other sensors.
   */
  void read() {
    try {
      io.updateInputs(inputs);
      inputs.error = "";
    } catch (RuntimeException e) {
      inputs.connected = false;
      inputs.frames = new CameraIO.Frame[] {};
      inputs.tags = new CameraIO.SeenTag[] {};
      inputs.skippedFrames = 0;
      String why = e.getMessage();
      inputs.error = why != null ? why.strip() : e.toString();
    }
    Logger.processInputs(key, inputs);
    boolean readable = inputs.error.isEmpty();
    disconnected.set(!inputs.connected && readable);
    if (!readable) {
      unreadable.setText("Camera " + name() + " can't be read: " + inputs.error);
    }
    unreadable.set(!readable);

    outcomes.clear();
    latencies.clear();
    seenTags.clear();
    acceptedTags.clear();
    rejectedTags.clear();
    rejectedReasons.clear();
    loopResidualSum = 0;
    loopResidualCount = 0;
    if (inputs.skippedFrames > 0) {
      outcomes.add(inputs.skippedFrames + " image(s) without a capture time, left out");
    }
  }

  /** Whether an image is new: a camera can hand over the same image twice. */
  boolean isNew(long timestampNanos) {
    if (timestampNanos <= lastFrame) {
      return false;
    }
    lastFrame = timestampNanos;
    return true;
  }

  /**
   * Whether an image arrived within the camera's band: no earlier than it was taken, and no later
   * than the camera's usual latency plus a margin. Outside it, the image is left out, and an alert
   * stays up for two seconds after the last one.
   *
   * <p>An image read this loop arrived after the previous loop read the camera, so it's late only
   * if it was late already then: a loop that ran long doesn't make the camera's images late.
   *
   * @param takenNanos when the image was taken, by its stamp
   * @param nowNanos this loop's time
   * @param previousLoopNanos the previous loop's time; the minimum on the first loop
   * @param marginSeconds how much later than the camera's usual latency is still in time
   * @param tags the image's tags, on the field: refused with it
   */
  boolean arrivedInTime(
      long takenNanos,
      long nowNanos,
      long previousLoopNanos,
      double marginSeconds,
      List<Pose3d> tags) {
    latencies.add(Timestamps.toSeconds(nowNanos - takenNanos));
    double latest = config.sensor().latency().in(Seconds) + marginSeconds;
    String why = null;
    if (takenNanos > nowNanos) {
      why =
          String.format(
              Locale.ROOT,
              "arrived %.0f ms before it was taken",
              Timestamps.toSeconds(takenNanos - nowNanos) * 1000);
    } else if (previousLoopNanos != Long.MIN_VALUE
        && Timestamps.toSeconds(previousLoopNanos - takenNanos) > latest) {
      why =
          String.format(
              Locale.ROOT,
              "arrived at least %.0f ms after it was taken, more than %.0f ms",
              Timestamps.toSeconds(previousLoopNanos - takenNanos) * 1000,
              latest * 1000);
    }
    if (why == null) {
      return true;
    }
    leftOut(tags, why);
    leftOutOfStep++;
    if (!(nowNanos < outOfStepUntil)) {
      // Said once as the alert goes up, not changed with every image while it's up.
      outOfStep.setText(
          "Camera "
              + possessive(name())
              + " images are out of step, and left out: one "
              + why
              + ". Are its clock and the robot's in step (time sync)? If it's just slower,"
              + " measure its latency");
    }
    outOfStepUntil = nowNanos + OUT_OF_STEP_HOLD_NANOS;
    return false;
  }

  /** What became of one of this loop's images. */
  void outcome(String outcome) {
    outcomes.add(outcome);
  }

  /**
   * One of this loop's images left out before the estimator saw it, and why: its outcome, and its
   * tags refused for that reason.
   *
   * @param tags the image's tags, where the field's layout has them
   */
  void leftOut(List<Pose3d> tags, String why) {
    outcome(why);
    refuse(tags, why);
  }

  private void refuse(List<Pose3d> tags, String why) {
    for (Pose3d tag : tags) {
      rejectedTags.add(tag);
      rejectedReasons.add(why);
    }
  }

  /**
   * Judges the camera's mount by the tags it has seen so far: whether it's out of calibration, and
   * how much less this loop's measurements are trusted for it. Once a loop, after {@link #read}.
   *
   * @param toleranceMeters how far a calibrated camera's tags land off, at most, on average
   */
  void checkMount(double toleranceMeters) {
    mountDoubt = mount.doubtMeters(toleranceMeters);
    // Up past the tolerance, and down only once back under 0.8 of it: an offset hovering at the
    // tolerance doesn't flicker the alert on and off.
    double offset = mount.offsetMeters();
    boolean off =
        flaggedOutOfCalibration
            ? offset > ALERT_CLEARS_AT * toleranceMeters
            : offset > toleranceMeters;
    flaggedOutOfCalibration = off;
    // The alert says how far off, in whole centimeters: its text changes only when that does.
    int centimeters = off ? (int) Math.round(mount.offsetMeters() * 100) : 0;
    boolean trustedLess = mountDoubt > 0;
    if (off && (centimeters != shownCentimeters || trustedLess != shownTrustedLess)) {
      outOfCalibration.setText(
          String.format(
              Locale.ROOT,
              "Camera %s is out of calibration: its tags land %d cm off%s. Recalibrate its mount"
                  + " (Calibrate the %s camera's mount)",
              name(),
              centimeters,
              trustedLess ? ", so it counts for less" : "",
              name()));
      shownCentimeters = centimeters;
      shownTrustedLess = trustedLess;
    }
    outOfCalibration.set(off);
  }

  /** How far this loop's measurements may be off from where the camera's mount moved, in meters. */
  double mountDoubtMeters() {
    return mountDoubt;
  }

  /** How far the tags its mount was judged by were, on average; NaN until judged. */
  double mountDistanceMeters() {
    return mount.distanceMeters();
  }

  /**
   * A tag this loop's image saw, placed on the field through the estimated pose and the camera's
   * configured mount, and where the tag really is, if the field has it.
   *
   * @param robot where the robot was, raised and tilted as it was
   * @param judge whether that pose is known well enough to judge the camera's mount by
   */
  void saw(Pose3d robot, Pose3d seen, @Nullable Pose3d real, boolean judge) {
    seenTags.add(seen);
    if (real != null) {
      loopResidualSum += seen.getTranslation().getDistance(real.getTranslation());
      loopResidualCount++;
      if (judge) {
        mount.add(robot, seen, real);
      }
    }
  }

  /**
   * Logs this loop's images and tags, the tag residual, this loop's and since startup, and how far
   * off the camera's mount looks; and keeps the out-of-step alert up while it's held, unless the
   * camera is turned off or gone, which says more.
   */
  void logLoop(long nowNanos) {
    if (!enabled.get() || !inputs.connected) {
      outOfStepUntil = Long.MIN_VALUE;
    }
    outOfStep.set(nowNanos < outOfStepUntil);
    Logger.recordOutput(outcomesKey, outcomes.toArray(String[]::new));
    Logger.recordOutput(leftOutOfStepKey, leftOutOfStep);
    Logger.recordOutput(
        latencySecondsKey, latencies.stream().mapToDouble(Double::doubleValue).toArray());
    Logger.recordOutput(seenTagsKey, seenTags.toArray(Pose3d[]::new));
    Logger.recordOutput(mountOffsetMetersKey, mount.offsetMeters());
    Logger.recordOutput(mountDoubtMetersKey, mountDoubt);
    if (loopResidualCount > 0) {
      Logger.recordOutput(tagResidualMetersKey, loopResidualSum / loopResidualCount);
      residualSum += loopResidualSum;
      residualCount += loopResidualCount;
      Logger.recordOutput(meanTagResidualMetersKey, meanTagResidualMeters());
    }
  }

  /**
   * Counts and logs what localization's update made of this camera's measurements (its images
   * refused before they became one were logged then), and this loop's tags by what became of them:
   * {@code AcceptedTags} and {@code RejectedTags}, where the field's layout has them, and beside
   * each refused one why ({@code RejectedReasons}, in the same order).
   *
   * @param agreeMeters an accepted measurement this close to the estimate agrees with it
   * @param field where the field's layout has each tag, by its ID
   */
  void record(List<FrameVerdict> frames, double agreeMeters, IntFunction<Optional<Pose3d>> field) {
    List<String> reasons = new ArrayList<>();
    for (FrameVerdict frame : frames) {
      // An image refused before it became a measurement was logged as it was refused.
      Optional<FrameVerdict.Measured> weighed = frame.measured();
      if (!frame.camera().equals(name()) || weighed.isEmpty()) {
        continue;
      }
      reasons.add(frame.reason());
      for (TagVerdict seen : frame.tags()) {
        field
            .apply(seen.tag())
            .ifPresent(
                tag -> {
                  if (seen.accepted()) {
                    acceptedTags.add(tag);
                  } else {
                    rejectedTags.add(tag);
                    rejectedReasons.add(seen.reason());
                  }
                });
      }
      if (frame.accepted()) {
        accepted++;
        if (weighed.get().innovationMeters() <= agreeMeters) {
          lastAgreed = Math.max(lastAgreed, frame.captureNanos());
        }
      } else {
        rejected++;
      }
    }
    Logger.recordOutput(decisionsKey, reasons.toArray(String[]::new));
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
   * The array to fill with this many elements: the one given while it has the length, else a new
   * one. AdvantageKit copies what it logs, so an array can be filled again next loop.
   */
  private static <T> T[] sized(T[] array, int length) {
    return array.length == length ? array : Arrays.copyOf(array, length);
  }

  /** This loop's tags that went into accepted measurements, where the field's layout has them. */
  List<Pose3d> acceptedTags() {
    return acceptedTags;
  }

  /** This loop's refused tags, where the field's layout has them. */
  List<Pose3d> rejectedTags() {
    return rejectedTags;
  }

  /** Why each of this loop's refused tags was, in the same order. */
  List<String> rejectedReasons() {
    return rejectedReasons;
  }

  /**
   * How far, on average since the robot started, this camera's tags land from where they really
   * are.
   *
   * @return NaN until the camera has seen a tag
   */
  double meanTagResidualMeters() {
    return residualCount == 0 ? Double.NaN : residualSum / residualCount;
  }

  /** How far off, on average, the camera's latest tags land: NaN until it has seen enough. */
  double mountOffsetMeters() {
    return mount.offsetMeters();
  }

  /**
   * When the latest image was taken that the estimator accepted from this camera within the
   * agreement it was given; none, the minimum.
   */
  long lastAgreedNanos() {
    return lastAgreed;
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
