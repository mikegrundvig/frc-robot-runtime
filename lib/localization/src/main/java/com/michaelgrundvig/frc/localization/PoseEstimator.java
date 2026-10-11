package com.michaelgrundvig.frc.localization;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.TreeSet;
import java.util.function.Predicate;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;

/**
 * Where the robot is: odometry, corrected by camera measurements, with the gyro owning heading.
 *
 * <p>The state is the robot's position and a heading offset: the field heading is the gyro's
 * reading plus the offset, so the gyro keeps all short-term authority over heading, and cameras
 * only correct its slow drift. A 3x3 covariance holds the uncertainty of all three and how they
 * relate: an unsure heading makes every meter driven add position uncertainty sideways.
 *
 * <p>Each odometry step predicts: the wheels' motion, along the arc the gyro turned through, moves
 * the position, and the uncertainty grows with the distance driven, the turning, and time, and with
 * whatever makes the wheels suspect: their disagreement with each other, the IMU's with them, a
 * tilt that lifts them, a collision (see {@link DriveConditions}). A camera measurement is applied
 * at the moment its image was taken, however late it arrives: the estimator keeps {@link
 * EstimatorConfig#historySeconds} of steps, rewinds to the measurement, applies it, and replays the
 * steps since, so measurements give the same result in any order of arrival. Each is gated against
 * the estimate first, and again whenever a replay applies it anew: one a later-arriving, earlier
 * image rules out is taken back, as it would have been rejected had the two come in order. When
 * several rejected ones agree with each other, odometry was the one that was wrong (a collision, a
 * spin), and the estimate recovers to them: to their heading first, if that is what they agree is
 * wrong (a bad heading reset, a robot placed backwards), since every position worked out with a
 * wrong heading is wrong too.
 *
 * <p>Its uncertainty is meant to be honest, since gates, χ² tests, and any bound on the error trust
 * it: the error should go past 3.72 times the spread it claims no more than once in a thousand
 * loops, which the consistency study checks in the simulator (docs/testing.md). So errors that
 * repeat are never averaged away as if each were new: a position a camera solved with the gyro's
 * heading carries the heading's doubt (and turns it only if {@link
 * EstimatorConfig#headingCorrection} lets every fix), the gyro's scale error is the same every
 * turn, each camera's turn and shift on its mount are in every image it takes, and each tag's
 * placement is in every image any camera takes of it ({@link PoseFilter} considers them). Each
 * camera is told apart by its name, and each tag by its number.
 *
 * <p>Its parts: {@link OdometryHistory} keeps the steps, {@link PoseFilter} does the filter's math,
 * and {@link Recovery} judges when the cameras agree against odometry. This class feeds them and
 * keeps the books.
 *
 * <p>Pure logic: no clock, no hardware, no static state. Every input comes in through a method,
 * stamped with its capture time, so a replay reproduces every decision. Use it from one thread.
 */
public final class PoseEstimator {
  private static final long VELOCITY_WINDOW_NANOS = 100_000_000L;

  private final EstimatorConfig config;
  private final Predicate<Translation2d> onField;
  private final double translationVariancePerMeter;
  private final double translationVariancePerRadian;
  private final double headingVariancePerSecond;
  private final double headingNoiseVariancePerRadian;
  private final double wheelHeadingVariancePerRadian;
  private final double turnVariance;
  private final double shiftVariance;
  private final long historyNanos;
  private final long futureToleranceNanos;
  private final long visionTimeoutNanos;
  private final double tiltStartRadians;
  private final double tiltZeroRadians;
  private final double collisionVariance;
  private final long collisionWindowNanos;
  private final double handlingVariancePerSecond;
  private final long disagreementWindowNanos;

  private final OdometryHistory history;
  private final PoseFilter filter;
  private final Recovery recovery;

  /** Each camera's place among the errors the filter keeps apart, by name, as cameras appear. */
  private final Map<String, Integer> cameraSlots = new LinkedHashMap<>();

  /**
   * Each camera's doubt about its mount, and the distance it was judged at, to apply next update.
   */
  private final Map<String, double[]> mountDoubts = new HashMap<>();

  /** The cameras that came after every place was taken, in the order they came. */
  private final Set<String> camerasWithoutPlaces = new LinkedHashSet<>();

  /** Each of the field's tags' place among the errors the filter keeps apart, by ID. */
  private final Map<Integer, Integer> tagSlots = new HashMap<>();

  /** The tags measurements named that the field doesn't have, in the order they came. */
  private final Set<Integer> tagsWithoutPlaces = new LinkedHashSet<>();

  /** A measurement's tags' places, for the filter: room for this many, grown as needed. */
  private int[] measuredTags = new int[8];

  // The IMUs' readings, unwrapped into one continuous yaw.
  private boolean started;
  private double yaw;
  private double lastImuYaw;
  private YawSource lastSource = YawSource.WHEELS;

  // What the wheels are going through, and the doubt it adds up to.
  private DriveConditions conditions = DriveConditions.NORMAL;
  private double suspectTravel;

  /** The doubt of a collision felt since the last step, for the next to take. */
  private double collisionPendingVariance;

  private long collisionUntilNanos = Long.MIN_VALUE;

  private PoseReset initial;
  private boolean placed;
  private long lastResetNanos = Long.MIN_VALUE;

  /**
   * Whether the heading is a driver's reset no camera has backed yet: a camera's own heading
   * agreeing with it, or the cameras' headings turning it. Until then a position solved with it
   * can't vote to move the estimate: wrong, it puts every such position off alike, and they agree.
   */
  private boolean headingInDoubt;

  private final double agreementRadians;

  private final List<VisionMeasurement> pending = new ArrayList<>();
  private final List<CameraCorrection> applied = new ArrayList<>();
  private final List<Agreement> agreements = new ArrayList<>();
  private long lastAcceptedNanos = Long.MIN_VALUE;
  private int consecutiveRejections;
  private int acceptedCount;
  private int rejectedCount;
  private int recoveries;
  private double lastInnovationMeters = Double.NaN;

  /** A camera's measurement the history applies at its time. */
  private record CameraCorrection(long time, VisionMeasurement measurement) {}

  /**
   * A correction to apply during a replay: a fresh one, or one applied before. Both are gated
   * against the estimate as the replay has it: one applied before that an earlier image, new this
   * update, now rules out is taken back, as it would have been rejected had the two come in order.
   */
  private record Event(CameraCorrection correction, boolean fresh) {
    long time() {
      return correction.time();
    }
  }

  /** An accepted measurement's distance from the estimate, for the health's agreement. */
  private record Agreement(long time, double meters) {}

  /**
   * @param config how to weigh the sources
   * @param onField whether a point is on the field: measurements off it are rejected
   * @param tags the field's tags: each one's placement is kept apart, the same for every camera
   *     that sees it and every time it's seen. A measurement from any other counts its placement as
   *     new in each image ({@link #tagsWithoutPlaces})
   * @param initial where the robot is at the start, and how sure that is; for a placeholder (the
   *     robot not placed yet) give a large uncertainty, and the first camera measurement wins
   */
  public PoseEstimator(
      EstimatorConfig config,
      Predicate<Translation2d> onField,
      Collection<Integer> tags,
      PoseReset initial) {
    this.config = config;
    this.onField = onField;
    this.initial = initial;
    translationVariancePerMeter = square(config.translationStdDevPerMeter());
    translationVariancePerRadian = square(config.translationStdDevPerRadian());
    headingVariancePerSecond = square(Math.toRadians(config.headingDriftDegreesPerMinute())) / 60;
    headingNoiseVariancePerRadian =
        square(Math.toRadians(config.headingNoiseDegreesPerTurn())) / (2 * Math.PI);
    turnVariance = square(Math.toRadians(config.cameraTurnStdDevDegrees()));
    shiftVariance = square(config.cameraShiftStdDevMeters());
    wheelHeadingVariancePerRadian =
        square(Math.toRadians(config.wheelHeadingStdDevDegreesPerTurn())) / (2 * Math.PI);
    historyNanos = Timestamps.fromSeconds(config.historySeconds());
    futureToleranceNanos = Timestamps.fromSeconds(config.futureToleranceSeconds());
    visionTimeoutNanos = Timestamps.fromSeconds(config.visionTimeoutSeconds());
    tiltStartRadians = Math.toRadians(config.tiltStartDegrees());
    tiltZeroRadians = Math.toRadians(config.tiltZeroDegrees());
    collisionVariance = square(config.collisionStdDevMeters());
    collisionWindowNanos = Timestamps.fromSeconds(config.collisionWindowSeconds());
    handlingVariancePerSecond = square(config.handlingStdDevPerSqrtSecond());
    disagreementWindowNanos = Timestamps.fromSeconds(config.disagreementWindowSeconds());
    agreementRadians = Math.toRadians(config.recoveryAgreementDegrees());
    for (int id : new TreeSet<>(tags)) {
      tagSlots.put(id, tagSlots.size());
    }
    recovery = new Recovery(config);
    filter =
        new PoseFilter(
            new PoseFilter.Considered(
                Math.toRadians(config.headingStdDevDegreesPerTurn()) / (2 * Math.PI),
                Math.toRadians(config.backupHeadingStdDevDegreesPerTurn()) / (2 * Math.PI),
                Math.toRadians(config.cameraTurnStdDevDegrees()),
                config.cameraShiftStdDevMeters(),
                Math.toRadians(config.cameraShiftStdDevDegrees()),
                config.tagPlacementStdDevMeters(),
                config.cameraShiftCorrelationMeters(),
                Math.toRadians(config.cameraShiftCorrelationDegrees())),
            tagSlots.size(),
            config.headingCorrection() == EstimatorConfig.HeadingCorrection.EVERY_FIX);
    // Room for the history at up to 1 kHz of odometry.
    history =
        new OdometryHistory(
            Math.max(16, (int) Math.ceil(config.historySeconds() * 1000) + 1),
            filter.uncertainty());
    filter.set(initial, yaw);
  }

  /**
   * One odometry step: how the robot moved since the previous step, ending at this time, and what
   * an IMU read then. Steps come in time order, at whatever rate odometry runs (250 Hz on a robot);
   * a step no newer than the last is ignored. The first step only starts the clock and the IMU's
   * reference.
   *
   * <p>An IMU turns a step only when it also read the step before: a step after it was lost, came
   * back (its reading may have jumped), or took over from the other IMU (on its own zero) is turned
   * by the wheels, and the IMU's next steps carry on from its new reading.
   *
   * @param yawRadians the IMU's yaw, counterclockwise; any zero, wrapped or not. One that isn't a
   *     number counts as no reading, so one bad reading can't poison the heading.
   * @param source which IMU read it: the gyro, or the backup IMU while the gyro is lost; the wheels
   *     when neither does, and the yaw is ignored
   */
  public void addOdometry(
      long timestampNanos, BodyMotion motion, double yawRadians, YawSource source) {
    YawSource reading =
        source != YawSource.WHEELS && Double.isFinite(yawRadians) ? source : YawSource.WHEELS;
    if (!started) {
      started = true;
      yaw = reading != YawSource.WHEELS ? yawRadians : 0;
      lastImuYaw = yaw;
      lastSource = reading;
      history.start(timestampNanos, reading, yaw);
      filter.set(initial, yaw);
      history.store(OdometryHistory.OLDEST_STEP, filter);
      lastResetNanos = timestampNanos;
      return;
    }
    long previousTime = history.newestTime();
    if (timestampNanos <= previousTime) {
      return;
    }
    // An IMU owns rotation, from its second reading on. Lost, just back, or just switched to, the
    // wheels' rotation stands in for the step.
    YawSource turnedBy =
        reading != YawSource.WHEELS && reading == lastSource ? reading : YawSource.WHEELS;
    yaw +=
        turnedBy != YawSource.WHEELS
            ? MathUtil.angleModulus(yawRadians - lastImuYaw)
            : motion.dthetaRadians();
    if (reading != YawSource.WHEELS) {
      lastImuYaw = yawRadians;
    }
    lastSource = reading;

    // A tilt the ground explains still rolls on the ground: the travel counts at the slope's
    // cosine. Tilt beyond it may have lifted the wheels: their travel counts for less.
    double trust = wheelTrust(conditions.unexplainedTiltRadians());
    double across = Math.cos(conditions.groundSlopeRadians());
    double extra =
        extraVariance(
            previousTime,
            timestampNanos,
            Math.hypot(motion.dxMeters(), motion.dyMeters()),
            trust,
            motion.residualMeters());
    int step =
        history.add(
            timestampNanos,
            trust * across * motion.dxMeters(),
            trust * across * motion.dyMeters(),
            motion.residualMeters(),
            extra,
            turnedBy,
            yaw);
    advance(step, 0, 1);
    history.store(step, filter);
    prune();
  }

  /**
   * What the robot's other sensors say about its wheels, for the odometry steps added after this
   * until the next call. A new collision makes the next step uncertain by {@link
   * EstimatorConfig#collisionStdDevMeters}: the robot may have moved without its wheels seeing it.
   * Jolts within {@link EstimatorConfig#collisionWindowSeconds} of the last are the same collision.
   * A share of one (a jolt the wheels partly explain) adds that share of its spread, outside a
   * collision's window, and opens none.
   */
  public void setConditions(DriveConditions conditions) {
    this.conditions = conditions;
    if (!started) {
      return;
    }
    boolean inWindow = history.newestTime() <= collisionUntilNanos;
    if (conditions.collision()) {
      if (!inWindow) {
        collisionPendingVariance += collisionVariance;
      }
      collisionUntilNanos = history.newestTime() + collisionWindowNanos;
    } else if (!inWindow) {
      collisionPendingVariance += square(conditions.collisionShare()) * collisionVariance;
    }
  }

  /** A camera measurement, to apply at the next {@link #update}. */
  public void addVision(VisionMeasurement measurement) {
    pending.add(measurement);
  }

  /**
   * A camera's doubt about its mount ({@link MountCheck}), as of now. With {@link
   * EstimatorConfig.HeadingCorrection#HEADING_MEASUREMENTS_AND_MOUNT_DOUBT}, the next {@link
   * #update} adds it to what the camera repeats in every image: the doubt to its shift, and the
   * doubt over its tags' distance to its turn. Otherwise it's ignored here (the vision pipeline
   * widens each image by it either way).
   *
   * <p>A doubt that comes after the camera's fixes counted is taken as new from then on: what the
   * estimate already took from them stays as sure as it was, and what's left of the doubt holds the
   * camera's later fixes off. Since the mount check judges over its latest few hundred tags, the
   * doubt always comes a little after the knock.
   *
   * @param doubtMeters how far its measurements may be off from where its mount moved
   * @param distanceMeters how far its tags were, on average, when that was judged; NaN until then
   */
  public void mountDoubt(String camera, double doubtMeters, double distanceMeters) {
    if (config.headingCorrection()
        == EstimatorConfig.HeadingCorrection.HEADING_MEASUREMENTS_AND_MOUNT_DOUBT) {
      mountDoubts.put(camera, new double[] {doubtMeters, distanceMeters});
    }
  }

  /**
   * The cameras beyond the {@value PoseFilter#MAX_CAMERAS} whose repeated errors the estimator
   * keeps apart: each image's turn and shift are taken as new, so their images, averaged, grow
   * surer than they should. Empty unless a robot has more cameras than that.
   */
  public List<String> camerasWithoutPlaces() {
    return List.copyOf(camerasWithoutPlaces);
  }

  /**
   * The tags measurements have named that aren't the field's, in the order each first came: their
   * placement can't be kept apart, so each image's is taken as new, and their images, averaged,
   * grow surer than they should. Empty unless a measurement comes from a tag off the layout.
   */
  public List<Integer> tagsWithoutPlaces() {
    return List.copyOf(tagsWithoutPlaces);
  }

  /**
   * Every camera it has weighed a measurement from (or been told a mount's doubt about), in the
   * order each first came: those whose repeated errors it keeps apart, then any beyond them.
   */
  public List<String> cameras() {
    List<String> cameras = new ArrayList<>(cameraSlots.keySet());
    cameras.addAll(camerasWithoutPlaces);
    return List.copyOf(cameras);
  }

  /**
   * Applies the measurements added since the last update, each at its capture time, and decides
   * whether odometry needs rescuing. Call it once per loop, after adding the loop's odometry and
   * measurements.
   *
   * @return what happened to each measurement, for the log
   */
  public List<VisionDecision> update() {
    List<VisionDecision> decisions = new ArrayList<>();
    List<VisionMeasurement> usable = new ArrayList<>();
    for (VisionMeasurement measurement : pending) {
      Verdict why = whyNotUsable(measurement);
      if (why == null) {
        usable.add(measurement);
      } else {
        rejectedCount++;
        decisions.add(new VisionDecision(measurement, why, Double.NaN, Double.NaN, Double.NaN));
      }
    }
    pending.clear();
    if (!usable.isEmpty()) {
      replay(usable, decisions);
    }
    recoverIfTheCamerasAgree();
    applyMountDoubts();
    if (started) {
      long cutoff = history.newestTime() - disagreementWindowNanos;
      agreements.removeIf(agreement -> agreement.time() < cutoff);
    }
    return decisions;
  }

  /**
   * Tells the estimator where the robot is, now: placed on its start, or a driver's "against the
   * wall here". History before it is dropped. A reset with no heading (infinite heading variance)
   * keeps the heading as it was.
   */
  public void reset(PoseReset reset) {
    placed = true;
    if (reset.covariance().hasHeading()) {
      headingInDoubt = false;
    }
    recovery.clear();
    consecutiveRejections = 0;
    suspectTravel = 0;
    if (!started) {
      initial = reset;
      filter.set(reset, yaw);
      return;
    }
    filter.set(reset, yaw);
    startHistoryHere();
  }

  /**
   * Whether the heading is a driver's reset no camera has backed yet ({@link #resetHeading}): an
   * image solved with it would land off by its error, and could never back it.
   */
  boolean headingInDoubt() {
    return headingInDoubt;
  }

  /**
   * Tells the estimator which way the robot faces, now, keeping where it is: a driver's "facing
   * straight downfield" when the heading has gone wrong. History before it is dropped.
   *
   * <p>The driver may be wrong: facing elsewhere, or pressing at the wrong moment. Until a camera's
   * own heading agrees with it (within {@link EstimatorConfig#recoveryAgreementDegrees}), a
   * position solved with it can't vote to move the estimate, since a wrong heading puts every such
   * position off alike; if the cameras' headings agree on another, the estimate turns to theirs.
   *
   * @param stdDevRadians how sure: how straight a driver lines the robot up by eye
   */
  public void resetHeading(Rotation2d heading, double stdDevRadians) {
    if (!(stdDevRadians > 0) || Double.isInfinite(stdDevRadians)) {
      throw new IllegalArgumentException("a heading reset's uncertainty: " + stdDevRadians);
    }
    headingInDoubt = true;
    recovery.clear();
    consecutiveRejections = 0;
    double variance = square(stdDevRadians);
    if (!started) {
      PoseCovariance before = initial.covariance();
      initial =
          new PoseReset(
              initial.timestampNanos(),
              new Pose2d(initial.pose().getTranslation(), heading),
              new PoseCovariance(before.xx(), before.xy(), before.yy(), variance),
              initial.reason());
      filter.set(initial, yaw);
      return;
    }
    filter.resetHeading(heading.getRadians() - yaw, variance);
    startHistoryHere();
  }

  /**
   * Where the robot was at a moment in the history, as the estimate now has it: for tying a camera
   * image to the robot's heading when the image was taken.
   *
   * @return empty outside the history
   */
  public Optional<Pose2d> poseAt(long timestampNanos) {
    if (!started || timestampNanos < history.oldestTime()) {
      return Optional.empty();
    }
    if (timestampNanos >= history.newestTime()) {
      return timestampNanos <= history.newestTime() + futureToleranceNanos
          ? Optional.of(pose())
          : Optional.empty();
    }
    return Optional.of(history.poseAt(timestampNanos, filter.offset()));
  }

  /** The current pose: cheap, for driving every loop. */
  public Pose2d pose() {
    return filter.pose(yaw);
  }

  /** The current estimate, with its uncertainty, velocity, mode, and health. */
  public LocalizationEstimate estimate() {
    long now = started ? history.newestTime() : 0;
    OdometryHistory.Motion motion =
        started
            ? history.motionOver(VELOCITY_WINDOW_NANOS, filter.offset())
            : OdometryHistory.Motion.NONE;
    boolean gyroConnected = !started || lastSource != YawSource.WHEELS;
    double visionAge =
        lastAcceptedNanos == Long.MIN_VALUE
            ? Double.POSITIVE_INFINITY
            : Timestamps.toSeconds(now - lastAcceptedNanos);
    double disagreement = Double.NaN;
    for (Agreement agreement : agreements) {
      if (agreement.time() >= now - disagreementWindowNanos
          && !(agreement.meters() <= disagreement)) {
        disagreement = agreement.meters();
      }
    }
    return new LocalizationEstimate(
        now,
        pose(),
        filter.covariance(),
        motion.fieldVelocity(),
        motion.yawRateRadiansPerSecond(),
        mode(
            gyroConnected,
            placed,
            consecutiveRejections >= config.lostAfterRejections(),
            lastAcceptedNanos != Long.MIN_VALUE && now - lastAcceptedNanos <= visionTimeoutNanos),
        new LocalizationEstimate.Health(
            visionAge,
            acceptedCount,
            rejectedCount,
            lastInnovationMeters,
            disagreement,
            started ? history.wheelResidual(history.newestStep()) : 0,
            suspectTravel,
            conditions.tiltRadians(),
            started && now <= collisionUntilNanos,
            recoveries,
            gyroConnected));
  }

  /**
   * What the estimate rests on, in order: a lost gyro, then not placed yet, then cameras that keep
   * disagreeing, then recent camera measurements, else odometry alone.
   *
   * @param lost whether the cameras have kept disagreeing, measurement after measurement
   * @param visionRecent whether a camera measurement was accepted recently
   */
  static LocalizationEstimate.Mode mode(
      boolean gyroConnected, boolean placed, boolean lost, boolean visionRecent) {
    if (!gyroConnected) {
      return LocalizationEstimate.Mode.IMU_FAULT;
    } else if (!placed) {
      return LocalizationEstimate.Mode.INITIALIZING;
    } else if (lost) {
      return LocalizationEstimate.Mode.LOST;
    } else if (visionRecent) {
      return LocalizationEstimate.Mode.TRACKING;
    }
    return LocalizationEstimate.Mode.DEAD_RECKONING;
  }

  private @Nullable Verdict whyNotUsable(VisionMeasurement measurement) {
    Verdict why = whyNotInHistory(measurement.timestampNanos());
    if (why != null) {
      return why;
    }
    if (!onField.test(measurement.pose().getTranslation())) {
      return Verdict.OFF_FIELD;
    }
    return null;
  }

  /** Why a measurement at a time can't be applied: before the history, or after it. */
  private @Nullable Verdict whyNotInHistory(long t) {
    if (!started) {
      return Verdict.BEFORE_ODOMETRY;
    }
    if (t <= lastResetNanos) {
      return Verdict.BEFORE_RESET;
    }
    if (t <= history.oldestTime()) {
      return Verdict.STALE;
    }
    if (t > history.newestTime() + futureToleranceNanos) {
      return Verdict.FROM_THE_FUTURE;
    }
    return null;
  }

  /**
   * Rewinds to just before the earliest new measurement and replays the history from there,
   * applying the new measurements and the ones applied before, in time order.
   */
  private void replay(List<VisionMeasurement> usable, List<VisionDecision> decisions) {
    long newest = history.newestTime();
    // A measurement stamped a little after the newest step is applied at the newest step.
    long earliest = newest;
    for (VisionMeasurement measurement : usable) {
      earliest = Math.min(earliest, measurement.timestampNanos());
    }
    int start = history.lastStepBefore(earliest);
    long startTime = history.endTime(start);
    List<Event> events = new ArrayList<>();
    for (CameraCorrection done : applied) {
      if (done.time() > startTime) {
        events.add(new Event(done, false));
      }
    }
    for (VisionMeasurement measurement : usable) {
      events.add(
          new Event(
              new CameraCorrection(Math.min(measurement.timestampNanos(), newest), measurement),
              true));
    }
    events.sort(Comparator.comparingLong(Event::time));
    applied.removeIf(done -> done.time() > startTime);

    history.restore(start, filter);
    int next = 0;
    for (int j = start + 1; j < history.stepCount(); j++) {
      long before = history.endTime(j - 1);
      long after = history.endTime(j);
      double done = 0;
      while (next < events.size() && events.get(next).time() <= after) {
        Event event = events.get(next++);
        double fraction = (double) (event.time() - before) / (after - before);
        advance(j, done, fraction);
        done = fraction;
        apply(event, history.yawPartWayThrough(j, fraction), decisions);
      }
      advance(j, done, 1);
      history.store(j, filter);
    }
    // Only when the history is a single step: measurements at that step itself.
    while (next < events.size()) {
      apply(events.get(next++), history.yawAfter(history.newestStep()), decisions);
    }
    history.store(history.newestStep(), filter);
  }

  private void apply(Event event, double yawThen, List<VisionDecision> decisions) {
    applyCamera(event.correction(), event.fresh(), yawThen, decisions);
  }

  /** Gates a camera measurement, then applies it as a Kalman update. */
  private void applyCamera(
      CameraCorrection camera, boolean fresh, double yawThen, List<VisionDecision> decisions) {
    VisionMeasurement measurement = camera.measurement();
    List<Integer> ids = measurement.tagIds();
    if (measuredTags.length < ids.size()) {
      measuredTags = new int[ids.size()];
    }
    for (int i = 0; i < ids.size(); i++) {
      measuredTags[i] = tagSlot(ids.get(i));
    }
    PoseFilter.Innovation innovation =
        filter.innovation(measurement, yawThen, cameraSlot(measurement.camera()), measuredTags);
    if (innovation == null) {
      return;
    }
    if (!gateCamera(camera, fresh, innovation, decisions)) {
      return;
    }
    filter.correct(innovation);
    applied.add(camera);
  }

  /**
   * Whether a camera measurement passes the gate, keeping the books: for a rejection, fresh or
   * taken back, the counts and what recovery needs; for a fresh one accepted, the counts and the
   * agreement. One applied before that passes again was counted then.
   */
  private boolean gateCamera(
      CameraCorrection camera,
      boolean fresh,
      PoseFilter.Innovation innovation,
      List<VisionDecision> decisions) {
    VisionMeasurement measurement = camera.measurement();
    double gate =
        innovation.withHeading() ? config.gateChiSquaredPose() : config.gateChiSquaredPosition();
    double chiSquared = innovation.chiSquared();
    if (!fresh && chiSquared <= gate) {
      return true;
    }
    lastInnovationMeters = innovation.meters();
    // A camera's own heading that agrees with a driver's reset backs it.
    if (innovation.withHeading() && Math.abs(innovation.heading()) <= agreementRadians) {
      headingInDoubt = false;
    }
    // Written so a measurement that isn't a number fails the gate instead of passing it.
    if (!(chiSquared <= gate)) {
      rejectedCount++;
      consecutiveRejections++;
      boolean leansOnHeading = !measurement.lever().equals(VisionMeasurement.Lever.NONE);
      recovery.rejected(
          new Recovery.Rejected(
              camera.time(),
              innovation.x(),
              innovation.y(),
              innovation.heading(),
              innovation.positionChiSquared() <= config.gateChiSquaredPosition(),
              !(leansOnHeading && headingInDoubt)));
      decisions.add(
          new VisionDecision(measurement, Verdict.GATED, innovation.meters(), chiSquared, gate));
      return false;
    }
    acceptedCount++;
    consecutiveRejections = 0;
    recovery.clear();
    lastAcceptedNanos = Math.max(lastAcceptedNanos, camera.time());
    agreements.add(new Agreement(camera.time(), innovation.meters()));
    placed = true;
    decisions.add(
        new VisionDecision(measurement, Verdict.ACCEPTED, innovation.meters(), chiSquared, gate));
    return true;
  }

  /**
   * When enough recent rejected measurements agree with each other, the cameras are right and
   * odometry is wrong: turn to the heading they agree on, or else jump to where they put the robot,
   * with a fresh uncertainty.
   */
  private void recoverIfTheCamerasAgree() {
    if (!started) {
      return;
    }
    long newest = history.newestTime();
    if (!recovery.enoughAsOf(newest)) {
      return;
    }
    double turn = recovery.headingCorrection();
    if (!Double.isNaN(turn)) {
      filter.turn(
          turn,
          square(config.lostPositionStdDevMeters()),
          square(Math.toRadians(config.recoveryHeadingStdDevDegrees())));
      headingInDoubt = false;
      recovered();
      return;
    }
    Recovery.Move move = recovery.positionCorrection();
    if (move == null) {
      return;
    }
    filter.shift(move.by().getX(), move.by().getY(), square(config.recoveryStdDevMeters()));
    recovered();
    // As recent as the images it rests on: a recovery made later, as an older image that held it
    // back ages out of the window, mustn't make them new.
    lastAcceptedNanos = Math.max(lastAcceptedNanos, move.newestTaken());
    placed = true;
  }

  /** After a recovery: the history starts again from it, and so do the doubts. */
  private void recovered() {
    startHistoryHere();
    suspectTravel = 0;
    recoveries++;
    consecutiveRejections = 0;
    recovery.clear();
  }

  /**
   * Sets each doubted camera's turn and shift variances from its doubt, at the newest step, and
   * keeps them there for a replay to start from. A replay from before restores the variances as
   * they were then; each update sets them again.
   */
  private void applyMountDoubts() {
    if (!started || mountDoubts.isEmpty()) {
      return;
    }
    for (Map.Entry<String, double[]> doubt : mountDoubts.entrySet()) {
      int slot = cameraSlot(doubt.getKey());
      if (slot < 0) {
        continue;
      }
      double meters = doubt.getValue()[0];
      double distance = doubt.getValue()[1];
      double turn = distance > 0 ? meters / distance : 0;
      filter.setCameraVariances(slot, turnVariance + turn * turn, shiftVariance + meters * meters);
    }
    mountDoubts.clear();
    history.store(history.newestStep(), filter);
  }

  /**
   * A camera's place among the errors the filter keeps apart, given as cameras first appear; -1
   * once there are more cameras than places, and a camera without one counts each image's errors as
   * new.
   */
  private int cameraSlot(String camera) {
    Integer slot = cameraSlots.get(camera);
    if (slot != null) {
      return slot;
    }
    if (cameraSlots.size() >= PoseFilter.MAX_CAMERAS) {
      camerasWithoutPlaces.add(camera);
      return -1;
    }
    cameraSlots.put(camera, cameraSlots.size());
    return cameraSlots.size() - 1;
  }

  /**
   * A tag's place among the errors the filter keeps apart: every field tag has one; -1 for a tag
   * the field doesn't have, which counts its placement as new in each image, and is reported.
   */
  private int tagSlot(int id) {
    Integer slot = tagSlots.get(id);
    if (slot != null) {
      return slot;
    }
    tagsWithoutPlaces.add(id);
    return -1;
  }

  /** Moves the current state through part of step j: from fraction f0 of it to f1. */
  private void advance(int j, double f0, double f1) {
    double fraction = f1 - f0;
    if (fraction <= 0) {
      return;
    }
    double stepTurn = history.yawAfter(j) - history.yawAfter(j - 1);
    double stepDx = history.travelForward(j);
    double stepDy = history.travelLeft(j);
    double distance = Math.hypot(stepDx, stepDy);
    double seconds = Timestamps.toSeconds(history.endTime(j) - history.endTime(j - 1));
    filter.predict(
        history.yawAfter(j - 1) + f0 * stepTurn,
        fraction * stepTurn,
        fraction * stepDx,
        fraction * stepDy,
        fraction
            * (translationVariancePerMeter * distance
                + translationVariancePerRadian * Math.abs(stepTurn)
                + history.addedPositionVariance(j)),
        fraction
            * (headingVariancePerSecond * seconds
                + (history.turnedBy(j) == YawSource.WHEELS
                        ? wheelHeadingVariancePerRadian
                        : headingNoiseVariancePerRadian)
                    * Math.abs(stepTurn)),
        history.turnedBy(j));
  }

  /**
   * How much of the wheels' travel counts at a tilt: all of it when level, less from the start
   * angle on, none from the zero angle.
   */
  private double wheelTrust(double tiltRadians) {
    return 1
        - Math.clamp(
            (tiltRadians - tiltStartRadians) / (tiltZeroRadians - tiltStartRadians), 0.0, 1.0);
  }

  /**
   * A step's position variance beyond the base terms. Its suspect travel (the share of the wheels'
   * travel the tilt left out, how far their disagreement with each other may have put it off, and
   * how far the IMU says they slid) adds to the recent suspect travel before squaring, since slip
   * keeps its direction for a while; a new collision adds its variance; and a disabled robot may be
   * carried.
   */
  private double extraVariance(long from, long to, double distance, double trust, double residual) {
    double seconds = Timestamps.toSeconds(to - from);
    double carried = suspectTravel * Math.exp(-seconds / config.slipCorrelationSeconds());
    suspectTravel =
        carried
            + (1 - trust) * distance
            + config.residualScale() * residual
            + conditions.slipMetersPerSecond() * seconds;
    double variance = square(suspectTravel) - square(carried);
    if (conditions.disabled()) {
      variance += handlingVariancePerSecond * seconds;
    }
    variance += collisionPendingVariance;
    collisionPendingVariance = 0;
    return variance;
  }

  /** Drops the history: the newest step, with the current state, becomes its start. */
  private void startHistoryHere() {
    history.keepOnlyNewest();
    history.store(OdometryHistory.OLDEST_STEP, filter);
    applied.clear();
    agreements.clear();
    lastResetNanos = history.oldestTime();
  }

  /** Drops steps older than the history keeps, and the measurements already inside the oldest. */
  private void prune() {
    history.dropStepsNotNeededSince(history.newestTime() - historyNanos);
    long oldestTime = history.oldestTime();
    applied.removeIf(done -> done.time() <= oldestTime);
  }

  private static double square(double value) {
    return value * value;
  }
}
