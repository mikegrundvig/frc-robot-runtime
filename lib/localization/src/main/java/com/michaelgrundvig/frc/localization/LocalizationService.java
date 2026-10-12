package com.michaelgrundvig.frc.localization;

import com.michaelgrundvig.frc.world.config.Finding;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Terrain;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Where the robot is: localization as one service, told what the robot's sensors read and asked
 * what it makes of them. It owns the {@link PoseEstimator} and everything around it: which IMU
 * turns odometry, how far the wheels are trusted, and solving each camera image as the robot was
 * when it was taken.
 *
 * <p>Each loop, on the robot loop's thread, in this order:
 *
 * <ol>
 *   <li>{@link #drivetrain}, with every wheel reading since the last loop and both IMUs;
 *   <li>{@link #place}, if the drive team placed the robot this loop;
 *   <li>{@link #mountDoubt} for each camera, and {@link #camera} for each image that arrived,
 *       stamped when it was taken;
 *   <li>{@link #update}: the one call that applies the loop's measurements, and says what became of
 *       each.
 * </ol>
 *
 * <p>Pure, like the rest of this package: it reads no clock (each loop's time comes in a {@link
 * LoopContext}), keeps no static state, and knows nothing of the robot program, the simulator, or
 * Pitwall. Its outputs are a function of its inputs and its configuration, so a replay of a logged
 * match reproduces every one. Use it from one thread.
 *
 * <p>Independent of the drivetrain: its wheels come in as {@link OdometrySample}s, through its
 * {@link DriveKinematicsModel}, so swerve, tank, or any other localizes the same way.
 *
 * <p>Each image is solved against the field's tags through one {@link VisionPipeline}, and with its
 * camera's mount from the robot's definition, looked up by name in {@link #solve}: the two places a
 * venue's surveyed tags and a camera that rides a joint would change what an image is solved with.
 */
public final class LocalizationService {
  /**
   * On known raised ground, a tilt up to this much steeper than its slopes is still the ground's:
   * bumpers, suspension, and the IMU's mount.
   */
  private static final double GROUND_SLOPE_MARGIN = Math.toRadians(3);

  /** How far off a surface's footprint the estimate may put the robot and still be on it. */
  private static final double MAX_GROUND_REACH_METERS = 1.0;

  /** How far back "lately" reaches, for how fast the robot has turned: a top team used 0.3 s. */
  private static final double RECENT_TURNING_SECONDS = 0.3;

  /** How much of the robot's attitude is kept, for each image's capture time. */
  private static final double ATTITUDE_SECONDS = 0.5;

  /** Where the ring of recent yaw rates stands before any loop: the first loop takes place 0. */
  private static final int BEFORE_THE_FIRST_LOOP = -1;

  /** A history of loops is never shorter than this: one loop either side of a moment. */
  private static final int FEWEST_LOOPS_KEPT = 2;

  private final PoseEstimator estimator;
  private final Terrain ground;

  /** How far the wheels reach from the robot's center, in meters. */
  private final double wheelReach;

  /** Each loop's wheel readings as the estimator's steps, with the yaw at each. */
  private final OdometrySteps odometrySteps;

  /** The gyro and the backup IMU should turn alike. */
  private final ImuCrossCheck crossCheck = new ImuCrossCheck(ImuCrossCheck.Settings.DEFAULTS);

  /** The wheels against the IMU: whether they slide over the floor together. */
  private final WheelsAgainstImu wheelsAgainstImu;

  /**
   * Which IMU turns odometry this loop, and so says everything else an IMU says this loop: the
   * gyro, the backup, or neither ({@link YawSource#WHEELS}).
   */
  private YawSource imuThisLoop = YawSource.WHEELS;

  /** What that IMU read this loop: null when neither IMU turns odometry. */
  private @Nullable ImuReadings imuReadingsThisLoop;

  /** What the IMU and the robot's state said about trusting the wheels this loop. */
  private DriveConditions conditions = DriveConditions.NORMAL;

  private LocalizationEstimate estimate;

  /** What the estimator decided about each measurement in the last update. */
  private List<VisionDecision> decisions = List.of();

  // The cameras: each one's mount and the field's tags to solve its images with (the venue's, once
  // one is selected), and how far the pit has tuned them to be trusted.
  private VisionPipeline pipeline;
  private final Set<Integer> tagIds;
  private final Map<String, CameraConfig> cameras = new LinkedHashMap<>();
  private VisionConfig vision = VisionConfig.DEFAULTS;

  /** What the robot last said of each camera's mount, by the camera's name. */
  private final Map<String, MountDoubt> mountDoubts = new HashMap<>();

  /** The images refused or held since the last update, before they became measurements. */
  private final List<FrameVerdict> refused = new ArrayList<>();

  /** Half a second of the robot's attitude, for each image's capture time. */
  private final AttitudeHistory attitudes;

  /** What the robot is, for where a camera on a joint was. */
  private final RobotMath math;

  /** Half a second of the joints' positions, for where a camera on one was at its capture time. */
  private final JointHistory jointPositions;

  /** How fast the robot turned in each of the latest loops, for how fast it has turned lately. */
  private final double[] recentYawRates;

  /**
   * This loop's place among {@link #recentYawRates}, which is used as a ring: {@link
   * #BEFORE_THE_FIRST_LOOP} until a loop has come.
   */
  private int yawRateLoop = BEFORE_THE_FIRST_LOOP;

  /** Whether a loop is under way: {@link #drivetrain} has come, and {@link #update} hasn't yet. */
  private boolean inLoop;

  /**
   * How the robot moves this loop, and how sure its heading is: what this loop's images are solved
   * with. Taken as the loop's readings come in, and again if the robot is placed during the loop.
   */
  private Motion motion;

  /**
   * @param speedMetersPerSecond how fast the robot moves over the field
   * @param recentYawRateRadiansPerSecond the fastest it turned lately, either way
   * @param headingStdDevRadians how sure the estimate is of its heading
   */
  private record Motion(
      double speedMetersPerSecond,
      double recentYawRateRadiansPerSecond,
      double headingStdDevRadians) {}

  /**
   * Localization for a robot on a field: its estimator starts with the robot not placed yet, so
   * anywhere on the field, and the first camera measurement (or placing it) wins.
   *
   * @param field the field: its tags, which images are solved against; its terrain, which a tilt
   *     the ground explains and a crest crossed come from; and its edges, past which no measurement
   *     can put the robot
   * @param robot what the robot is: its wheels, its IMUs, and its cameras' mounts
   * @param kinematics how the drivetrain's wheels move the robot: {@link SwerveKinematicsModel} for
   *     the robot's swerve modules
   * @param config how the estimator weighs its sources
   * @param loopSeconds how long one robot loop is
   */
  public LocalizationService(
      FieldModel field,
      RobotDefinition robot,
      DriveKinematicsModel kinematics,
      EstimatorConfig config,
      double loopSeconds) {
    this(
        new PoseEstimator(config, field::contains, field.tags().keySet(), notPlacedYet(field)),
        kinematics,
        field,
        field.terrain(),
        robot,
        loopSeconds);
  }

  /**
   * Localization from its parts: an estimator already made (in a test, one that starts where the
   * robot stands), and a terrain other than the field's own.
   *
   * @param estimator the pose estimator, which this owns from now on
   * @param kinematics how the drivetrain's wheels move the robot
   * @param field the field whose tags images are solved against
   * @param terrain the ground the robot drives on: a tilt it explains keeps the wheels trusted
   * @param robot what the robot is: its wheels, its IMUs, and its cameras' mounts
   * @param loopSeconds how long one robot loop is
   */
  public LocalizationService(
      PoseEstimator estimator,
      DriveKinematicsModel kinematics,
      FieldModel field,
      Terrain terrain,
      RobotDefinition robot,
      double loopSeconds) {
    if (!(loopSeconds > 0) || Double.isInfinite(loopSeconds)) {
      throw new IllegalArgumentException("a loop takes some time: " + loopSeconds);
    }
    this.estimator = estimator;
    odometrySteps = new OdometrySteps(kinematics);
    double reach = 0;
    for (Translation2d wheel : robot.moduleTranslations()) {
      reach = Math.max(reach, wheel.getNorm());
    }
    wheelReach = reach;
    ground = terrain;
    // The most the tread's grip can change the robot's velocity is its friction times gravity.
    double gripMetersPerSecSq =
        robot.drivetrain().wheelCoefficientOfFriction() * ImuReadings.STANDARD_GRAVITY;
    wheelsAgainstImu =
        new WheelsAgainstImu(
            new SlipCheck(SlipCheck.Settings.DEFAULTS, reach, gripMetersPerSecSq), robot.imus());
    pipeline = new VisionPipeline(field);
    tagIds = Set.copyOf(field.tags().keySet());
    for (CameraConfig camera : robot.cameras()) {
      cameras.put(camera.name(), camera);
    }
    attitudes = new AttitudeHistory(loops(ATTITUDE_SECONDS, loopSeconds));
    math = robot.math();
    jointPositions = new JointHistory(math.rests(), loops(ATTITUDE_SECONDS, loopSeconds));
    recentYawRates = new double[loops(RECENT_TURNING_SECONDS, loopSeconds)];
    estimate = estimator.estimate();
    motion =
        new Motion(estimate.fieldVelocity().getNorm(), 0, estimate.covariance().headingStdDev());
  }

  /**
   * What localization doubts of a robot, said as it starts: treads said to grip at or past the jolt
   * the slip check calls a whole collision, where it can't tell a slide from a collision.
   */
  public static List<Finding> doubts(RobotDefinition robot) {
    double grip = robot.drivetrain().wheelCoefficientOfFriction();
    double most = SlipCheck.Settings.DEFAULTS.joltMetersPerSecSq() / ImuReadings.STANDARD_GRAVITY;
    if (grip < most) {
      return List.of();
    }
    return List.of(
        Finding.doubt(
            robot.name(),
            String.format(
                Locale.ROOT,
                "the treads' grip, %.3g, is past %.3g, above which the slip check can't tell a"
                    + " slide from a collision: a jolt up to %.3g g may be the floor's, so it's"
                    + " none, and a hit under it is seen only as a slide. Give the tread's own,"
                    + " about 1.0 worn and 1.3 new, from a pull test (docs/characterization.md,"
                    + " tread friction); a path planner's friction is a tuning value, not the"
                    + " tread's",
                grip,
                most,
                grip)));
  }

  /**
   * Solves images against a venue's tags from now on: the field as surveyed, its tags where the
   * survey found them. The estimate carries on as it was; a venue changes where the tags are, not
   * where the robot is. The caller changes it only while the robot is disabled.
   *
   * @param venue the same field, with the same tags, surveyed
   * @throws IllegalArgumentException if its tags aren't the ones the estimator keeps apart
   */
  public void venue(FieldModel venue) {
    if (!venue.tags().keySet().equals(tagIds)) {
      throw new IllegalArgumentException(
          "a venue has the field's tags, surveyed: " + venue.name() + " has others");
    }
    pipeline = new VisionPipeline(venue);
  }

  /**
   * Until the robot is placed it could be anywhere: the middle of the field stands in, with an
   * uncertainty as big as the field.
   */
  private static PoseReset notPlacedYet(FieldModel field) {
    return new PoseReset(
        0,
        new Pose2d(field.length() / 2, field.width() / 2, Rotation2d.ZERO),
        PoseCovariance.fromStdDevs(field.length(), field.width(), Math.PI),
        "not placed yet");
  }

  /** How many loops a stretch of time takes, to the nearest loop, and at least two. */
  private static int loops(double seconds, double loopSeconds) {
    return Math.max(FEWEST_LOOPS_KEPT, (int) Math.round(seconds / loopSeconds));
  }

  /**
   * One loop of the drivetrain's readings: after it reads its sensors, and before the cameras.
   * Picks what turns odometry (the gyro, the backup IMU while the gyro is lost, or the wheels),
   * weighs the wheels by what the IMU says (a tilt, a jolt, the wheels sliding: {@link SlipCheck}),
   * and feeds every wheel reading to the estimator.
   */
  public void drivetrain(DrivetrainReadings readings, LoopContext loop) {
    ImuReadings gyro = readings.gyro();
    ImuReadings backup = readings.backupImu();
    OdometrySteps.Loop steps = odometrySteps.take(readings.wheels(), gyro, backup);
    // One IMU says everything this loop: the one whose yaw turns odometry.
    imuThisLoop = steps.yawFrom();
    imuReadingsThisLoop =
        switch (imuThisLoop) {
          case GYRO -> gyro;
          case BACKUP -> backup;
          case WHEELS -> null;
        };
    wheelsAgainstImu.compare(steps, imuReadingsThisLoop, readings.velocities());
    conditions = conditions(loop.enabled());
    estimator.setConditions(conditions);
    for (OdometrySteps.Step step : steps.steps()) {
      estimator.addOdometry(
          step.timestampNanos(), step.motion(), step.yawRadians(), steps.yawFrom());
    }
    checkImus(gyro, backup, loop.nowNanos());
    estimate = estimator.estimate();
    // The loop's images are solved as the robot is now. Its attitude and its turning are recorded
    // once a loop for that.
    attitudes.add(estimate.timestampNanos(), pitchRadians(), rollRadians());
    yawRateLoop = (yawRateLoop + 1) % recentYawRates.length;
    inLoop = true;
    takeMotion();
  }

  /**
   * The drive team put the robot here (on its start before the match, say), or a driver's "against
   * the wall here". After {@link #drivetrain}, so a turn made in placing it isn't counted twice.
   */
  public void place(PoseReset reset) {
    estimator.reset(reset);
    estimate = estimator.estimate();
    if (inLoop) {
      takeMotion();
    }
  }

  /**
   * The robot faces this heading, as sure as the given spread; its position stays. If that's wrong,
   * the cameras' headings will agree against it and the estimate turns back to them.
   */
  public void resetHeading(Rotation2d heading, double stdDevRadians) {
    estimator.resetHeading(heading, stdDevRadians);
    estimate = estimator.estimate();
    if (inLoop) {
      takeMotion();
    }
  }

  /**
   * How far the cameras' images are trusted, from the next image on: as committed, or as the pit
   * tuned it. Until told, {@link VisionConfig#DEFAULTS}.
   */
  public void vision(VisionConfig settings) {
    vision = settings;
  }

  /**
   * How far a camera's mount may have moved since it was calibrated ({@link MountCheck}), as of
   * now: its images from now on are widened by it, and with {@link
   * EstimatorConfig.HeadingCorrection#HEADING_MEASUREMENTS_AND_MOUNT_DOUBT} the estimator takes it
   * as part of the camera's repeated errors. Before the cameras' images, each loop.
   *
   * @param doubtMeters how far its measurements may be off from where its mount moved
   * @param distanceMeters how far its tags were, on average, when that was judged; NaN until then
   */
  public void mountDoubt(String camera, double doubtMeters, double distanceMeters) {
    estimator.mountDoubt(camera, doubtMeters, distanceMeters);
    mountDoubts.put(camera, new MountDoubt(doubtMeters, distanceMeters));
  }

  /** One camera image, from every tag it saw: see {@link #camera(TagDetections, TagAllowlist)}. */
  public VisionResult camera(TagDetections frame) {
    return camera(frame, null);
  }

  /**
   * One camera image, stamped when it was taken: solved as the robot was then ({@link #capture}),
   * widened by its camera's mount's doubt ({@link #mountDoubt}), and its measurement, if it makes
   * one, applied at that moment on the next {@link #update}.
   *
   * @param onlyTags the tags the current task trusts ({@link TagAllowlist}); null for every tag
   * @return what the image made: a measurement, or why not. The next update says it too, among its
   *     frames, as it does what became of the measurement
   * @throws IllegalArgumentException if the robot's definition has no camera by the frame's name
   */
  public VisionResult camera(TagDetections frame, @Nullable TagAllowlist onlyTags) {
    VisionResult result = solve(frame, onlyTags);
    VisionMeasurement measurement = result.measurement();
    if (measurement != null) {
      camera(measurement);
    } else {
      refused.add(FrameVerdict.refused(result));
    }
    return result;
  }

  /**
   * An image the robot holds back, from a camera being calibrated or while autonomous starts:
   * solved as {@link #camera(TagDetections, TagAllowlist)} would, never applied, and among the next
   * update's frames with why it was held, or why the pipeline refused it.
   *
   * @param why why it's held: {@link Verdict#CALIBRATING} or {@link Verdict#HELD}
   * @return what the image made: a measurement, or why not, for the log
   */
  public VisionResult hold(TagDetections frame, @Nullable TagAllowlist onlyTags, Verdict why) {
    if (why != Verdict.CALIBRATING && why != Verdict.HELD) {
      throw new IllegalArgumentException(
          "the robot holds an image back for its own reasons: " + why);
    }
    VisionResult result = solve(frame, onlyTags);
    boolean pipelineRefusedIt = result.measurement() == null;
    if (pipelineRefusedIt) {
      refused.add(FrameVerdict.refused(result));
    } else {
      refused.add(FrameVerdict.held(frame, why));
    }
    return result;
  }

  /** What an image makes, solved as the robot was when it was taken: nothing is kept. */
  private VisionResult solve(TagDetections frame, @Nullable TagAllowlist onlyTags) {
    CameraConfig camera = cameras.get(frame.camera());
    if (camera == null) {
      throw new IllegalArgumentException(
          "the robot's definition has no camera named " + frame.camera());
    }
    MountDoubt doubt = mountDoubts.get(frame.camera());
    double mountDoubtMeters = doubt == null ? 0 : doubt.meters();
    return pipeline.process(
        frame,
        camera,
        mount(camera, frame.timestampNanos()),
        capture(frame.timestampNanos()),
        vision,
        new VisionPipeline.Trust(mountDoubtMeters, onlyTags));
  }

  /**
   * A camera measurement solved elsewhere (a Limelight's own pose, say, or a test's), applied at
   * its capture time on the next {@link #update}.
   */
  public void camera(VisionMeasurement measurement) {
    estimator.addVision(measurement);
  }

  /**
   * The joints' positions as the robot read them, each loop: where a camera on a joint is when its
   * image was taken. A robot whose cameras are all on the chassis needs none; before the first,
   * each joint is at its reference.
   *
   * @param timestampNanos when they were read, on the robot's clock
   * @param positions each driven joint's position, in its units; a joint not named stays where it
   *     was
   */
  public void joints(long timestampNanos, Map<Joint, Double> positions) {
    jointPositions.add(timestampNanos, positions);
  }

  /** Where a camera was on the robot at a moment: on a joint, where the joint had carried it. */
  private Transform3d mount(CameraConfig camera, long timestampNanos) {
    if (camera.joint().isEmpty()) {
      return camera.robotToCamera();
    }
    Pose3d pose = math.cameraPose(camera, jointPositions.at(timestampNanos));
    return new Transform3d(pose.getTranslation(), pose.getRotation());
  }

  /**
   * The robot when an image was taken, as the vision pipeline solves it: its heading then, if the
   * estimate reaches back that far and the heading isn't a driver's reset no camera has backed, its
   * tilt then, and how it moves this loop. Read-only: asking changes nothing, however often or
   * early in the loop it's asked.
   */
  public CaptureContext capture(long timestampNanos) {
    Motion now = motion;
    AttitudeHistory.Attitude tilted = attitudes.at(timestampNanos);
    return new CaptureContext(
        headingToSolveWith(timestampNanos),
        now.headingStdDevRadians(),
        tilted.pitchRadians(),
        tilted.rollRadians(),
        now.speedMetersPerSecond(),
        now.recentYawRateRadiansPerSecond());
  }

  /**
   * The heading an image taken at a moment is solved with, or null when it must be solved without
   * one: the estimate does not reach back that far, or its heading is in doubt. A heading in doubt
   * solves no image, however sure the driver's reset says it is. Solved with it, an image lands off
   * by its error, and only an image solved without it can back it.
   */
  private @Nullable Rotation2d headingToSolveWith(long timestampNanos) {
    if (estimator.headingInDoubt()) {
      return null;
    }
    return estimator.poseAt(timestampNanos).map(Pose2d::getRotation).orElse(null);
  }

  /**
   * Applies the loop's measurements, each at its capture time, and says what became of each. After
   * the cameras; once a loop, closing the loop {@link #drivetrain} opened.
   */
  public LocalizationOutput update() {
    inLoop = false;
    decisions = List.copyOf(estimator.update());
    estimate = estimator.estimate();
    List<FrameVerdict> frames = new ArrayList<>(refused.size() + decisions.size());
    frames.addAll(refused);
    refused.clear();
    for (VisionDecision decision : decisions) {
      frames.add(FrameVerdict.of(decision));
    }
    return LocalizationOutput.builder().estimate(estimate).trust(trust()).frames(frames).build();
  }

  /**
   * Every measurement the last {@link #update} was given, each whole (its pose, its uncertainty,
   * its tags) with what the estimator decided about it, in the order they were applied: for a study
   * that hands another estimator the same measurements.
   */
  public List<VisionDecision> decisions() {
    return decisions;
  }

  /** Where the robot is: cheap, for driving every loop. */
  public Pose2d pose() {
    return estimator.pose();
  }

  /** Where the robot was at a moment, if the estimate reaches back that far. */
  public Optional<Pose2d> poseAt(long timestampNanos) {
    return estimator.poseAt(timestampNanos);
  }

  /** The estimate, with its uncertainty, mode, and health, as of the last step. */
  public LocalizationEstimate estimate() {
    return estimate;
  }

  /** A pose on the field in 3D: as high as the ground under it, tilted as the IMU says now. */
  public Pose3d pose3d(Pose2d pose) {
    return new Pose3d(
        pose.getX(),
        pose.getY(),
        ground.heightAt(pose.getTranslation()),
        new Rotation3d(rollRadians(), pitchRadians(), pose.getRotation().getRadians()));
  }

  /** The robot's pitch, from the IMU: zero without one. Positive is nose down. */
  public double pitchRadians() {
    ImuReadings imu = imuReadingsThisLoop;
    return imu == null ? 0 : imu.pitchRadians();
  }

  /** The robot's roll, from the IMU: zero without one. */
  public double rollRadians() {
    ImuReadings imu = imuReadingsThisLoop;
    return imu == null ? 0 : imu.rollRadians();
  }

  /** What turns odometry now: the gyro, the backup IMU, or the wheels. */
  public YawSource yawSource() {
    return odometrySteps.turnsOdometryNow();
  }

  /** What the IMU said about trusting the wheels this loop. */
  public DriveConditions conditions() {
    return conditions;
  }

  /** How far the wheels are trusted this loop, and why. */
  public WheelTrust wheels() {
    SlipCheck slipCheck = wheelsAgainstImu.slipCheck();
    return WheelTrust.builder()
        .imu(imuThisLoop)
        .conditions(conditions)
        .turnSlipMetersPerSecond(slipCheck.turnSlipMetersPerSecond())
        .speedSlipMetersPerSecond(slipCheck.speedSlipMetersPerSecond())
        .turnApartRadians(slipCheck.turnApartRadians())
        .slideMetersPerSecond(slipCheck.slideMetersPerSecond())
        .unexplainedMetersPerSecSq(slipCheck.unexplainedMetersPerSecSq())
        .accelerometerBiasMetersPerSecSq(slipCheck.biasMetersPerSecSq())
        .accelerometerLeaksGravity(slipCheck.leaksGravity())
        .build();
  }

  /** How far the gyro and the backup IMU have turned apart lately, while both report. */
  public double imuDisagreementRadians() {
    return crossCheck.disagreementRadians();
  }

  /**
   * Whether the gyro and the backup IMU disagree for long: one is failing, loose, or needs its
   * scale calibrated.
   */
  public boolean imusDisagree() {
    return crossCheck.disagrees();
  }

  /**
   * This loop's images' motion, as the estimate has it now: how fast the robot moves, the fastest
   * it turned lately (this loop's turning in its place among the latest loops'), and how sure its
   * heading is. Taken as the loop's readings come in, and again when a placement moves the estimate
   * before the loop's images.
   */
  private void takeMotion() {
    recentYawRates[yawRateLoop] = Math.abs(estimate.yawRateRadiansPerSecond());
    double recentYawRate = 0;
    for (double rate : recentYawRates) {
      recentYawRate = Math.max(recentYawRate, rate);
    }
    motion =
        new Motion(
            estimate.fieldVelocity().getNorm(),
            recentYawRate,
            estimate.covariance().headingStdDev());
  }

  /**
   * How far each source is trusted: each camera the estimator has heard from, in the order it first
   * did, then the robot's others; and the wheels.
   */
  private Trust trust() {
    Map<String, CameraTrust> trust = new LinkedHashMap<>();
    List<String> beyond = estimator.camerasWithoutPlaces();
    for (String camera : estimator.cameras()) {
      trust.put(camera, trust(camera, !beyond.contains(camera)));
    }
    for (String camera : cameras.keySet()) {
      trust.putIfAbsent(camera, trust(camera, true));
    }
    for (String camera : mountDoubts.keySet()) {
      trust.putIfAbsent(camera, trust(camera, true));
    }
    return Trust.builder()
        .cameras(trust)
        .wheels(wheels())
        .tagsWithoutPlaces(new LinkedHashSet<>(estimator.tagsWithoutPlaces()))
        .build();
  }

  /** One camera's trust: whether its errors are kept apart, and its mount's doubt if it has one. */
  private CameraTrust trust(String camera, boolean keptApart) {
    CameraTrustBuilder trust = CameraTrust.builder().keptApart(keptApart);
    MountDoubt doubt = mountDoubts.get(camera);
    if (doubt == null) {
      return trust.build();
    }
    return trust
        .mountDoubtMeters(doubt.meters())
        .mountDistanceMeters(doubt.tagDistanceMeters())
        .build();
  }

  /**
   * What the IMU says about trusting the wheels: the robot's tilt, a jolt beyond driving, the
   * wheels sliding over the floor, and whether people might be carrying it. The IMU that turns
   * odometry says (the gyro, or the backup while the gyro is lost). Without one, nothing can say
   * the robot is tilted.
   */
  private DriveConditions conditions(boolean enabled) {
    ImuReadings imu = imuReadingsThisLoop;
    double tilt = 0;
    double groundSlope = 0;
    if (imu != null) {
      tilt = WheelsAgainstImu.tiltFromLevel(imu);
      groundSlope = groundSlope(tilt);
    }
    SlipCheck slipCheck = wheelsAgainstImu.slipCheck();
    return new DriveConditions(
        tilt, groundSlope, slipCheck.collisionShare(), slipCheck.slipMetersPerSecond(), !enabled);
  }

  /**
   * How much of a tilt the ground explains. On known raised ground it is the tilt itself, up to the
   * ground's steepest slope and a margin. On flat ground it is none.
   *
   * <p>The robot counts as on raised ground when the estimate puts it within reach of a surface's
   * footprint. That reach is the wheels' own reach from the robot's center plus twice the
   * estimate's position spread, since the estimate may be off by that much, and never more than
   * {@link #MAX_GROUND_REACH_METERS}.
   */
  private double groundSlope(double tilt) {
    if (ground.surfaces().isEmpty() || tilt == 0) {
      return 0;
    }
    // The estimate as it was last worked out is still how the estimator stands. Everything that
    // changes it (odometry, a placement, the update that applies the cameras) works it out again
    // as it does, and none of those has run since. So the estimate isn't worked out again here,
    // which would be a second full estimate every loop the robot is tilted.
    double estimateMayBeOffBy = 2 * estimate.covariance().largestPositionStdDev();
    double reach = Math.min(MAX_GROUND_REACH_METERS, wheelReach + estimateMayBeOffBy);
    return ground
        .under(pose().getTranslation(), reach)
        .map(surface -> Math.min(tilt, surface.steepestSlopeRadians() + GROUND_SLOPE_MARGIN))
        .orElse(0.0);
  }

  /**
   * While both report, compares the gyro with the backup IMU: a sustained disagreement means one is
   * failing, loose, or needs its scale calibrated. It raises the question rather than silently
   * picking one.
   */
  private void checkImus(ImuReadings gyro, ImuReadings backup, long nowNanos) {
    if (gyro.usable() && backup.usable()) {
      crossCheck.update(nowNanos, gyro.yawRadians(), backup.yawRadians());
    } else {
      crossCheck.reset();
    }
  }
}
