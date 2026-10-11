package com.michaelgrundvig.frc.runtime.subsystems.vision;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Milliseconds;

import com.michaelgrundvig.frc.localization.BodyMotion;
import com.michaelgrundvig.frc.localization.DrivetrainReadings;
import com.michaelgrundvig.frc.localization.EstimatorConfig;
import com.michaelgrundvig.frc.localization.FrameVerdict;
import com.michaelgrundvig.frc.localization.ImuReadings;
import com.michaelgrundvig.frc.localization.LocalizationService;
import com.michaelgrundvig.frc.localization.LoopContext;
import com.michaelgrundvig.frc.localization.PoseCovariance;
import com.michaelgrundvig.frc.localization.PoseEstimator;
import com.michaelgrundvig.frc.localization.PoseReset;
import com.michaelgrundvig.frc.localization.SwerveKinematicsModel;
import com.michaelgrundvig.frc.localization.TagAllowlist;
import com.michaelgrundvig.frc.localization.TagVerdict;
import com.michaelgrundvig.frc.localization.VisionConfig;
import com.michaelgrundvig.frc.localization.VisionConfigBuilder;
import com.michaelgrundvig.frc.localization.YawSamples;
import com.michaelgrundvig.frc.localization.YawSource;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.world.field.Alliance;
import com.michaelgrundvig.frc.world.field.Bump;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Place;
import com.michaelgrundvig.frc.world.field.Symmetry;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfigBuilder;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;

/** The cameras' side of localization, with a scripted camera in place of PhotonVision. */
class VisionTest {
  private static final FieldModel FIELD =
      new FieldModel(
          Fields.FRC_2026_REBUILT_WELDED.loadField(),
          Symmetry.ROTATED,
          List.of(),
          List.of(),
          List.of());
  private static final Pose2d ROBOT = new Pose2d(2.5, 4.1, Rotation2d.fromDegrees(10));
  private static final long T0 = 1_000_000_000L;

  private final CameraConfig camera =
      CameraConfig.builder()
          .name("front")
          .forward(Inches.of(12))
          .left(Inches.of(0))
          .up(Inches.of(12))
          .roll(Degrees.of(0))
          .tiltUp(Degrees.of(15))
          .yaw(Degrees.of(0))
          .sensor(
              CameraConfig.Sensor.builder()
                  .widthPixels(1280)
                  .heightPixels(800)
                  .fieldOfView(Degrees.of(80))
                  .fieldOfViewAcross(CameraConfig.Across.DIAGONAL)
                  .frameRate(Hertz.of(30))
                  .latency(Milliseconds.of(30))
                  .sightRange(Meters.of(7))
                  .build())
          .noiseScale(1.0)
          .build();

  /** A camera that reports whatever the test scripts. */
  private static final class ScriptedCamera implements CameraIO {
    boolean connected = true;
    @Nullable RuntimeException failure;
    List<Frame> frames = new ArrayList<>();
    List<SeenTag> tags = new ArrayList<>();

    @Override
    public void updateInputs(CameraIOInputs inputs) {
      if (failure != null) {
        throw failure;
      }
      inputs.connected = connected;
      inputs.frames = frames.toArray(Frame[]::new);
      inputs.tags = tags.toArray(SeenTag[]::new);
      frames = new ArrayList<>();
      tags = new ArrayList<>();
    }

    /** The exact view of the blue hub's near face from the robot. */
    void see(long timestampNanos, CameraConfig mount) {
      see(timestampNanos, mount, new Pose3d(ROBOT));
    }

    /** The exact view from the robot, tilted as it is. */
    void see(long timestampNanos, CameraConfig mount, Pose3d robot) {
      Pose3d cameraPose = robot.transformBy(mount.robotToCamera());
      frames.add(new Frame(timestampNanos, 2, false, Transform3d.ZERO, 0));
      for (int id : new int[] {25, 26}) {
        Transform3d view = new Transform3d(cameraPose, FIELD.tag(id).orElseThrow());
        tags.add(new SeenTag(id, 0, 0, 1, 0, 1, 1, 0, 1, view, Transform3d.ZERO, 0.05, false));
      }
    }
  }

  private final ScriptedCamera io = new ScriptedCamera();

  /**
   * Odometry, fed by hand below: the robot stands still, its gyro reading 0.3 rad, while the
   * cameras see it.
   */
  private final PoseEstimator estimator =
      new PoseEstimator(
          EstimatorConfig.DEFAULTS,
          FIELD::contains,
          FIELD.tags().keySet(),
          new PoseReset(0, ROBOT, PoseCovariance.fromStdDevs(0.05, 0.05, 0.01), "placed"));

  /** The template's robot, with this test's cameras: the front one, and a second beside it. */
  private final RobotDefinition definition =
      RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
          .cameras(List.of(camera, CameraConfigBuilder.builder(camera).name("second").build()))
          .build();

  /** Localization around that odometry: it solves each image as the robot was then. */
  private final LocalizationService localization =
      new LocalizationService(
          estimator,
          new SwerveKinematicsModel(definition.moduleTranslations()),
          FIELD,
          FIELD.terrain(),
          definition,
          RobotLoop.PERIOD_SECONDS);

  private Vision vision;

  /** The robot's alliance, as the Driver Station would say it: unknown until set. */
  private Optional<Alliance> ourAlliance = Optional.empty();

  /** The vision settings as Pitwall would hand them over, tuned or not. */
  private VisionConfig tuned = VisionConfig.DEFAULTS;

  @BeforeEach
  void setUp() {
    TunableRegistry.registerBackend("", new MockTunableBackend());
    vision =
        new Vision(
            List.of(camera),
            new CameraIO[] {io},
            localization,
            FIELD,
            () -> tuned,
            () -> ourAlliance);
    for (int k = 0; k <= 10; k++) {
      estimator.addOdometry(T0 + k * 20_000_000L, new BodyMotion(0, 0, 0, 0), 0.3, YawSource.GYRO);
    }
    estimator.addOdometry(T0 + 220_000_000L, new BodyMotion(0, 0, 0, 0), 0.3, YawSource.GYRO);
  }

  @AfterEach
  void tearDown() {
    vision.close();
    TunableRegistry.reset();
  }

  private void loop() {
    loop(0, 0);
  }

  /**
   * One loop as the robot runs it, at the newest odometry's time: localization hears the IMU (its
   * tilt, and no new wheel readings), the cameras hand it their images, and it applies them.
   *
   * @return what became of each image's measurement
   */
  private List<FrameVerdict> loop(double pitch, double roll) {
    long now = estimator.estimate().timestampNanos();
    LoopContext loop = new LoopContext(now, true);
    localization.drivetrain(tilted(now, pitch, roll), loop);
    vision.periodic(now);
    List<FrameVerdict> frames = localization.update().frames();
    vision.record(frames);
    return frames;
  }

  /** The drivetrain's readings in a loop without new wheel readings: the gyro, tilted so. */
  private static DrivetrainReadings tilted(long now, double pitch, double roll) {
    return DrivetrainReadings.builder()
        .wheels(List.of())
        .gyro(
            ImuReadings.builder()
                .reporting(true)
                .yawRadians(0.3)
                .yawRateRadiansPerSecond(0)
                .pitchRadians(pitch)
                .rollRadians(roll)
                // Gravity, as an accelerometer tilted so feels it standing still.
                .forwardAccelerationMetersPerSecSq(-ImuReadings.STANDARD_GRAVITY * Math.sin(pitch))
                .leftAccelerationMetersPerSecSq(
                    ImuReadings.STANDARD_GRAVITY * Math.sin(roll) * Math.cos(pitch))
                .yaws(YawSamples.of(new long[] {now}, new double[] {0.3}))
                .build())
        .backupImu(ImuReadings.NONE)
        .velocities(new ChassisVelocities())
        .build();
  }

  @Test
  void anImageBecomesAMeasurementTheEstimatorAccepts() {
    io.see(T0 + 200_000_000L, camera);

    loop();

    assertThat(vision.camera(0).accepted()).isEqualTo(1);
    assertThat(vision.summary())
        .isEqualTo("1 of 1 cameras connected, 100% of measurements accepted");
    assertThat(vision.meanTagResidualMeters("front")).isLessThan(1e-6);
    assertThat(vision.camera(0).acceptedTags())
        .as("where the field has the tags it saw, for AdvantageScope's lines")
        .containsExactly(FIELD.tag(25).orElseThrow(), FIELD.tag(26).orElseThrow());
    assertThat(vision.camera(0).rejectedTags()).isEmpty();
  }

  /** A measurement the estimator refuses has its tags drawn refused, each with the reason. */
  @Test
  void anImageTheEstimatorRefusesShowsItsTagsRefusedAndWhy() {
    io.see(
        T0 + 200_000_000L, camera, new Pose3d(ROBOT.plus(new Transform2d(3, 0, Rotation2d.ZERO))));

    loop();

    Camera front = vision.camera(0);
    assertThat(front.rejected()).isEqualTo(1);
    assertThat(front.acceptedTags()).isEmpty();
    assertThat(front.rejectedTags())
        .containsExactly(FIELD.tag(25).orElseThrow(), FIELD.tag(26).orElseThrow());
    assertThat(front.rejectedReasons()).hasSize(2).allMatch(why -> !why.isBlank());
    assertThat(front.rejectedReasons().get(0)).isEqualTo(front.rejectedReasons().get(1));

    loop();

    assertThat(front.rejectedTags()).as("only this loop's").isEmpty();
  }

  /** A tuned setting applies from the next loop's images. */
  @Test
  void theSettingsAreReadEveryLoop() {
    tuned = VisionConfigBuilder.builder(VisionConfig.DEFAULTS).maxDistanceMeters(0.1).build();
    io.see(T0 + 200_000_000L, camera);
    loop();
    tuned = VisionConfig.DEFAULTS;
    io.see(T0 + 220_000_000L, camera);

    loop();

    assertThat(vision.camera(0).accepted()).as("only the image after").isEqualTo(1);
  }

  @Test
  void theSameImageTwiceCountsOnce() {
    io.see(T0 + 200_000_000L, camera);
    loop();
    io.see(T0 + 200_000_000L, camera);

    loop();

    assertThat(vision.camera(0).accepted() + vision.camera(0).rejected()).isEqualTo(1);
  }

  @Test
  void aCameraTurnedOffIsLeftOut() {
    vision.camera(0).enabled.set(false);
    io.see(T0 + 200_000_000L, camera);

    loop();

    assertThat(vision.camera(0).accepted() + vision.camera(0).rejected()).isZero();
    assertThat(vision.summary()).endsWith("no measurements yet");
    assertThat(vision.camera(0).rejectedTags()).hasSize(2);
    assertThat(vision.camera(0).rejectedReasons()).containsOnly("camera turned off");
  }

  @Test
  void aDisconnectedCameraRaisesItsAlert() {
    io.connected = false;

    loop();

    assertThat(vision.camera(0).isDisconnectedAlertActive()).isTrue();
    assertThat(vision.summary()).startsWith("0 of 1 cameras connected");
  }

  @Test
  void aTiltedRobotsImagesAreSolvedTilted() {
    double pitch = Math.toRadians(-8); // nose up on a ramp
    double roll = Math.toRadians(3);
    io.see(
        T0 + 200_000_000L,
        camera,
        new Pose3d(
            ROBOT.getX(),
            ROBOT.getY(),
            0,
            new Rotation3d(roll, pitch, ROBOT.getRotation().getRadians())));

    loop(pitch, roll);

    assertThat(vision.camera(0).accepted()).isEqualTo(1);
    assertThat(estimator.pose().getTranslation().getDistance(ROBOT.getTranslation()))
        .isLessThan(1e-6);
    assertThat(vision.meanTagResidualMeters("front")).isLessThan(1e-6);
  }

  /** On a bump the attitude changes before an image arrives: it's solved as the robot was then. */
  @Test
  void anImageIsSolvedWithTheAttitudeWhenItWasTaken() {
    double pitch = Math.toRadians(-8);
    double roll = Math.toRadians(3);
    loop(pitch, roll); // tilted as of T0 + 220 ms, when the image is taken
    estimator.addOdometry(T0 + 240_000_000L, new BodyMotion(0, 0, 0, 0), 0.3, YawSource.GYRO);
    io.see(
        T0 + 220_000_000L,
        camera,
        new Pose3d(
            ROBOT.getX(),
            ROBOT.getY(),
            0,
            new Rotation3d(roll, pitch, ROBOT.getRotation().getRadians())));

    loop(0, 0); // level again by the time it arrives

    assertThat(vision.camera(0).accepted()).isEqualTo(1);
    assertThat(estimator.pose().getTranslation().getDistance(ROBOT.getTranslation()))
        .isLessThan(1e-6);
  }

  @Test
  void autonomousStartingCanHoldTheCamerasBack() {
    vision.autoStartHoldSeconds().set(2.0);
    vision.autonomousStarted(T0 + 100_000_000L);
    io.see(T0 + 200_000_000L, camera);

    loop();

    assertThat(vision.camera(0).accepted() + vision.camera(0).rejected()).as("held").isZero();
    vision.autoStartHoldSeconds().set(0.0);
    vision.autonomousStarted(T0 + 100_000_000L);
    io.see(T0 + 210_000_000L, camera);
    loop();
    assertThat(vision.camera(0).accepted()).as("off: nothing held").isEqualTo(1);
  }

  /**
   * PhotonLib throws, every few seconds, when the coprocessor runs a different PhotonVision than
   * the robot's library: the camera reads as unreadable, and the robot program runs on.
   */
  @Test
  void aCameraThatThrowsIsUnreadableAndTheLoopGoesOn() {
    io.failure = new UnsupportedOperationException("Photonlib version mismatch\n");

    loop();

    assertThat(vision.camera(0).isUnreadableAlertActive()).isTrue();
    assertThat(vision.camera(0).isDisconnectedAlertActive())
        .as("unreadable says why instead")
        .isFalse();
    assertThat(vision.summary()).startsWith("0 of 1 cameras connected");

    io.failure = null;
    io.see(T0 + 200_000_000L, camera);
    loop();
    assertThat(vision.camera(0).isUnreadableAlertActive()).isFalse();
    assertThat(vision.camera(0).accepted()).isEqualTo(1);
  }

  /** Whatever a camera sends that isn't a number costs that image, never the robot program. */
  @Test
  void badNumbersFromACameraCostOnlyTheirImage() {
    Transform3d notANumber = new Transform3d(Double.NaN, 0, 0, Rotation3d.ZERO);
    io.frames.add(new CameraIO.Frame(T0 + 120_000_000L, 1, true, notANumber, Double.NaN));
    io.tags.add(
        new CameraIO.SeenTag(26, 0, 0, 1, 0, 1, 1, 0, 1, notANumber, notANumber, Double.NaN, true));
    io.frames.add(new CameraIO.Frame(-1, 0, false, Transform3d.ZERO, 0)); // from an old log
    io.see(T0 + 200_000_000L, camera);
    io.tags.set(io.tags.size() - 1, withAmbiguity(io.tags.getLast(), Double.NaN));

    loop();

    assertThat(vision.camera(0).accepted()).as("only the good image").isEqualTo(1);
    assertThat(estimator.pose().getTranslation().getDistance(ROBOT.getTranslation()))
        .isLessThan(1e-6);
  }

  @Test
  void anAutoStartHoldThatIsntATimeHoldsNothing() {
    vision.autoStartHoldSeconds().set(Double.POSITIVE_INFINITY);
    vision.autonomousStarted(T0 + 100_000_000L);
    vision.autoStartHoldSeconds().set(Double.NaN);
    vision.autonomousStarted(T0 + 100_000_000L);
    io.see(T0 + 200_000_000L, camera);

    loop();

    assertThat(vision.camera(0).accepted()).isEqualTo(1);
  }

  private static CameraIO.SeenTag withAmbiguity(CameraIO.SeenTag tag, double ambiguity) {
    return new CameraIO.SeenTag(
        tag.id(),
        tag.corner0X(),
        tag.corner0Y(),
        tag.corner1X(),
        tag.corner1Y(),
        tag.corner2X(),
        tag.corner2Y(),
        tag.corner3X(),
        tag.corner3Y(),
        tag.bestCameraToTag(),
        tag.altCameraToTag(),
        ambiguity,
        tag.inMultiTag());
  }

  /**
   * On a bump the robot stands as high as the ground under it, and its tags are placed from there:
   * from the floor, every one would land as far off as the bump is tall.
   */
  @Test
  void onABumpTagsArePlacedFromAsHighAsTheGround() {
    double crest = 0.165;
    FieldModel bumpy =
        new FieldModel(
            Fields.FRC_2026_REBUILT_WELDED.loadField(),
            Symmetry.ROTATED,
            List.of(),
            List.of(),
            List.of(),
            List.of(
                new Bump(
                    "Test bump",
                    new Translation2d(ROBOT.getX(), 3.0),
                    new Translation2d(ROBOT.getX(), 5.0),
                    crest,
                    1.0)));
    vision.close();
    vision =
        new Vision(
            List.of(camera),
            new CameraIO[] {io},
            localization,
            bumpy,
            () -> tuned,
            () -> ourAlliance);
    io.see(
        T0 + 200_000_000L,
        camera,
        new Pose3d(
            ROBOT.getX(),
            ROBOT.getY(),
            crest,
            new Rotation3d(0, 0, ROBOT.getRotation().getRadians())));

    loop();

    assertThat(vision.meanTagResidualMeters("front")).isLessThan(1e-6);
  }

  @Test
  void aCameraWhoseTagsLandOffShowsItInItsResidual() {
    CameraConfig reallyHigher =
        CameraConfig.builder()
            .name("front")
            .forward(Inches.of(12))
            .left(Inches.of(0))
            .up(Inches.of(16))
            .roll(Degrees.of(0))
            .tiltUp(Degrees.of(15))
            .yaw(Degrees.of(0))
            .sensor(camera.sensor())
            .noiseScale(1.0)
            .build();
    io.see(T0 + 200_000_000L, reallyHigher);

    loop();

    assertThat(vision.meanTagResidualMeters("front")).isBetween(0.09, 0.11); // 4 in
  }

  /**
   * An image stamped far earlier than its camera's images take to arrive, or after the robot read
   * it, has the wrong capture time: the camera's clock and the robot's are out of step. It's left
   * out before the estimator sees it, counted, and an alert says so.
   */
  @Test
  void anImageOutsideItsCamerasLatencyBandIsLeftOut() {
    loop(); // the loop before, which every image read later arrived after
    long now = estimator.estimate().timestampNanos();
    io.see(now - 300_000_000L, camera); // the camera takes 30 ms; 100 ms more is the margin

    loop();

    assertThat(vision.camera(0).accepted() + vision.camera(0).rejected()).isZero();
    assertThat(vision.camera(0).rejectedReasons())
        .containsOnly("arrived at least 300 ms after it was taken, more than 130 ms");
    assertThat(vision.camera(0).isOutOfStepAlertActive()).isTrue();
    assertThat(vision.camera(0).outOfStepAlertText())
        .isEqualTo(
            "Camera front's images are out of step, and left out: one arrived at least 300 ms"
                + " after it was taken, more than 130 ms. Are its clock and the robot's in step"
                + " (time sync)? If it's just slower, measure its latency");
    io.see(now + 5_000_000L, camera);
    loop();
    assertThat(vision.camera(0).accepted() + vision.camera(0).rejected()).isZero();
    assertThat(vision.summary()).endsWith("no measurements yet, 2 images left out of step");
  }

  /**
   * The alert stays up two seconds after the last image out of step, however many arrive in time.
   */
  @Test
  void theOutOfStepAlertStaysUpTwoSecondsAfterTheLast() {
    loop();
    io.see(estimator.estimate().timestampNanos() - 300_000_000L, camera);
    loop();

    imagesFrom(camera, 95); // 1.9 s of images in time

    assertThat(vision.camera(0).accepted()).isEqualTo(95);
    assertThat(vision.camera(0).isOutOfStepAlertActive()).as("still held").isTrue();
    imagesFrom(camera, 10);
    assertThat(vision.camera(0).isOutOfStepAlertActive()).isFalse();
  }

  /**
   * A loop that ran long (here 400 ms) reads images that arrived just after the loop before: they
   * waited on the robot, not the camera, and are in time.
   */
  @Test
  void aLoopThatRanLongDoesNotMakeItsImagesLate() {
    loop();
    long before = estimator.estimate().timestampNanos();
    long now = before + 400_000_000L;
    estimator.addOdometry(now, new BodyMotion(0, 0, 0, 0), 0.3, YawSource.GYRO);
    io.see(before + 20_000_000L, camera); // arrived 30 ms later, then waited 350 ms for the loop

    loop();

    assertThat(vision.camera(0).accepted()).isEqualTo(1);
    assertThat(vision.camera(0).isOutOfStepAlertActive()).isFalse();
  }

  /** A camera turned off, or gone, says so itself: its out-of-step alert comes down at once. */
  @Test
  void aCameraTurnedOffOrGoneTakesDownItsOutOfStepAlert() {
    loop();
    io.see(estimator.estimate().timestampNanos() - 300_000_000L, camera);
    loop();
    assertThat(vision.camera(0).isOutOfStepAlertActive()).isTrue();

    vision.camera(0).enabled.set(false);
    loop();

    assertThat(vision.camera(0).isOutOfStepAlertActive()).isFalse();
    vision.camera(0).enabled.set(true);
    io.see(estimator.estimate().timestampNanos() - 299_000_000L, camera);
    loop();
    assertThat(vision.camera(0).isOutOfStepAlertActive()).isTrue();
    io.connected = false;
    loop();
    assertThat(vision.camera(0).isOutOfStepAlertActive()).isFalse();
  }

  /** One loop, and the tags each of its measurements came from. */
  private List<List<Integer>> tagsMeasuredFrom() {
    return loop(0, 0).stream()
        .filter(frame -> frame.measured().isPresent())
        .map(frame -> frame.tags().stream().map(TagVerdict::tag).toList())
        .toList();
  }

  /** The next image of the hub's near face, each a millisecond after the last. */
  private long nextImage = T0 + 200_000_000L;

  /** The tags the next image's measurements came from, with the scheduler run first. */
  private List<List<Integer>> nextImageAfter(Scheduler scheduler) {
    scheduler.run();
    io.see(nextImage++, camera);
    return tagsMeasuredFrom();
  }

  /** A task that drives until it's interrupted: an approach that never quite arrives. */
  private static Command approach(Mechanism drive, String name) {
    return drive
        .run(
            coroutine -> {
              while (true) {
                coroutine.yield();
              }
            })
        .named(name);
  }

  /**
   * A task lining up on the hub trusts only the hub's tags while it runs; when it ends, or is
   * canceled partway, the cameras trust every tag again.
   */
  @Test
  void aTaskTrustsOnlyItsTagsWhileItRuns() {
    Scheduler scheduler = Scheduler.createIndependentScheduler();
    Command approach =
        Command.noRequirements(
                coroutine -> {
                  coroutine.yield();
                  coroutine.yield();
                })
            .named("Approach the hub");
    Command task = vision.trustingOnly(TagAllowlist.of("Hub", List.of(26)), approach);
    scheduler.schedule(task);

    assertThat(task.name()).isEqualTo("Approach the hub (trusting Hub: 26)");
    assertThat(nextImageAfter(scheduler)).containsExactly(List.of(26));
    assertThat(vision.trustingFewerTagsAlertText()).isEqualTo("The cameras trust only Hub: 26");
    for (int k = 0; k < 5; k++) {
      scheduler.run();
    }
    assertThat(scheduler.isScheduledOrRunning(task)).as("done").isFalse();
    assertThat(nextImageAfter(scheduler)).containsExactly(List.of(25, 26));
    assertThat(vision.trustingFewerTagsAlertText()).as("not active").isEmpty();

    Command canceled = vision.trustingOnly(TagAllowlist.of("Hub", List.of(26)), approach);
    scheduler.schedule(canceled);
    scheduler.run();
    scheduler.cancel(canceled);
    assertThat(nextImageAfter(scheduler)).containsExactly(List.of(25, 26));
  }

  /**
   * A task that interrupts another on the same mechanism ends it first, so the one ending lets go
   * of its tags before the one starting takes its own.
   */
  @Test
  void aTaskThatInterruptsAnotherTrustsItsOwnTags() {
    Scheduler scheduler = Scheduler.createIndependentScheduler();
    Mechanism drive = new Mechanism() {};
    Command first =
        vision.trustingOnly(TagAllowlist.of("Hub", List.of(26)), approach(drive, "Approach"));
    Command second =
        vision.trustingOnly(TagAllowlist.of("Left face", List.of(25)), approach(drive, "Swerve"));
    scheduler.schedule(first);
    assertThat(nextImageAfter(scheduler)).containsExactly(List.of(26));

    scheduler.schedule(second);

    assertThat(nextImageAfter(scheduler)).containsExactly(List.of(25));
    assertThat(scheduler.isScheduledOrRunning(first)).isFalse();
    scheduler.cancel(second);
    assertThat(nextImageAfter(scheduler)).containsExactly(List.of(25, 26));
  }

  /** A task the scheduler refuses (a higher priority one holds the mechanism) never takes tags. */
  @Test
  void aTaskRefusedLeavesTheRunningTasksTags() {
    Scheduler scheduler = Scheduler.createIndependentScheduler();
    Mechanism drive = new Mechanism() {};
    Command important =
        vision.trustingOnly(
            TagAllowlist.of("Hub", List.of(26)),
            drive
                .run(
                    coroutine -> {
                      while (true) {
                        coroutine.yield();
                      }
                    })
                .withPriority(10)
                .named("Score"));
    Command refused =
        vision.trustingOnly(TagAllowlist.of("Left face", List.of(25)), approach(drive, "Swerve"));
    scheduler.schedule(important);
    scheduler.run();

    assertThat(scheduler.schedule(refused).successful()).as("refused outright").isFalse();
    assertThat(nextImageAfter(scheduler)).containsExactly(List.of(26));
    assertThat(scheduler.isScheduledOrRunning(important)).isTrue();
    assertThat(scheduler.isScheduledOrRunning(refused)).isFalse();
  }

  /**
   * Two tasks on different mechanisms both run: the later one's tags are in effect, and when it
   * ends, the earlier one's are again.
   */
  @Test
  void whenTheLaterTaskEndsTheEarlierOnesTagsAreBack() {
    Scheduler scheduler = Scheduler.createIndependentScheduler();
    Command aiming =
        vision.trustingOnly(
            TagAllowlist.of("Hub", List.of(26)), approach(new Mechanism() {}, "Aim"));
    Command reaching =
        vision.trustingOnly(
            TagAllowlist.of("Left face", List.of(25)), approach(new Mechanism() {}, "Reach"));
    scheduler.schedule(aiming);
    scheduler.run();
    scheduler.schedule(reaching);
    assertThat(nextImageAfter(scheduler)).containsExactly(List.of(25));

    scheduler.cancel(reaching);

    assertThat(nextImageAfter(scheduler)).containsExactly(List.of(26));
    scheduler.cancel(aiming);
    assertThat(nextImageAfter(scheduler)).containsExactly(List.of(25, 26));
  }

  /**
   * A task lining up on a place trusts that place's tags for the alliance the robot is on when it
   * starts: a binding is made before the alliance is known.
   */
  @Test
  void aTaskOnAPlaceTrustsItsAlliancesTagsWhenItStarts() {
    Scheduler scheduler = Scheduler.createIndependentScheduler();
    Place face = Place.facing("Hub face", 26, 1.0);
    Command beforeTheAlliance = vision.trustingOnly(face, approach(new Mechanism() {}, "Line up"));
    assertThat(beforeTheAlliance.name()).isEqualTo("Line up (trusting Hub face's tags)");

    scheduler.schedule(beforeTheAlliance);
    assertThat(nextImageAfter(scheduler))
        .as("no alliance: every tag")
        .containsExactly(List.of(25, 26));
    assertThat(vision.trustingFewerTagsAlertText())
        .isEqualTo("Hub face: the alliance isn't known, so every tag is trusted");
    scheduler.cancel(beforeTheAlliance);

    ourAlliance = Optional.of(Alliance.BLUE);
    scheduler.schedule(beforeTheAlliance);
    assertThat(nextImageAfter(scheduler)).containsExactly(List.of(26));
    scheduler.cancel(beforeTheAlliance);

    ourAlliance = Optional.of(Alliance.RED);
    scheduler.schedule(beforeTheAlliance);
    assertThat(nextImageAfter(scheduler)).as("red's face is across the field").isEmpty();
  }

  @Test
  void aPlaceWrittenAsAPoseHasNoTagsToTrust() {
    assertThatThrownBy(
            () ->
                vision.trustingOnly(
                    Place.at("Start", Pose2d.ZERO), approach(new Mechanism() {}, "Line up")))
        .hasMessage("Start is written as a pose, not found from tags: it has no tags to trust");
  }

  @Test
  void anImageWithNoTagTheTaskTrustsIsLeftOut() {
    Vision.TagLease lease = vision.trustOnly(TagAllowlist.of("Loading", List.of(13)));
    io.see(T0 + 200_000_000L, camera);

    assertThat(tagsMeasuredFrom()).isEmpty();
    lease.release();
    lease.release(); // twice changes nothing more
    io.see(T0 + 210_000_000L, camera);
    assertThat(tagsMeasuredFrom()).containsExactly(List.of(25, 26));
  }

  /** Images from the front camera, one a loop, with odometry moving on to each one's time. */
  private void imagesFrom(CameraConfig reallyMounted, int images) {
    for (int k = 0; k < images; k++) {
      long now = estimator.estimate().timestampNanos() + 20_000_000L;
      estimator.addOdometry(now, new BodyMotion(0, 0, 0, 0), 0.3, YawSource.GYRO);
      io.see(now, reallyMounted);
      if (second != null) {
        secondIo.see(now, second);
      }
      loop();
    }
  }

  /** A second camera, mounted as configured beside the front one, to hold the pose. */
  private @Nullable CameraConfig second;

  private final ScriptedCamera secondIo = new ScriptedCamera();

  /** Vision with the front camera and a second one, as configured, that sees the same tags. */
  private void withASecondCamera() {
    second = CameraConfigBuilder.builder(camera).name("second").build();
    vision.close();
    vision =
        new Vision(
            List.of(camera, second),
            new CameraIO[] {io, secondIo},
            localization,
            FIELD,
            () -> tuned,
            () -> ourAlliance);
  }

  /** A camera on its own has nothing to hold the pose it'd be judged against: it never is. */
  @Test
  void aCameraAloneIsNeverJudged() {
    CameraConfig reallyLower = CameraConfigBuilder.builder(camera).up(Inches.of(8)).build();

    imagesFrom(reallyLower, 300);

    assertThat(vision.mountOffsetMeters("front")).isNaN();
  }

  /**
   * A camera knocked 4 in down since it was calibrated: once it has seen enough tags, every one of
   * them 4 in high, it's out of calibration, and the alert says where to recalibrate it. Knocked
   * straight down, it still places the robot right on the floor, so it counts as much as before.
   */
  @Test
  void aCameraWhoseTagsLandOffIsOutOfCalibrationOnceJudged() {
    CameraConfig reallyLower = CameraConfigBuilder.builder(camera).up(Inches.of(8)).build();
    withASecondCamera();

    imagesFrom(reallyLower, 200); // two tags an image: 400 seen, not enough to judge

    assertThat(vision.mountOffsetMeters("front")).isNaN();
    assertThat(vision.camera(0).isOutOfCalibrationAlertActive()).isFalse();
    imagesFrom(reallyLower, 60);
    assertThat(vision.mountOffsetMeters("front")).isCloseTo(0.1016, within(0.002));
    assertThat(vision.camera(0).isOutOfCalibrationAlertActive()).isTrue();
    assertThat(vision.camera(0).outOfCalibrationAlertText())
        .isEqualTo(
            "Camera front is out of calibration: its tags land 10 cm off. Recalibrate its mount"
                + " (Calibrate the front camera's mount)");
    assertThat(vision.mountDoubtMeters("front")).as("still places the robot right").isZero();
    assertThat(estimator.pose().getTranslation().getDistance(ROBOT.getTranslation()))
        .isLessThan(1e-6);
  }

  /**
   * The tolerance is the pit's to tune, and applies from the next loop; the alert holds still
   * around it.
   */
  @Test
  void theMountToleranceIsReadEveryLoop() {
    CameraConfig reallyLower = CameraConfigBuilder.builder(camera).up(Inches.of(8)).build();
    withASecondCamera();
    imagesFrom(reallyLower, 260);
    assertThat(vision.camera(0).isOutOfCalibrationAlertActive()).isTrue();

    // 10 cm off. Up past the tolerance, the alert comes down only under 0.8 of it.
    tuned = VisionConfigBuilder.builder(VisionConfig.DEFAULTS).mountToleranceMeters(0.11).build();
    loop();
    assertThat(vision.camera(0).isOutOfCalibrationAlertActive()).as("8.8 cm: still up").isTrue();
    tuned = VisionConfigBuilder.builder(VisionConfig.DEFAULTS).mountToleranceMeters(0.13).build();
    loop();
    assertThat(vision.camera(0).isOutOfCalibrationAlertActive()).as("10.4 cm: down").isFalse();
    tuned = VisionConfigBuilder.builder(VisionConfig.DEFAULTS).mountToleranceMeters(0.11).build();
    loop();
    assertThat(vision.camera(0).isOutOfCalibrationAlertActive()).as("11 cm: stays down").isFalse();
  }
}
