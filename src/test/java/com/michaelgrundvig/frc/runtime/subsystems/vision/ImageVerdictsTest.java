package com.michaelgrundvig.frc.runtime.subsystems.vision;

import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Milliseconds;

import com.michaelgrundvig.frc.localization.BodyMotion;
import com.michaelgrundvig.frc.localization.DefaultLocalizer;
import com.michaelgrundvig.frc.localization.DrivetrainReadings;
import com.michaelgrundvig.frc.localization.EstimatorConfig;
import com.michaelgrundvig.frc.localization.ImuReadings;
import com.michaelgrundvig.frc.localization.LocalizationService;
import com.michaelgrundvig.frc.localization.LoopContext;
import com.michaelgrundvig.frc.localization.PoseCovariance;
import com.michaelgrundvig.frc.localization.PoseEstimator;
import com.michaelgrundvig.frc.localization.PoseReset;
import com.michaelgrundvig.frc.localization.SlipCheck;
import com.michaelgrundvig.frc.localization.SwerveKinematicsModel;
import com.michaelgrundvig.frc.localization.Verdict;
import com.michaelgrundvig.frc.localization.VisionConfig;
import com.michaelgrundvig.frc.localization.YawSamples;
import com.michaelgrundvig.frc.localization.YawSource;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Symmetry;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;

/**
 * What became of every image a camera took is counted by its verdict, whatever decided it: the
 * robot (turned off, the same image again, out of step, no capture time, unreadable), the pipeline
 * or the estimator. Each "no measurement" cause says itself in its own words in the camera's log
 * and on its card, and the pit's line names a camera whose every image is refused, with its leading
 * reason.
 */
class ImageVerdictsTest {
  private static final FieldModel FIELD =
      new FieldModel(
          Rebuilt.GAME.layout(Fields.FRC_2026_REBUILT_WELDED.loadField()),
          Symmetry.ROTATED,
          List.of(),
          List.of(),
          List.of());
  private static final Pose2d ROBOT = new Pose2d(2.5, 4.1, Rotation2d.fromDegrees(10));
  private static final long T0 = 1_000_000_000L;
  private static final long LOOP_NANOS = 20_000_000L;

  private static final CameraConfig PHOTONVISION = camera(CameraConfig.Software.PHOTONVISION);

  /** A camera on Visor, on a robot, that nobody has calibrated a lens for. */
  private static final CameraConfig WITHOUT_A_LENS =
      camera(CameraConfig.Software.visor().device("/dev/v4l/by-id/usb-front-video-index0"));

  private static CameraConfig camera(CameraConfig.Software software) {
    return CameraConfig.of(
            "front",
            CameraConfig.ImageSensor.of(1280, 800, Hertz.of(30))
                .fieldOfView(Degrees.of(80), CameraConfig.Across.DIAGONAL)
                .latency(Milliseconds.of(30))
                .sightRange(Meters.of(7)))
        .at(Inches.of(12), Inches.of(0), Inches.of(12))
        .facing(Degrees.of(15), Degrees.of(0))
        .roll(Degrees.of(0))
        .noiseScale(1.0)
        .measured()
        .software(software);
  }

  /** A camera that hands over whatever the test scripts, its tags with poses or without. */
  private static final class ScriptedCamera implements CameraIO {
    int skipped;
    List<Frame> frames = new ArrayList<>();
    List<SeenTag> tags = new ArrayList<>();
    List<Integer> unsolved = new ArrayList<>();

    @Override
    public void updateInputs(CameraIOInputs inputs) {
      inputs.connected = true;
      inputs.skippedFrames = skipped;
      inputs.frames = frames.toArray(Frame[]::new);
      inputs.tags = tags.toArray(SeenTag[]::new);
      inputs.unsolvedTags = unsolved.stream().mapToInt(Integer::intValue).toArray();
      frames = new ArrayList<>();
      tags = new ArrayList<>();
      unsolved = new ArrayList<>();
      skipped = 0;
    }

    /** The exact view of these tags from the robot, each at its place on the field. */
    void see(long takenNanos, CameraConfig mount, int... ids) {
      Pose3d cameraPose = new Pose3d(ROBOT).transformBy(mount.robotToCamera());
      frames.add(new Frame(takenNanos, ids.length, false, Transform3d.ZERO, 0));
      for (int id : ids) {
        Transform3d view =
            FIELD
                .tag(id)
                .map(tag -> new Transform3d(cameraPose, tag))
                .orElse(new Transform3d(1, 0, 0, new Rotation3d()));
        tags.add(new SeenTag(id, 0, 0, 1, 0, 1, 1, 0, 1, view, Transform3d.ZERO, 0.05, false));
      }
    }

    /** Tags the camera found and sent without poses, as Visor does without a lens. */
    void seeUnsolved(long takenNanos, int... ids) {
      frames.add(new Frame(takenNanos, 0, ids.length, false, Transform3d.ZERO, 0));
      for (int id : ids) {
        unsolved.add(id);
      }
    }
  }

  /** One camera's robot: odometry fed by hand, the robot standing still where it was placed. */
  private static final class Rig implements AutoCloseable {
    final ScriptedCamera io = new ScriptedCamera();
    final PoseEstimator estimator =
        new PoseEstimator(
            EstimatorConfig.defaults(),
            FIELD::contains,
            FIELD.tags().keySet(),
            new PoseReset(0, ROBOT, PoseCovariance.fromStdDevs(0.05, 0.05, 0.01), "placed"));
    final Vision vision;
    final LocalizationService localization;
    long now = T0;

    Rig(CameraConfig camera) {
      // Odometry from the start, so the images that follow are inside its history.
      for (int k = 0; k <= 10; k++) {
        now = T0 + k * LOOP_NANOS;
        estimator.addOdometry(now, new BodyMotion(0, 0, 0, 0), 0.3, YawSource.GYRO);
      }
      RobotDefinition definition =
          RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
              .cameras(List.of(camera))
              .build();
      localization =
          new LocalizationService(
              FIELD,
              FIELD.terrain(),
              definition,
              new DefaultLocalizer(
                  estimator,
                  new SwerveKinematicsModel(definition.moduleTranslations()),
                  FIELD.terrain(),
                  definition,
                  SlipCheck.Settings.defaults()),
              RobotLoop.PERIOD_SECONDS);
      vision =
          new Vision(
              List.of(camera),
              new CameraIO[] {io},
              localization,
              FIELD,
              () -> VisionConfig.defaults(),
              Optional::empty);
    }

    /** One loop as the robot runs it, the odometry a loop on. */
    void loop() {
      now += LOOP_NANOS;
      estimator.addOdometry(now, new BodyMotion(0, 0, 0, 0), 0.3, YawSource.GYRO);
      localization.drivetrain(standing(now), new LoopContext(now, true));
      vision.periodic(now);
      vision.record(localization.update().frames());
    }

    Camera front() {
      return vision.camera(0);
    }

    @Override
    public void close() {
      vision.close();
    }
  }

  private static DrivetrainReadings standing(long now) {
    return DrivetrainReadings.builder()
        .wheels(List.of())
        .gyro(
            ImuReadings.builder()
                .reporting(true)
                .yawRadians(0.3)
                .yawRateRadiansPerSecond(0)
                .pitchRadians(0)
                .rollRadians(0)
                .forwardAccelerationMetersPerSecSq(0)
                .leftAccelerationMetersPerSecSq(0)
                .yaws(YawSamples.of(new long[] {now}, new double[] {0.3}))
                .build())
        .backupImu(ImuReadings.NONE)
        .velocities(new ChassisVelocities())
        .build();
  }

  @BeforeEach
  void setUp() {
    TunableRegistry.registerBackend("", new MockTunableBackend());
  }

  @AfterEach
  void tearDown() {
    TunableRegistry.reset();
  }

  /**
   * What a camera's loops hand over, and then: what its log says of the last image, how many of its
   * images came to each verdict, and what the pit's line says of it.
   */
  private record Row(
      String name,
      CameraConfig camera,
      Consumer<Rig> loops,
      String outcome,
      Map<Verdict, Long> counted,
      String pitLineEnds) {}

  @Test
  void everyImageIsCountedByItsVerdictAndEachCauseSaysItself() {
    List<Row> rows =
        List.of(
            new Row(
                "nothing in view",
                PHOTONVISION,
                rig -> rig.io.see(rig.now, PHOTONVISION),
                "saw no tags",
                Map.of(Verdict.NO_TAGS, 1L),
                "no measurements yet"),
            new Row(
                "tags off the field's layout",
                PHOTONVISION,
                rig -> rig.io.see(rig.now, PHOTONVISION, 98, 99),
                "tags 98, 99 aren't in this field's layout",
                Map.of(Verdict.UNKNOWN_TAGS, 1L),
                "front used 0 of 1 images with tags (1: tags not in this field's layout)"),
            new Row(
                "tags from Visor with no lens calibration, every loop",
                WITHOUT_A_LENS,
                rig -> rig.io.seeUnsolved(rig.now, 25, 26),
                "saw tags 25, 26 and has no lens calibration to solve them: Visor sends corners and"
                    + " no poses without one (docs/visor.md)",
                Map.of(Verdict.NO_LENS, 3L),
                "front used 0 of 3 images with tags (3: no lens calibration)"),
            new Row(
                "turned off",
                PHOTONVISION,
                rig -> {
                  rig.front().enabled.set(false);
                  rig.io.see(rig.now, PHOTONVISION, 25, 26);
                },
                "camera turned off",
                Map.of(Verdict.TURNED_OFF, 1L),
                "front used 0 of 1 images with tags (1: camera turned off)"),
            new Row(
                "the same image again, after one accepted",
                PHOTONVISION,
                rig -> {
                  rig.io.see(rig.now, PHOTONVISION, 25, 26);
                  rig.loop();
                  rig.io.see(rig.now - LOOP_NANOS, PHOTONVISION, 25, 26);
                },
                "the same image again",
                Map.of(Verdict.ACCEPTED, 1L, Verdict.SEEN_BEFORE, 1L),
                "front used 1 of 2 images with tags (1: the same image again)"),
            new Row(
                "no capture time",
                PHOTONVISION,
                rig -> rig.io.skipped = 2,
                "2 image(s) without a capture time, left out",
                Map.of(Verdict.NO_CAPTURE_TIME, 2L),
                "front used 0 of 2 images with tags (2: no capture time)"),
            new Row(
                "unreadable data",
                PHOTONVISION,
                // A capture time no clock has, as an old log can hold.
                rig -> rig.io.frames.add(new CameraIO.Frame(-1, 0, false, Transform3d.ZERO, 0)),
                "unusable data from the camera: ",
                Map.of(Verdict.UNREADABLE, 1L),
                "front used 0 of 1 images with tags (1: unreadable data)"));
    for (Row row : rows) {
      try (Rig rig = new Rig(row.camera())) {
        rig.loop();
        boolean repeats = row.name().endsWith("every loop");
        for (int loop = 0; loop < (repeats ? 3 : 1); loop++) {
          row.loops().accept(rig);
          rig.loop();
        }

        Camera front = rig.front();
        assertThat(front.verdicts()).as(row.name()).last().asString().startsWith(row.outcome());
        for (Verdict verdict : Verdict.values()) {
          assertThat(front.images(verdict))
              .as(row.name() + ": " + verdict)
              .isEqualTo(row.counted().getOrDefault(verdict, 0L));
        }
        assertThat(rig.vision.summary()).as(row.name()).endsWith(row.pitLineEnds());
        assertThat(new CameraPit(front, CameraPit.Frames.FROM_THE_SIMULATOR).card().lines())
            .as(row.name() + ": the camera's card")
            .anySatisfy(
                line -> {
                  assertThat(line.label()).isEqualTo("Images");
                  assertThat(line.value()).contains("latest: " + row.outcome());
                });
      }
    }
  }
}
