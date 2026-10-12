package com.michaelgrundvig.frc.runtime.views.world;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.tuple;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.localization.FrameVerdict;
import com.michaelgrundvig.frc.localization.LocalizationEstimate;
import com.michaelgrundvig.frc.localization.LocalizerReport;
import com.michaelgrundvig.frc.localization.PoseCovariance;
import com.michaelgrundvig.frc.localization.TagVerdict;
import com.michaelgrundvig.frc.localization.Verdict;
import com.michaelgrundvig.frc.localization.VisionMeasurement;
import com.michaelgrundvig.frc.pitwall.push.Feed;
import com.michaelgrundvig.frc.pitwall.scene.Inspection;
import com.michaelgrundvig.frc.pitwall.scene.Scenes;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfigBuilder;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import io.avaje.jsonb.JsonType;
import io.avaje.jsonb.Jsonb;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicLong;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

/**
 * The template's robot on its field as Pitwall's World view gets it: what's described, what moves,
 * and what a click on a sight line says.
 */
class WorldViewTest {
  private static final JsonType<Object> JSON =
      Jsonb.builder().serializeNulls(true).build().type(Object.class);
  private static final RobotDefinition ROBOT = ExampleRobot.SIM.definition();

  private final AtomicLong clock = new AtomicLong(1_000_000_000L);
  private final List<String> said = new ArrayList<>();
  private final Scenes scenes = new Scenes(clock::get, said::add);

  private LocalizationEstimate estimate =
      estimate(new Pose2d(3, 4, Rotation2d.fromDegrees(30)), LocalizationEstimate.Mode.TRACKING);

  /** Whether the robot's localizer reports its options, as the default one does, or none. */
  private enum Options {
    ALL_REPORTED,
    NONE_REPORTED
  }

  private Options options = Options.ALL_REPORTED;
  private SwerveModuleVelocity[] modules = still();
  private List<FrameVerdict> frames = List.of();

  private final WorldView.Readings readings =
      new WorldView.Readings() {
        @Override
        public LocalizerReport report() {
          if (options == Options.NONE_REPORTED) {
            return new LocalizerReport(
                estimate.timestampNanos(),
                estimate.pose(),
                Optional.empty(),
                Optional.empty(),
                Optional.empty());
          }
          return new LocalizerReport(
              estimate.timestampNanos(),
              estimate.pose(),
              Optional.of(estimate.covariance()),
              Optional.of(estimate.mode()),
              Optional.of(estimate.health()));
        }

        @Override
        public double pitchRadians() {
          return 0;
        }

        @Override
        public double rollRadians() {
          return 0;
        }

        @Override
        public SwerveModuleVelocity[] modules() {
          return modules;
        }

        @Override
        public List<FrameVerdict> frames() {
          return frames;
        }
      };

  private static LocalizationEstimate estimate(Pose2d pose, LocalizationEstimate.Mode mode) {
    return new LocalizationEstimate(
        0,
        pose,
        new PoseCovariance(0.0004, 0.0001, 0.0009, 0.0001),
        Translation2d.ZERO,
        0,
        mode,
        new LocalizationEstimate.Health(0.1, 12, 3, 0.02, 0.03, 0, 0, 0, false, 0, true));
  }

  private static SwerveModuleVelocity[] still() {
    SwerveModuleVelocity[] modules = new SwerveModuleVelocity[4];
    for (int i = 0; i < 4; i++) {
      modules[i] = new SwerveModuleVelocity(0, Rotation2d.ZERO);
    }
    return modules;
  }

  private Feed world(boolean simulated) {
    scenes.add(
        WorldView.scene(
            () -> RobotSim.FIELD,
            ROBOT,
            readings,
            simulated ? () -> new Pose3d(new Pose2d(3.05, 4, Rotation2d.fromDegrees(31))) : null,
            camera -> Optional.of("Calibrate " + camera)));
    return scenes.feeds().getFirst();
  }

  private String loop(Feed feed) {
    clock.addAndGet(60_000_000L);
    return JSON.toJson(feed.patch());
  }

  @Test
  void theFieldTheRobotAndItsCamerasAreDescribedFromTheModelAndTheConfiguration() {
    world(true);
    String described = JSON.toJson(scenes.descriptions());

    assertThat(described)
        .contains("\"focus\":\"robot\"")
        .contains("\"walls\"", "\"surface.0\"", "\"tag.7\":{\"parent\":\"field\",\"pose\":")
        .contains("\"id\":\"tag.7.id\"", "\"text\":\"7\"")
        .contains("\"camera.front-left\":{\"parent\":\"robot\",\"pose\":")
        .contains(
            "\"id\":\"camera.front-left.stub\",\"kind\":\"frustum\",\"frame\":\"camera.front-left\","
                + "\"style\":\"muted\",\"inspect\":true,\"reveals\":[\"camera.front-left.view\"]")
        .contains("\"range\":[0,1]")
        .contains("\"camera-views\":{\"title\":\"Camera views\",\"shown\":false")
        .contains("\"camera.front-left.view\"", "\"range\":[0,7]")
        .contains("\"wheel.back-right\"", "\"length\":\"speed.back-right\"")
        .contains("\"imu.gyro\":{\"parent\":\"robot\",\"pose\":", "\"imu.gyro.block\"")
        .contains("\"radii\":[\"uncertainty.major\",\"uncertainty.minor\"]")
        .contains("\"truth.robot\"", "\"style\":\"truth dashed\"")
        .contains("\"sight-lines\":{\"title\":\"Sight lines (dashed: rejected)\"");
    assertThat(RobotSim.FIELD.tags()).hasSizeGreaterThan(20);
    for (int tag : RobotSim.FIELD.tags().keySet()) {
      assertThat(described).contains("\"tag." + tag + ".square\"");
    }
    assertThat(said).isEmpty();
  }

  @Test
  void theRobotIsDrawnWhereTheEstimateIsWithItsWheelsTurnedAndDriving() {
    Feed feed = world(false);
    modules = still();
    modules[0] = new SwerveModuleVelocity(2.0, Rotation2d.fromDegrees(45));
    String patch = loop(feed);

    assertThat(patch)
        .contains("\"robot\":[3,4,30]")
        .contains("\"module.front-left\":[")
        .contains(",45]")
        .contains("\"speed.front-left\":0.4")
        .contains("\"uncertainty\":[3,4,");
    assertThat(loop(feed)).as("a still robot sends nothing").isEqualTo("{}");
    modules[0] = new SwerveModuleVelocity(2.0, Rotation2d.fromDegrees(45.6));
    assertThat(loop(feed)).as("a wheel's jitter at rest is no news").doesNotContain("module.");
    modules[0] = new SwerveModuleVelocity(2.0, Rotation2d.fromDegrees(46.2));
    assertThat(loop(feed)).as("a degree's turn is").contains("\"module.front-left\":[");
    assertThat(said).isEmpty();
  }

  @Test
  void aSightLineSaysWhyItsMeasurementWasRefusedAndOffersTheCamerasCalibration() {
    Feed feed = world(false);
    frames = List.of(frontLeftSawTag7(Verdict.GATED, 0.8, 45.2));
    String patch = loop(feed);

    assertThat(patch)
        .contains(
            "\"sight-lines\":[{\"id\":\"ray.front-left.7.bad\",\"kind\":\"line\","
                + "\"frame\":\"field\",\"style\":\"bad dashed\",\"inspect\":true,"
                + "\"between\":[\"camera.front-left\",\"tag.7\"]}]");
    Inspection said = scenes.inspect(WorldView.ID, "ray.front-left.7.bad").orElseThrow();
    assertThat(said.title()).isEqualTo("front-left → tag 7");
    assertThat(said.details())
        .extracting(Inspection.Detail::value)
        .contains(
            "rejected: gate: 45.2 > 13.8",
            "0.80 m from the estimate, before this measurement moved it",
            "45.2");
    assertThat(said.actions()).containsExactly("Calibrate front-left");

    // A good camera's innovation is millimeters, its χ² hundredths: shown, not rounded to nothing.
    frames = List.of(frontLeftSawTag7(Verdict.ACCEPTED, 0.0058, 0.0189));
    loop(feed);
    assertThat(scenes.inspect(WorldView.ID, "ray.front-left.7.ok").orElseThrow().details())
        .extracting(Inspection.Detail::value)
        .contains("6 mm from the estimate, before this measurement moved it", "0.019");

    for (int i = 0; i < 10; i++) {
      loop(feed); // the line goes after a moment without a measurement
    }
    assertThat(JSON.toJson(feed.sent())).doesNotContain("ray.front-left.7");
    assertThat(scenes.inspect(WorldView.ID, "ray.front-left.7.bad"))
        .as("still answered for a while after")
        .isPresent();
  }

  /** The front-left camera's single-tag measurement from tag 7, as an update judged it. */
  private static FrameVerdict frontLeftSawTag7(
      Verdict verdict, double innovationMeters, double chiSquared) {
    double gate = 13.8;
    return FrameVerdict.builder()
        .camera("front-left")
        .captureNanos(1)
        .verdict(verdict)
        .threshold(gate)
        .value(chiSquared)
        .tags(
            List.of(
                TagVerdict.builder()
                    .tag(7)
                    .verdict(verdict)
                    .innovationMeters(innovationMeters)
                    .chiSquared(chiSquared)
                    .threshold(gate)
                    .value(chiSquared)
                    .build()))
        .measured(
            Optional.of(
                FrameVerdict.Measured.builder()
                    .pose(new Pose2d(3.2, 4.1, Rotation2d.fromDegrees(30)))
                    .solver(VisionMeasurement.Solver.SINGLE_TAG)
                    .innovationMeters(innovationMeters)
                    .chiSquared(chiSquared)
                    .build()))
        .build();
  }

  @Test
  void aClickOnACameraSaysHowItsMountedAndOnTheEllipseHowSureTheEstimateIs() {
    Feed feed = world(false);
    loop(feed);

    Inspection camera = scenes.inspect(WorldView.ID, "camera.front-left.view").orElseThrow();
    assertThat(camera.title()).isEqualTo("Camera front-left");
    assertThat(camera.details()).extracting(Inspection.Detail::label).contains("Mounted", "Sees");
    assertThat(scenes.inspect(WorldView.ID, "uncertainty.ellipse").orElseThrow().details())
        .extracting(Inspection.Detail::value)
        .contains("tracking");
    assertThat(scenes.inspect(WorldView.ID, "tag.7.square")).isPresent();
    assertThat(scenes.inspect(WorldView.ID, "surface.0")).isPresent();

    estimate = estimate(estimate.pose(), LocalizationEstimate.Mode.LOST);
    assertThat(loop(feed)).contains("\"styles\":{\"uncertainty.ellipse\":\"bad dashed\"}");
  }

  @Test
  void whatALocalizerDoesNotReportIsSaidSoAndNeverDrawn() {
    options = Options.NONE_REPORTED;
    Feed feed = world(false);
    String first = loop(feed);

    assertThat(first)
        .as("no ellipse of any size")
        .contains("\"uncertainty.major\":0")
        .contains("\"uncertainty.minor\":0");
    assertThat(scenes.inspect(WorldView.ID, "uncertainty.ellipse").orElseThrow().details())
        .extracting(Inspection.Detail::label, Inspection.Detail::value)
        .contains(
            tuple("Based on", "not reported by this localizer"),
            tuple("Position, 1σ", "not reported by this localizer"),
            tuple("Heading, 1σ", "not reported by this localizer"),
            tuple("Cameras", "not reported by this localizer"),
            tuple("Drawn", "nothing: the localizer reports no uncertainty"));

    // A measurement given to it is drawn as neither accepted nor rejected, and says why.
    frames = List.of(frontLeftSawTag7(Verdict.GIVEN, Double.NaN, Double.NaN));
    assertThat(loop(feed))
        .contains("\"id\":\"ray.front-left.7.given\"")
        .contains("\"style\":\"muted\"");
    assertThat(scenes.inspect(WorldView.ID, "ray.front-left.7.given").orElseThrow().details())
        .extracting(Inspection.Detail::value)
        .contains("not reported by this localizer: the measurement was given to it");
    assertThat(said).isEmpty();
  }

  /**
   * A switch and a rangefinder are drawn where they're mounted, the rangefinder's cone out to its
   * range, and a click on either says what it is and where, or that where isn't known.
   */
  @Test
  void aClickOnASensorSaysWhatItIsAndWhere() {
    RobotDefinition sensed =
        RobotDefinitionBuilder.builder(ROBOT)
            .sensors(
                List.of(
                    Sensor.Switch.beam("hopper beam", Part.CHASSIS)
                        .at(new Pose3d(0.2, 0, 0.3, Rotation3d.ZERO)),
                    Sensor.Rangefinder.builder()
                        .label("floor range")
                        .part(Part.CHASSIS)
                        .range(Meters.of(1.3))
                        .fieldOfView(Degrees.of(27))
                        .build()))
            .build();
    scenes.add(
        WorldView.scene(() -> RobotSim.FIELD, sensed, readings, null, camera -> Optional.empty()));

    assertThat(JSON.toJson(scenes.descriptions())).contains("\"sensor.1.cone\"");
    Inspection beam = scenes.inspect(WorldView.ID, "sensor.0.block").orElseThrow();
    assertThat(beam.title()).isEqualTo("hopper beam");
    assertThat(beam.details())
        .extracting(Inspection.Detail::value)
        .contains(
            "a beam break", "on chassis, 0.200 m forward, 0.000 m left, 0.300 m up of its origin");
    assertThat(scenes.inspect(WorldView.ID, "sensor.1.cone").orElseThrow().details())
        .extracting(Inspection.Detail::value)
        .contains(
            "a rangefinder: to 1.30 m, in a 27.0° cone",
            "on chassis, where unknown: drawn at its origin");
  }

  /** The gyro's IMU is drawn where its mount puts it, and a click on it says so. */
  @Test
  void aClickOnTheGyroSaysWhereItsMountedAndWhetherItsAccelerometerIsChecked() {
    world(false);

    Inspection gyro = scenes.inspect(WorldView.ID, "imu.gyro.block").orElseThrow();
    assertThat(gyro.title()).isEqualTo("Gyro IMU");
    assertThat(gyro.details())
        .extracting(Inspection.Detail::value)
        .contains(
            "0.000 m forward, 0.000 m left, 0.076 m up",
            "roll 0.0°, pitch 0.0°, yaw 0.0° from level with its x forward",
            "checked: reads along the robot's axes");
  }

  @Test
  void camerasAreNamedOffTheRobotAndOffEachOther() {
    double halfLength = ROBOT.body().bumperLength().in(Meters) / 2;
    double halfWidth = ROBOT.body().bumperWidth().in(Meters) / 2;
    // The template's four, and two more facing the same way from one place.
    List<CameraConfig> cameras = new ArrayList<>(ROBOT.cameras());
    CameraConfig back = ROBOT.cameras().getLast();
    cameras.add(CameraConfigBuilder.builder(back).name("back-2").build());
    cameras.add(CameraConfigBuilder.builder(back).name("back-3").yaw(Degrees.of(90)).build());
    List<Translation3d> places = RobotLayer.labelPlaces(cameras, ROBOT.body());

    for (int i = 0; i < places.size(); i++) {
      Translation3d place = places.get(i);
      assertThat(Math.abs(place.getX()) > halfLength || Math.abs(place.getY()) > halfWidth)
          .as(cameras.get(i).name() + " off the robot, at " + place)
          .isTrue();
      for (int j = 0; j < i; j++) {
        assertThat(place.toTranslation2d().getDistance(places.get(j).toTranslation2d()))
            .as(cameras.get(i).name() + " apart from " + cameras.get(j).name())
            .isGreaterThan(0.3);
      }
    }
  }

  @Test
  void aCamerasFieldOfViewComesFromItsDiagonal() {
    RobotLayer.FieldOfView sees = RobotLayer.fieldOfView(ROBOT.cameras().getFirst().sensor());
    // 80° across the diagonal of a 1280 x 800 image.
    assertThat(Math.toDegrees(sees.acrossRadians())).isBetween(70.0, 71.0);
    assertThat(Math.toDegrees(sees.upAndDownRadians())).isBetween(47.0, 48.5);
  }
}
