package com.michaelgrundvig.frc.toolconfigs.advantagescope;

import static com.michaelgrundvig.frc.toolconfigs.advantagescope.GlbReader.list;
import static com.michaelgrundvig.frc.toolconfigs.advantagescope.GlbReader.map;
import static com.michaelgrundvig.frc.toolconfigs.advantagescope.GlbReader.number;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfigBuilder;
import com.michaelgrundvig.frc.world.robot.Drivetrain;
import com.michaelgrundvig.frc.world.tools.RobotCatalog;
import com.michaelgrundvig.frc.world.tools.RobotDescription;
import io.avaje.jsonb.Jsonb;
import java.io.IOException;
import java.lang.reflect.RecordComponent;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.TreeSet;
import java.util.stream.Stream;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * The AdvantageScope writer, read back as AdvantageScope reads it: every model a valid glTF, the
 * config in the custom-assets format of its release, and every part where the configuration puts it
 * once AdvantageScope's own transforms are applied (worked here with WPILib's rotations, not the
 * writer's own math).
 */
class AdvantageScopeAssetsTest {
  /** Two cameras, one turned every way: a frame longer than wide, so x and y can't swap unseen. */
  private static final RobotDescription ROBOT =
      new RobotDescription(
          "test",
          TestRobot.body(),
          TestRobot.drivetrain(),
          List.of(
              CameraConfigBuilder.builder(
                      TestRobot.camera("front", new Translation3d(0.25, 0.2, 0.5)))
                  .roll(Degrees.of(5))
                  .tiltUp(Degrees.of(15))
                  .yaw(Degrees.of(30))
                  .build(),
              CameraConfigBuilder.builder(
                      TestRobot.camera("back", new Translation3d(-0.3, -0.1, 0.4)))
                  .tiltUp(Degrees.of(-10))
                  .yaw(Degrees.of(180))
                  .build()),
          Optional.empty());

  private static final Map<String, byte[]> FILES = AdvantageScopeAssets.files(ROBOT);

  @Test
  void eachRobotHasTwoFoldersAndEveryModelIsAValidGltf() {
    assertThat(FILES.keySet())
        .containsExactly(
            "Robot_test/GENERATED.md",
            "Robot_test/config.json",
            "Robot_test/model.glb",
            "Robot_test/model_0.glb",
            "Robot_test/model_1.glb",
            "Robot_test/model_2.glb",
            "Robot_test/model_3.glb",
            "Robot_test/model_4.glb",
            "Robot_test_NoCones/GENERATED.md",
            "Robot_test_NoCones/config.json",
            "Robot_test_NoCones/model.glb",
            "Robot_test_NoCones/model_0.glb",
            "Robot_test_NoCones/model_1.glb",
            "Robot_test_NoCones/model_2.glb",
            "Robot_test_NoCones/model_3.glb");
    FILES.forEach(
        (path, bytes) -> {
          if (path.endsWith(".glb")) {
            GlbReader model = GlbReader.read(bytes);
            assertThat(model.problems()).as(path).isEmpty();
            assertThat(map(model.json().get("asset")).get("generator"))
                .as(path + " says it's generated")
                .asString()
                .contains("do not edit");
          }
        });
  }

  @Test
  void theSameRobotGivesTheSameBytes() {
    Map<String, byte[]> again = AdvantageScopeAssets.files(ROBOT);

    assertThat(again.keySet()).isEqualTo(FILES.keySet());
    FILES.forEach((path, bytes) -> assertThat(again.get(path)).as(path).isEqualTo(bytes));
  }

  /** Only the keys AdvantageScope's parseRobot reads at its release, with their types. */
  @Test
  void theConfigIsAdvantageScopesFormat() {
    Map<String, Object> config = config("Robot_test");

    assertThat(config.keySet())
        .containsExactly(
            "name",
            "isFTC",
            "disableSimplification",
            "rotations",
            "position",
            "cameras",
            "components");
    assertThat(config.get("name")).isEqualTo("test (schematic)");
    assertThat(config.get("isFTC")).isEqualTo(false);
    assertThat(config.get("disableSimplification")).isEqualTo(true);
    assertThat(numbers(config.get("position"))).containsExactly(0.0, 0.0, 0.0);
    List<Object> cameras = list(config.get("cameras"));
    assertThat(cameras).hasSize(2);
    for (Object camera : cameras) {
      assertThat(map(camera).keySet())
          .containsExactly("name", "rotations", "position", "resolution", "fov");
    }
    Map<String, Object> front = map(cameras.get(0));
    assertThat(front.get("name")).isEqualTo("front");
    assertThat(numbers(front.get("position"))).containsExactly(0.25, 0.2, 0.5);
    assertThat(numbers(front.get("resolution"))).containsExactly(1280.0, 800.0);
    // An 80° diagonal across 1280 x 800 pixels sees 47.95° from top to bottom, which
    // AdvantageScope's view takes as fov over the aspect ratio (1.6).
    double vertical =
        2 * Math.toDegrees(Math.atan(Math.tan(Math.toRadians(40)) * 800 / Math.hypot(1280, 800)));
    assertThat(vertical).isCloseTo(47.95, within(0.01));
    assertThat(number(front.get("fov")) / 1.6).isCloseTo(vertical, within(1e-5));
    List<Object> components = list(config.get("components"));
    assertThat(components).as("four wheels, then the fields of view").hasSize(5);
    for (Object component : components) {
      assertThat(map(component).keySet()).containsExactly("zeroedRotations", "zeroedPosition");
    }
    assertThat(config("Robot_test_NoCones").get("name")).isEqualTo("test (schematic, no cones)");
    assertThat(list(config("Robot_test_NoCones").get("cameras"))).isEmpty();
    assertThat(list(config("Robot_test_NoCones").get("components"))).hasSize(4);
  }

  /** A camera's fixed view looks along its optical axis, as WPILib's mount says it points. */
  @Test
  void eachCamerasFixedViewIsItsMount() {
    for (int i = 0; i < ROBOT.cameras().size(); i++) {
      CameraConfig camera = ROBOT.cameras().get(i);
      Map<String, Object> view = map(list(config("Robot_test").get("cameras")).get(i));

      Rotation3d turned = rotations(view.get("rotations"));

      assertThat(turned.relativeTo(camera.robotToCamera().getRotation()).getAngle())
          .as(camera.name())
          .isCloseTo(0, within(1e-9));
    }
  }

  /** The base model, in AdvantageScope's frame: upright, its bumpers around the frame. */
  @Test
  void theModelStandsInTheRobotsFrame() {
    GlbReader model = GlbReader.read(file("Robot_test/model.glb"));
    Rotation3d toRobot = rotations(config("Robot_test").get("rotations"));
    Body body = ROBOT.body();
    double halfLength = body.bumperLength().in(Meters) / 2;
    double halfWidth = body.bumperWidth().in(Meters) / 2;

    double[][] bumpers = bounds(model.mesh("bumpers").positions(), toRobot, Pose3d.ZERO);
    double[][] frame = bounds(model.mesh("frame").positions(), toRobot, Pose3d.ZERO);
    double[][] front = bounds(model.mesh("front").positions(), toRobot, Pose3d.ZERO);

    assertThat(bumpers[0])
        .containsExactly(new double[] {-halfLength, -halfWidth, 0.0635}, within(1e-6));
    assertThat(bumpers[1])
        .containsExactly(new double[] {halfLength, halfWidth, 0.14605}, within(1e-6));
    assertThat(frame[1][0]).isCloseTo(body.frameLength().in(Meters) / 2, within(1e-6));
    assertThat(frame[1][1]).isCloseTo(body.frameWidth().in(Meters) / 2, within(1e-6));
    assertThat(frame[1][2]).isCloseTo(body.overallHeight().in(Meters), within(1e-6));
    assertThat(front[0][2])
        .as("the arrow on the frame's top")
        .isCloseTo(body.overallHeight().in(Meters), within(1e-6));
    assertThat(front[1][0]).as("pointing forward").isGreaterThan(-front[0][0]);
    assertThat(model.mesh("frame").color().get(3)).as("see-through").isLessThan(0.75);
    assertThat(model.mesh("bumpers").color().get(3)).isEqualTo(1.0);
    assertThat(model.meshes()).extracting(GlbReader.ReadMesh::name).doesNotContain("imu");
  }

  /**
   * Each camera's field of view starts at its lens and runs along its optical axis to its sight
   * range, as AdvantageScope places a component with no logged pose.
   */
  @Test
  void eachConeRunsFromItsLensAlongItsMount() {
    GlbReader cones = GlbReader.read(file("Robot_test/model_4.glb"));
    Rotation3d toRobot = rotations(config("Robot_test").get("rotations"));

    for (CameraConfig camera : ROBOT.cameras()) {
      List<Translation3d> corners =
          corners(cones.mesh("field of view " + camera.name()).positions(), toRobot);
      Translation3d lens = camera.position();
      Translation3d apex =
          corners.stream()
              .min((a, b) -> Double.compare(a.getDistance(lens), b.getDistance(lens)))
              .orElseThrow();
      List<Translation3d> far = corners.stream().filter(c -> c.getDistance(lens) > 1).toList();
      Translation3d farCenter =
          far.stream().reduce(Translation3d.ZERO, Translation3d::plus).div(far.size());

      Translation3d expected =
          new Translation3d(camera.sightRangeOrDefault().in(Meters), 0, 0)
              .rotateBy(camera.robotToCamera().getRotation());
      assertThat(apex.getDistance(lens)).as(camera.name() + "'s lens").isLessThan(1e-6);
      assertThat(farCenter.minus(lens).getDistance(expected))
          .as(camera.name() + "'s optical axis, out to its sight range")
          .isLessThan(1e-5);
    }
    assertThat(cones.mesh("field of view front").color().get(3)).isLessThan(0.75);
  }

  /**
   * A wheel's model is brought to the origin by its zeroed transform, then placed by its logged
   * pose: at its module, turned as the module points (AdvantageScope's RobotManager composes the
   * robot's pose, the logged pose, then the zeroed one).
   */
  @Test
  void eachWheelGoesWhereItsLoggedPosePutsIt() {
    Map<String, Object> config = config("Robot_test");
    Translation2d[] places = ROBOT.drivetrain().moduleTranslations();
    double radius = ROBOT.drivetrain().module(0).wheelRadius().in(Meters);

    for (int i = 0; i < 4; i++) {
      GlbReader wheel = GlbReader.read(file("Robot_test/model_" + i + ".glb"));
      Map<String, Object> component = map(list(config.get("components")).get(i));
      Rotation3d zeroed = rotations(component.get("zeroedRotations"));
      Translation3d shift = translation(component.get("zeroedPosition"));
      // As Drive logs it: the wheel's center, turned to the module's angle (a quarter turn left).
      Pose3d logged =
          new Pose3d(places[i].getX(), places[i].getY(), radius, new Rotation3d(0, 0, Math.PI / 2));

      Translation3d center =
          centroid(corners(wheel.meshes().get(0).positions(), zeroed, shift, logged));
      double[][] arrow =
          bounds(
              wheel.mesh("wheel arrow " + AdvantageScopeAssets.WHEELS.get(i)).positions(),
              zeroed,
              shift,
              logged);

      assertThat(center.getDistance(new Translation3d(places[i].getX(), places[i].getY(), radius)))
          .as("wheel " + i)
          .isLessThan(1e-6);
      assertThat(arrow[1][1] - places[i].getY())
          .as("its arrow points left now")
          .isGreaterThan(places[i].getY() - arrow[0][1]);
    }
  }

  /** Without a logged pose, a wheel stands where it was drawn, pointing forward. */
  @Test
  void aWheelWithoutALoggedPoseStandsAtItsModule() {
    Rotation3d toRobot = rotations(config("Robot_test").get("rotations"));
    Translation2d place = ROBOT.drivetrain().moduleTranslations()[1];

    Translation3d center =
        centroid(
            corners(
                GlbReader.read(file("Robot_test/model_1.glb")).meshes().get(0).positions(),
                toRobot));

    assertThat(center.toTranslation2d().getDistance(place)).isLessThan(1e-6);
  }

  @Test
  void anImuIsDrawnWhereItsMountSays() {
    RobotDescription withImu =
        new RobotDescription(
            "imu",
            ROBOT.body(),
            ROBOT.drivetrain(),
            List.of(),
            Optional.of(new Translation3d(0.1, -0.05, 0.2)));

    GlbReader model =
        GlbReader.read(
            Objects.requireNonNull(AdvantageScopeAssets.files(withImu).get("Robot_imu/model.glb")));

    Translation3d at =
        centroid(
            corners(
                model.mesh("imu").positions(),
                rotations(config(withImu, "Robot_imu").get("rotations"))));
    assertThat(at.getDistance(new Translation3d(0.1, -0.05, 0.2))).isLessThan(1e-6);
    assertThat(AdvantageScopeAssets.files(withImu))
        .as("no cameras, no fields of view")
        .doesNotContainKey("Robot_imu/model_4.glb");
  }

  /**
   * What the model draws and what it drops, pinned: a value added to a record it reads fails here
   * until it's one or the other, and the list in the writer's Javadoc says why.
   */
  @Test
  void everyValueItReadsIsDrawnOrDropped() {
    assertThat(AdvantageScopeAssets.DROPPED)
        .containsExactly(
            "Body.weight",
            "Body.cg",
            "Body.inertia",
            "Drivetrain.Module.driveMotor",
            "Drivetrain.Module.steerMotor",
            "Drivetrain.Module.driveGearing",
            "Drivetrain.Module.steerGearing",
            "Drivetrain.Module.wheelCoefficientOfFriction",
            "Drivetrain.Module.driveInertia",
            "Drivetrain.Module.steerInertia",
            "Drivetrain.Module.couplingRatio",
            "Drivetrain.wiring",
            "Drivetrain.control",
            "CameraConfig.noiseScale",
            "CameraConfig.joint",
            "CameraConfig.mountTolerance",
            "CameraConfig.ImageSensor.frameRate",
            "CameraConfig.ImageSensor.latency");
    Set<String> values = new TreeSet<>();
    for (Class<?> record :
        List.of(
            Body.class,
            Drivetrain.class,
            Drivetrain.Corner.class,
            Drivetrain.Module.class,
            CameraConfig.class,
            CameraConfig.ImageSensor.class)) {
      String name =
          record.getName().substring(record.getPackageName().length() + 1).replace('$', '.');
      for (RecordComponent component : record.getRecordComponents()) {
        values.add(name + "." + component.getName());
      }
    }
    Set<String> sorted = new TreeSet<>(AdvantageScopeAssets.DRAWN);
    sorted.addAll(AdvantageScopeAssets.DROPPED);
    assertThat(sorted).isEqualTo(values);
    assertThat(AdvantageScopeAssets.DRAWN)
        .doesNotContainAnyElementsOf(AdvantageScopeAssets.DROPPED);
  }

  @Test
  void theConfigReadsAsTextAValueToALine() {
    String config = new String(file("Robot_test/config.json"), StandardCharsets.UTF_8);

    assertThat(config).startsWith("{\n  \"name\": \"test (schematic)\",\n");
    assertThat(config)
        .contains("\"position\": [\n        0.25,\n        0.2,\n        0.5\n      ],");
    assertThat(config).as("to six places").contains("\"fov\": 76.722442\n");
    assertThat(config).endsWith("}\n");
    assertThat(new String(file("Robot_test/GENERATED.md"), StandardCharsets.UTF_8))
        .contains("Robots.java");
  }

  /** The build's plugin runs main: every robot the catalog has, or the robots it names. */
  @Test
  void mainWritesEveryRobotOrTheNamedOnes(@TempDir Path every, @TempDir Path out) throws Exception {
    AdvantageScopeAssets.main("--catalog=" + TwoRobots.class.getName(), "--out=" + every);
    assertThat(names(every))
        .containsExactly(
            "Robot_first", "Robot_first_NoCones", "Robot_second", "Robot_second_NoCones");

    AdvantageScopeAssets.main(
        "--catalog=" + TwoRobots.class.getName(), "--out=" + out, "--robot=second");
    assertThat(names(out)).containsExactly("Robot_second", "Robot_second_NoCones");
    assertThat(Files.readAllBytes(out.resolve("Robot_second/model.glb")))
        .isEqualTo(
            AdvantageScopeAssets.files(TwoRobots.robot("second")).get("Robot_second/model.glb"));

    assertThatThrownBy(
            () ->
                AdvantageScopeAssets.main(
                    "--catalog=" + TwoRobots.class.getName(), "--out=" + out, "--robot=third"))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessage("no robot named third; the robots are first, second");
    assertThatThrownBy(() -> AdvantageScopeAssets.main("--out=" + out))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessageContaining("--catalog=");
    assertThatThrownBy(() -> AdvantageScopeAssets.main("--colour=blue"))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessageStartingWith("unknown argument --colour=blue");
  }

  @Test
  void twoRobotsCantShareAFolder(@TempDir Path out) {
    RobotDescription spaced = TwoRobots.robot("a b");
    RobotDescription underscored = TwoRobots.robot("a_b");

    assertThat(AdvantageScopeAssets.folder("a b")).isEqualTo("Robot_a_b");
    assertThatThrownBy(() -> AdvantageScopeAssets.write(List.of(spaced, underscored), out))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessage("the robots a b and a_b would share a folder");
  }

  /** A catalog for main to load by name, as the build names the robot program's. */
  public static final class TwoRobots implements RobotCatalog {
    public TwoRobots() {}

    static RobotDescription robot(String name) {
      return new RobotDescription(
          name, TestRobot.body(), TestRobot.drivetrain(), List.of(), Optional.empty());
    }

    @Override
    public List<RobotDescription> all() {
      return List.of(robot("first"), robot("second"));
    }
  }

  private static byte[] file(String path) {
    return Objects.requireNonNull(FILES.get(path), path);
  }

  private static Map<String, Object> config(String folder) {
    return config(ROBOT, folder);
  }

  @SuppressWarnings("unchecked")
  private static Map<String, Object> config(RobotDescription robot, String folder) {
    byte[] bytes =
        Objects.requireNonNull(AdvantageScopeAssets.files(robot).get(folder + "/config.json"));
    String text = new String(bytes, StandardCharsets.UTF_8);
    return (Map<String, Object>) Jsonb.instance().type(Object.class).fromJson(text);
  }

  private static List<Double> numbers(@Nullable Object value) {
    return list(value).stream().map(GlbReader::number).toList();
  }

  /**
   * A config's rotations as AdvantageScope applies them (geometry.ts,
   * rotationSequenceToQuaternion): each about the fixed axes, in order, each premultiplying the
   * ones before.
   */
  private static Rotation3d rotations(@Nullable Object sequence) {
    Rotation3d total = Rotation3d.ZERO;
    for (Object step : list(sequence)) {
      Map<String, Object> rotation = map(step);
      double radians = Math.toRadians(number(rotation.get("degrees")));
      Rotation3d about =
          switch (String.valueOf(rotation.get("axis"))) {
            case "x" -> new Rotation3d(radians, 0, 0);
            case "y" -> new Rotation3d(0, radians, 0);
            case "z" -> new Rotation3d(0, 0, radians);
            default -> throw new AssertionError("no axis " + rotation.get("axis"));
          };
      total = total.rotateBy(about);
    }
    return total;
  }

  private static Translation3d translation(@Nullable Object position) {
    List<Double> xyz = numbers(position);
    return new Translation3d(xyz.get(0), xyz.get(1), xyz.get(2));
  }

  /** A model's corners, turned from glTF's axes to the robot's. */
  private static List<Translation3d> corners(float[] positions, Rotation3d toRobot) {
    return corners(positions, toRobot, Translation3d.ZERO, Pose3d.ZERO);
  }

  /** A component's corners: turned and moved by its zeroed transform, then placed by its pose. */
  private static List<Translation3d> corners(
      float[] positions, Rotation3d zeroed, Translation3d shift, Pose3d logged) {
    List<Translation3d> corners = new ArrayList<>();
    for (int i = 0; i < positions.length; i += 3) {
      Translation3d local =
          new Translation3d(positions[i], positions[i + 1], positions[i + 2])
              .rotateBy(zeroed)
              .plus(shift);
      corners.add(local.rotateBy(logged.getRotation()).plus(logged.getTranslation()));
    }
    return corners;
  }

  private static double[][] bounds(float[] positions, Rotation3d toRobot, Pose3d logged) {
    return bounds(positions, toRobot, Translation3d.ZERO, logged);
  }

  /** The smallest and largest x, y and z of a mesh's corners. */
  private static double[][] bounds(
      float[] positions, Rotation3d zeroed, Translation3d shift, Pose3d logged) {
    return extent(corners(positions, zeroed, shift, logged));
  }

  private static double[][] extent(List<Translation3d> corners) {
    double[] low = {Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE};
    double[] high = {-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};
    for (Translation3d corner : corners) {
      double[] xyz = {corner.getX(), corner.getY(), corner.getZ()};
      for (int axis = 0; axis < 3; axis++) {
        low[axis] = Math.min(low[axis], xyz[axis]);
        high[axis] = Math.max(high[axis], xyz[axis]);
      }
    }
    return new double[][] {low, high};
  }

  /** The middle of a solid's corners: for a wheel or a box, its center. */
  private static Translation3d centroid(List<Translation3d> corners) {
    double[][] box = extent(corners);
    return new Translation3d(
        (box[0][0] + box[1][0]) / 2, (box[0][1] + box[1][1]) / 2, (box[0][2] + box[1][2]) / 2);
  }

  private static List<String> names(Path folder) throws IOException {
    try (Stream<Path> files = Files.list(folder)) {
      return files.map(path -> path.getFileName().toString()).sorted().toList();
    }
  }
}
