package com.michaelgrundvig.frc.toolconfigs.advantagescope;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.Drivetrain;
import com.michaelgrundvig.frc.world.tools.RobotDescription;
import io.avaje.jsonb.JsonType;
import io.avaje.jsonb.Jsonb;
import java.math.BigDecimal;
import java.math.RoundingMode;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Writes the {@code config.json} that tells AdvantageScope how to place a robot's model: which way
 * up its files are, where its cameras look from, and where each of its moving parts starts. The
 * keys are AdvantageScope's own, from its custom-assets format.
 *
 * <p>Keys are written in the order they are put, and every number to six places, so the file comes
 * out the same, byte for byte, for the same robot.
 */
final class RobotConfigJson {
  /**
   * From glTF's axes to the robot's, as AdvantageScope applies a config's rotations: each about the
   * fixed axes, in order. 90° about x stands glTF's up (+y) along +z; 90° about z then turns its
   * front (+z, now -y) to +x.
   */
  static final List<Map<String, Object>> GLTF_TO_ROBOT = List.of(turn("x", 90), turn("z", 90));

  /** How many places after the point every number is written to. */
  private static final int DECIMAL_PLACES = 6;

  /** Any JSON value: maps in their own order, lists, text, numbers, true or false. */
  private static final JsonType<Object> JSON = Jsonb.instance().type(Object.class);

  private RobotConfigJson() {}

  /**
   * A robot's config.json for one of its two models.
   *
   * @param conesAreDrawn whether this model has the cameras' fields of view as a component, which
   *     it does when it is the model with cones and the robot has a camera to draw one for
   */
  static byte[] write(RobotDescription robot, SchematicModel model, boolean conesAreDrawn) {
    Map<String, Object> config = new LinkedHashMap<>();
    config.put("name", model.modelName(robot.name()));
    config.put("isFTC", false);
    // A schematic's parts are few, and some small (a camera, the IMU): AdvantageScope leaves out
    // small parts of a model to save drawing, unless told not to.
    config.put("disableSimplification", true);
    config.put("rotations", GLTF_TO_ROBOT);
    config.put("position", List.of(0.0, 0.0, 0.0));
    config.put("cameras", cameraViews(robot, model));
    config.put("components", components(robot, conesAreDrawn));
    return (JSON.toJsonPretty(rounded(config)) + "\n").getBytes(StandardCharsets.UTF_8);
  }

  /**
   * The robot's cameras as views to look through. Only the model with cones lists them, so that
   * they are listed once when both models are drawn.
   */
  private static List<Object> cameraViews(RobotDescription robot, SchematicModel model) {
    List<Object> views = new ArrayList<>();
    if (model == SchematicModel.WITH_CONES) {
      for (CameraConfig camera : RobotSchematic.camerasOnChassis(robot)) {
        views.add(cameraView(camera));
      }
    }
    return views;
  }

  /**
   * The models AdvantageScope moves by logged poses, in the order of their files: each wheel, and
   * then the fields of view.
   *
   * <p>A component's zeroed position is where its model has to be moved so that the point its
   * logged pose is about sits at the origin. A wheel is drawn where it sits on the robot, its
   * center a wheel radius above the floor, and its logged pose is its center's, so it is moved back
   * by exactly where its center is. The fields of view are drawn where they belong and have no
   * logged pose, so they are not moved.
   */
  private static List<Object> components(RobotDescription robot, boolean conesAreDrawn) {
    List<Object> components = new ArrayList<>();
    for (Drivetrain.Corner corner : robot.drivetrain().corners()) {
      Translation2d wheel = corner.position();
      double radius = corner.module().wheelRadius().in(Meters);
      components.add(component(List.of(-wheel.getX(), -wheel.getY(), -radius)));
    }
    if (conesAreDrawn) {
      components.add(component(List.of(0.0, 0.0, 0.0)));
    }
    return components;
  }

  private static Map<String, Object> component(List<Double> zeroedPosition) {
    Map<String, Object> component = new LinkedHashMap<>();
    component.put("zeroedRotations", GLTF_TO_ROBOT);
    component.put("zeroedPosition", zeroedPosition);
    return component;
  }

  /**
   * A camera as AdvantageScope's fixed view: its lens, turned as WPILib's {@code Rotation3d(roll,
   * pitch, yaw)} turns it, and its field of view.
   *
   * <p>AdvantageScope's docs call {@code fov} the horizontal field of view, but its renderer takes
   * the view's vertical angle as {@code fov} over the image's aspect ratio (Field3dRendererImpl.ts,
   * at the release this writes for): a linear scaling that's only right for narrow lenses. So this
   * writes the vertical angle times the aspect ratio (76.7° for an 80° diagonal across 1280 x 800,
   * whose true horizontal angle is 70.9°), and the view sees exactly what the camera does. The
   * cones are drawn from the true angles.
   */
  private static Map<String, Object> cameraView(CameraConfig camera) {
    CameraConfig.ImageSensor sensor = camera.sensor();
    Translation3d lens = camera.position();
    // WPILib's pitch is positive nose down, so a camera tilted up has a negative one.
    List<Map<String, Object>> rollPitchYaw =
        List.of(
            turn("x", camera.rollOrDefault().in(Degrees)),
            turn("y", -camera.tiltUp().orElseThrow().in(Degrees)),
            turn("z", camera.yaw().orElseThrow().in(Degrees)));
    double aspectRatio = (double) sensor.widthPixels() / sensor.heightPixels();
    double verticalRadians = 2 * RobotSchematic.halfAngle(camera, sensor.heightPixels());

    Map<String, Object> view = new LinkedHashMap<>();
    view.put("name", camera.name());
    view.put("rotations", rollPitchYaw);
    view.put("position", List.of(lens.getX(), lens.getY(), lens.getZ()));
    view.put("resolution", List.of(sensor.widthPixels(), sensor.heightPixels()));
    view.put("fov", StrictMath.toDegrees(verticalRadians) * aspectRatio);
    return view;
  }

  /**
   * Every number to six places (a micrometer, a millionth of a degree), so a config.json changes
   * only where the robot did, never in a last digit.
   */
  private static Object rounded(Object value) {
    return switch (value) {
      case Double number ->
          BigDecimal.valueOf(number).setScale(DECIMAL_PLACES, RoundingMode.HALF_EVEN).doubleValue();
      case Map<?, ?> map -> {
        Map<Object, Object> copy = new LinkedHashMap<>();
        map.forEach((key, item) -> copy.put(key, rounded(item)));
        yield copy;
      }
      case List<?> list -> list.stream().map(RobotConfigJson::rounded).toList();
      default -> value;
    };
  }

  /** A turn about one of the fixed axes, as a config writes one. */
  private static Map<String, Object> turn(String axis, double degrees) {
    Map<String, Object> turn = new LinkedHashMap<>();
    turn.put("axis", axis);
    turn.put("degrees", degrees);
    return turn;
  }
}
