package com.michaelgrundvig.frc.runtime.views.world;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.localization.FrameVerdict;
import com.michaelgrundvig.frc.localization.LocalizerReport;
import com.michaelgrundvig.frc.pitwall.scene.Inspection;
import com.michaelgrundvig.frc.pitwall.scene.Scene;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.ImuMount;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.units.measure.Distance;

/**
 * The robot's world on Pitwall's World view. It has a layer for each of:
 *
 * <ul>
 *   <li>the field the robot plays on ({@link FieldLayer});
 *   <li>the robot where localization believes it is, with its wheels, cameras, sensors and gyro as
 *       configured ({@link RobotLayer});
 *   <li>each camera's full view, which is off until switched on ({@link CameraViews});
 *   <li>how sure the estimate is ({@link Uncertainty});
 *   <li>each camera's sight lines to the tags it saw, solid where the estimator accepted the
 *       measurement and dashed where it refused it ({@link SightLines});
 *   <li>in the simulator, the simulator's truth beside the estimate, as a dashed ghost ({@link
 *       TruthLayer}).
 * </ul>
 *
 * <p>A click on a sight line says why its measurement was accepted or refused. A click on a camera
 * says how it's mounted, and shows its full view.
 *
 * <p>The view draws the code's own beliefs, not a log. A camera's wedge pointing the wrong way is a
 * typo in {@code Robots.java}, found at a glance. Pitwall does the drawing, and this only says what
 * is there.
 */
public final class WorldView {
  /** The scene's id: the page's address for it is {@code #world/world}. */
  public static final String ID = "world";

  /**
   * How far ahead a wheel's drive arrow reaches. Its tip is where the wheel will be after this long
   * at its speed.
   */
  static final double ARROW_SECONDS = 0.2;

  /**
   * The swerve modules' names on the page, in the order the drivetrain lists them ({@link
   * RobotDefinition#moduleTranslations}).
   */
  static final List<String> MODULES =
      List.of("front-left", "front-right", "back-left", "back-right");

  /** What the view reads in the robot loop, while a page shows it. */
  public interface Readings {
    /** The pose, with what the localizer reports about it, as of the last loop. */
    LocalizerReport report();

    /** The robot's pitch from the IMU. Positive is nose down. */
    double pitchRadians();

    /** The robot's roll from the IMU. */
    double rollRadians();

    /**
     * Each module's measured wheel speed and angle, in {@link RobotDefinition#moduleTranslations}'s
     * order.
     */
    SwerveModuleVelocity[] modules();

    /**
     * What became of each camera image's measurement in the last loop's update, by tag ({@code
     * LocalizationOutput.frames}).
     */
    List<FrameVerdict> frames();
  }

  private WorldView() {}

  /**
   * The world as a scene for Pitwall: {@code pitwall.scenes().add(WorldView.scene(...))}.
   *
   * @param field the field the robot believes it plays on, which is the selected venue's once there
   *     is one
   * @param robot what the robot is: its frame, wheels, cameras and IMUs, and where each is mounted
   * @param truth where the robot really is, in the simulator, and null elsewhere
   * @param cameraAction the Pitwall action that calibrates a camera's mount, by the camera's name.
   *     A camera's details and a sight line's details end in it.
   */
  public static Scene scene(
      Supplier<FieldModel> field,
      RobotDefinition robot,
      Readings readings,
      @Nullable Supplier<Pose3d> truth,
      Function<String, Optional<String>> cameraAction) {
    Scene scene =
        Scene.named(ID, "World")
            .layer(new FieldLayer(field))
            .layer(new RobotLayer(field.get(), robot, readings, cameraAction))
            .layer(new CameraViews(robot.cameras(), cameraAction), Scene.Visibility.OFF)
            .layer(new Uncertainty(readings))
            .layer(new SightLines(field, robot.cameras(), readings, cameraAction));
    if (truth != null) {
      scene.layer(new TruthLayer(robot.body(), truth));
    }
    return scene.focus(RobotLayer.FRAME);
  }

  /** A camera's frame on the page, by its id ({@link CameraConfig#id()}). */
  static String cameraFrame(String cameraId) {
    return "camera." + cameraId;
  }

  /**
   * What a click on a camera, or on its view, says: where it's mounted, how it's turned, what it
   * sees, and the action that calibrates its mount, if the page has one.
   */
  static Inspection camera(CameraConfig camera, Function<String, Optional<String>> cameraAction) {
    CameraConfig.Sensor sensor = camera.sensor();
    Inspection said =
        Inspection.of("Camera " + camera.name())
            .detail("Mounted", mountedAt(camera.forward(), camera.left(), camera.up()))
            .detail(
                "Turned",
                String.format(
                    Locale.ROOT,
                    "yaw %.1f°, tilted up %.1f°, roll %.1f°",
                    camera.yaw().in(Degrees),
                    camera.tiltUp().in(Degrees),
                    camera.roll().in(Degrees)))
            .detail(
                "Sees",
                String.format(
                    Locale.ROOT,
                    "%.0f° across its diagonal, %d × %d, out to %.0f m",
                    sensor.diagonalFieldOfView().in(Degrees),
                    sensor.widthPixels(),
                    sensor.heightPixels(),
                    sensor.sightRange().in(Meters)))
            .detail("From", "the robot's configuration, in Robots.java");
    Optional<String> calibrate = cameraAction.apply(camera.name());
    if (calibrate.isEmpty()) {
      return said;
    }
    return said.action(calibrate.get());
  }

  /**
   * What a click on an IMU says: where it's mounted, how it's turned, and whether its accelerometer
   * has been checked against the robot's axes.
   */
  static Inspection imu(String title, ImuMount imu) {
    String accelerometer = "not yet checked (docs/calibration.md)";
    if (imu.accelerometerChecked()) {
      accelerometer = "checked: reads along the robot's axes";
    }
    return Inspection.of(title)
        .detail("Mounted", mountedAt(imu.forward(), imu.left(), imu.up()))
        .detail(
            "Turned",
            String.format(
                Locale.ROOT,
                "roll %.1f°, pitch %.1f°, yaw %.1f° from level with its x forward",
                Math.toDegrees(imu.orientation().getX()),
                Math.toDegrees(imu.orientation().getY()),
                Math.toDegrees(imu.orientation().getZ())))
        .detail("Accelerometer", accelerometer)
        .detail("From", "the robot's configuration, in Robots.java");
  }

  /** Where something is mounted on the robot, from the robot's center on the floor. */
  private static String mountedAt(Distance forward, Distance left, Distance up) {
    return String.format(
        Locale.ROOT,
        "%.3f m forward, %.3f m left, %.3f m up",
        forward.in(Meters),
        left.in(Meters),
        up.in(Meters));
  }

  /** A switch's or a rangefinder's details: what it is, and where it's mounted. */
  static Inspection sensor(Sensor.Placed sensor) {
    return Inspection.of(sensor.label())
        .detail("Is", whatItIs(sensor))
        .detail("Mounted", whereItIsMounted(sensor))
        .detail("From", "the robot's definition, in Robots.java");
  }

  private static String whatItIs(Sensor.Placed sensor) {
    return switch (sensor) {
      case Sensor.Switch onSwitch -> whatKindOfSwitch(onSwitch.kind());
      case Sensor.Rangefinder rangefinder ->
          String.format(
              Locale.ROOT,
              "a rangefinder: to %.2f m, in a %.1f° cone",
              rangefinder.range().in(Meters),
              rangefinder.fieldOfView().in(Degrees));
    };
  }

  private static String whatKindOfSwitch(Sensor.Switch.Kind kind) {
    return switch (kind) {
      case Sensor.Switch.Limit limit -> limitSwitch(limit);
      case Sensor.Switch.Beam beam -> "a beam break";
      case Sensor.Switch.Contact contact -> "a switch";
    };
  }

  /** A limit switch: which joint's, at which end, and where along the joint it trips, if known. */
  private static String limitSwitch(Sensor.Switch.Limit limit) {
    String end = "upper";
    if (limit.end() == Sensor.Switch.Limit.End.LOWER) {
      end = "lower";
    }
    String tripsAt = ", where unknown";
    if (limit.position().isPresent()) {
      tripsAt = String.format(Locale.ROOT, ", at %.3f", limit.position().get());
    }
    return String.format(
        Locale.ROOT, "a limit switch at %s's %s end%s", limit.joint().label(), end, tripsAt);
  }

  /** Which part a sensor rides on, and where on it, if the robot's definition says. */
  private static String whereItIsMounted(Sensor.Placed sensor) {
    String part = sensor.part().label();
    if (sensor.pose().isEmpty()) {
      return "on " + part + ", where unknown: drawn at its origin";
    }
    Pose3d pose = sensor.pose().get();
    return String.format(
        Locale.ROOT,
        "on %s, %.3f m forward, %.3f m left, %.3f m up of its origin",
        part,
        pose.getX(),
        pose.getY(),
        pose.getZ());
  }

  /** A tag's frame on the page. */
  static String tagFrame(int id) {
    return "tag." + id;
  }

  /** A distance to two places of meters, for details, or that it wasn't measured. */
  static String meters(double meters) {
    if (!Double.isFinite(meters)) {
      return "not measured";
    }
    return String.format(Locale.ROOT, "%.2f m", meters);
  }

  /**
   * A distance that's often small, such as a camera's innovation. Under 10 cm it is given to the
   * millimeter, because two places of meters would show a few millimeters as 0.00 m.
   */
  static String offset(double meters) {
    boolean small = Double.isFinite(meters) && Math.abs(meters) < 0.1;
    if (!small) {
      return meters(meters);
    }
    return String.format(Locale.ROOT, "%.0f mm", meters * 1000);
  }

  /**
   * A χ² to three figures, or to the whole number from 100 up, so that a small one isn't shown as
   * 0.0.
   */
  static String chiSquared(double value) {
    double size = Math.abs(value);
    String format = "%.3f";
    if (size >= 100) {
      format = "%.0f";
    } else if (size >= 10) {
      format = "%.1f";
    } else if (size >= 1) {
      format = "%.2f";
    }
    return String.format(Locale.ROOT, format, value);
  }
}
