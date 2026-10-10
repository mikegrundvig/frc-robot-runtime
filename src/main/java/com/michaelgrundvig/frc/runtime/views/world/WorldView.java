package com.michaelgrundvig.frc.runtime.views.world;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.localization.FrameVerdict;
import com.michaelgrundvig.frc.localization.LocalizationEstimate;
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

/**
 * The robot's world on Pitwall's World view: the field it plays on, the robot where localization
 * believes it is, its wheels, cameras and gyro as configured, how sure the estimate is, and each
 * camera's sight lines to the tags it saw, solid where the estimator accepted the measurement and
 * dashed where it refused it. A click on a sight line says why; a click on a camera says how it's
 * mounted, and shows its full view, which the "Camera views" layer shows for every camera. In the
 * simulator, the simulator's truth stands beside the estimate as a dashed ghost.
 *
 * <p>It draws the code's own beliefs, not a log: a camera's wedge pointing the wrong way is a typo
 * in {@code Robots.java}, found at a glance. Pitwall draws it; this only says what's there.
 */
public final class WorldView {
  /** The scene's id: the page's address for it is {@code #world/world}. */
  public static final String ID = "world";

  /**
   * How far ahead a wheel's drive arrow reaches: where the wheel will be in this long at its speed.
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
    /** The localization estimate, as of the last loop. */
    LocalizationEstimate estimate();

    /** The robot's pitch from the IMU: positive is nose down. */
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
   * @param field the field the robot believes it plays on: the selected venue's, once there is one
   * @param robot what the robot is: its frame, wheels, cameras and IMUs, where each is mounted
   * @param truth where the robot really is, in the simulator; null elsewhere
   * @param cameraAction the Pitwall action that calibrates a camera's mount, by the camera's name,
   *     to end a camera's or a sight line's details in
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
   * What a click on a camera, or its view, says: where it's mounted, what it sees, and its
   * calibration.
   */
  static Inspection camera(CameraConfig camera, Function<String, Optional<String>> cameraAction) {
    Inspection said =
        Inspection.of("Camera " + camera.name())
            .detail(
                "Mounted",
                String.format(
                    Locale.ROOT,
                    "%.3f m forward, %.3f m left, %.3f m up",
                    camera.forward().in(Meters),
                    camera.left().in(Meters),
                    camera.up().in(Meters)))
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
                    camera.sensor().diagonalFieldOfView().in(Degrees),
                    camera.sensor().widthPixels(),
                    camera.sensor().heightPixels(),
                    camera.sensor().sightRange().in(Meters)))
            .detail("From", "the robot's configuration, in Robots.java");
    Optional<String> calibrate = cameraAction.apply(camera.name());
    return calibrate.isPresent() ? said.action(calibrate.get()) : said;
  }

  /** What a click on an IMU says: where it's mounted, how it's turned, and its check. */
  static Inspection imu(String title, ImuMount imu) {
    return Inspection.of(title)
        .detail(
            "Mounted",
            String.format(
                Locale.ROOT,
                "%.3f m forward, %.3f m left, %.3f m up",
                imu.forward().in(Meters),
                imu.left().in(Meters),
                imu.up().in(Meters)))
        .detail(
            "Turned",
            String.format(
                Locale.ROOT,
                "roll %.1f°, pitch %.1f°, yaw %.1f° from level with its x forward",
                Math.toDegrees(imu.orientation().getX()),
                Math.toDegrees(imu.orientation().getY()),
                Math.toDegrees(imu.orientation().getZ())))
        .detail(
            "Accelerometer",
            imu.accelerometerChecked()
                ? "checked: reads along the robot's axes"
                : "not yet checked (docs/calibration.md)")
        .detail("From", "the robot's configuration, in Robots.java");
  }

  /** A switch's or a rangefinder's details: what it is, and where. */
  static Inspection sensor(Sensor.Placed sensor) {
    String is =
        switch (sensor) {
          case Sensor.Switch onSwitch ->
              switch (onSwitch.kind()) {
                case Sensor.Switch.Limit limit ->
                    String.format(
                        Locale.ROOT,
                        "a limit switch at %s's %s end%s",
                        limit.joint().label(),
                        limit.end() == Sensor.Switch.Limit.End.LOWER ? "lower" : "upper",
                        limit
                            .position()
                            .map(at -> String.format(Locale.ROOT, ", at %.3f", at))
                            .orElse(", where unknown"));
                case Sensor.Switch.Beam beam -> "a beam break";
                case Sensor.Switch.Contact contact -> "a switch";
              };
          case Sensor.Rangefinder rangefinder ->
              String.format(
                  Locale.ROOT,
                  "a rangefinder: to %.2f m, in a %.1f° cone",
                  rangefinder.range().in(Meters),
                  rangefinder.fieldOfView().in(Degrees));
        };
    String mounted =
        sensor
            .pose()
            .map(
                pose ->
                    String.format(
                        Locale.ROOT,
                        "on %s, %.3f m forward, %.3f m left, %.3f m up of its origin",
                        sensor.part().label(),
                        pose.getX(),
                        pose.getY(),
                        pose.getZ()))
            .orElse("on " + sensor.part().label() + ", where unknown: drawn at its origin");
    return Inspection.of(sensor.label())
        .detail("Is", is)
        .detail("Mounted", mounted)
        .detail("From", "the robot's definition, in Robots.java");
  }

  /** A tag's frame on the page. */
  static String tagFrame(int id) {
    return "tag." + id;
  }

  /** A number to two places, for details. */
  static String meters(double meters) {
    return Double.isFinite(meters) ? String.format(Locale.ROOT, "%.2f m", meters) : "not measured";
  }

  /**
   * A distance that's often small, such as a camera's innovation: to the millimeter under 10 cm,
   * where two places of meters would show a few millimeters as 0.00 m.
   */
  static String offset(double meters) {
    return Double.isFinite(meters) && Math.abs(meters) < 0.1
        ? String.format(Locale.ROOT, "%.0f mm", meters * 1000)
        : meters(meters);
  }

  /** A χ² to three figures, or to the whole number from 100: a small one isn't shown as 0.0. */
  static String chiSquared(double value) {
    String format =
        Math.abs(value) >= 100
            ? "%.0f"
            : Math.abs(value) >= 10 ? "%.1f" : Math.abs(value) >= 1 ? "%.2f" : "%.3f";
    return String.format(Locale.ROOT, format, value);
  }
}
