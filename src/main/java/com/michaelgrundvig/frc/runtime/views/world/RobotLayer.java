package com.michaelgrundvig.frc.runtime.views.world;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import com.michaelgrundvig.frc.pitwall.scene.Layer;
import com.michaelgrundvig.frc.pitwall.scene.SceneBuilder;
import com.michaelgrundvig.frc.pitwall.scene.SceneState;
import com.michaelgrundvig.frc.pitwall.scene.Style;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.ImuMount;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

/**
 * The robot where localization believes it is, as its configuration describes it: its frame and
 * bumpers with an arrow out the front; each wheel at its module, turned to its measured angle, with
 * an arrow where it's driving; each camera at its exact mount, an arrow along its optical axis and
 * the first meter of its view, named off the robot; and the gyro's IMU, a small block where its
 * mount puts it, turned as it's mounted, with an arrow along its own x axis; and each switch and
 * rangefinder where it's mounted, a small block, a rangefinder's cone out to its range. A click on
 * a camera says how it's mounted, offers its calibration, and shows its full view ({@link
 * CameraViews}) until the details close; a click on the IMU says how it's mounted and whether its
 * accelerometer has been checked; a click on a sensor says what it is and where.
 */
final class RobotLayer implements Layer {
  /** The robot's frame on the page: the frame the page's Follow follows. */
  static final String FRAME = "robot";

  /** How tall a bumper is, about: the rules allow 5 in. */
  private static final double BUMPER_HEIGHT = 0.127;

  /** How far above the floor bumpers ride. */
  private static final double BUMPER_CLEARANCE = 0.02;

  /** The gyro's IMU on the page: its frame, turned as it's mounted. */
  private static final String IMU = "imu.gyro";

  /** The IMU's block: about a Pigeon 2's, and AdvantageScope's. */
  private static final double IMU_SIZE = 0.05;

  private static final double IMU_THICKNESS = 0.02;

  /** The arrow along the IMU's own x axis, which shows how it's turned. */
  private static final double IMU_ARROW = 0.1;

  /** A switch's or a rangefinder's block: about a CANrange's, or a limit switch's. */
  private static final double SENSOR_SIZE = 0.03;

  /** A swerve wheel's width, about. */
  private static final double WHEEL_WIDTH = 0.038;

  /** The arrow along a camera's optical axis. */
  private static final double CAMERA_ARROW = 0.3;

  /**
   * How far a camera's view is always drawn: past the bumpers, to show where it points and how wide
   * it sees, and far short of the 7 m that covered the field four times over.
   */
  static final double VIEW_STUB_METERS = 1.0;

  /** How far a camera's name stays clear of the bumpers. */
  private static final double LABEL_CLEARANCE = 0.15;

  /** How far apart two cameras' names stay, center to center. */
  private static final double LABEL_SPACING = 0.35;

  /** How far above its camera a camera's name is drawn. */
  private static final double LABEL_RISE = 0.1;

  /**
   * How far the estimate must move before the page hears of it: a still robot's estimate wanders a
   * few millimeters as each image corrects it, which is no news, and a centimeter is less than a
   * pixel of a field on a laptop.
   */
  static final double DEADBAND_METERS = 0.01;

  static final double DEADBAND_RADIANS = Math.toRadians(0.5);

  /**
   * How far a wheel must turn before the page hears of it: a wheel at rest reads a few tenths of a
   * degree either way as its encoder settles, and a degree is less than a pixel of its drawing.
   */
  static final double STEER_DEADBAND_RADIANS = Math.toRadians(1);

  /** A wheel's frame only turns: the least deadband for its place, a millimeter. */
  private static final double WHEEL_DEADBAND_METERS = 0.001;

  private final FieldModel field;
  private final Body body;

  /** Each wheel's radius, in meters. */
  private final double[] radii;

  private final List<CameraConfig> cameras;

  /** Each camera's mount on the robot, in the cameras' order: on a joint, as the joint rests. */
  private final Transform3d[] mounts;

  /** The switches and rangefinders. */
  private final List<Sensor.Placed> sensors;

  /** Each one's mount on the robot, in their order: on a joint, as the joint rests. */
  private final Transform3d[] sensorMounts;

  private final ImuMount gyro;
  private final WorldView.Readings readings;
  private final Function<String, Optional<String>> cameraAction;
  private final Translation2d[] modules;

  /** Each module's frame and speed on the page, named once, as the loop reads them often. */
  private final String[] moduleFrames;

  private final String[] moduleSpeeds;

  RobotLayer(
      FieldModel field,
      RobotDefinition robot,
      WorldView.Readings readings,
      Function<String, Optional<String>> cameraAction) {
    this.field = field;
    body = robot.body();
    radii = new double[robot.drivetrain().corners().size()];
    for (int i = 0; i < radii.length; i++) {
      radii[i] = robot.drivetrain().module(i).wheelRadius().in(Meters);
    }
    cameras = List.copyOf(robot.cameras());
    // A camera on a joint is drawn where its joint rests: the view doesn't follow the joints yet.
    RobotMath math = robot.math();
    mounts = new Transform3d[cameras.size()];
    for (int i = 0; i < cameras.size(); i++) {
      Pose3d resting = math.cameraPose(cameras.get(i), Map.of());
      mounts[i] = new Transform3d(resting.getTranslation(), resting.getRotation());
    }
    sensors =
        math.sensors().stream()
            .filter(Sensor.Placed.class::isInstance)
            .map(Sensor.Placed.class::cast)
            .toList();
    sensorMounts = new Transform3d[sensors.size()];
    for (int i = 0; i < sensors.size(); i++) {
      Pose3d resting = math.sensorPose(sensors.get(i), Map.of());
      sensorMounts[i] = new Transform3d(resting.getTranslation(), resting.getRotation());
    }
    gyro = robot.imus().gyro();
    this.readings = readings;
    this.cameraAction = cameraAction;
    modules = robot.moduleTranslations();
    int count = Math.min(modules.length, WorldView.MODULES.size());
    moduleFrames = new String[count];
    moduleSpeeds = new String[count];
    for (int i = 0; i < count; i++) {
      moduleFrames[i] = "module." + WorldView.MODULES.get(i);
      moduleSpeeds[i] = "speed." + WorldView.MODULES.get(i);
    }
  }

  @Override
  public String id() {
    return "robot";
  }

  @Override
  public String title() {
    return "Robot";
  }

  @Override
  public void describe(SceneBuilder scene) {
    double height = body.height().in(Meters);
    double bumperLength = body.bumperLength().in(Meters);
    scene.frame(FRAME, SceneBuilder.ROOT).deadband(FRAME, DEADBAND_METERS, DEADBAND_RADIANS);
    scene
        .box("robot.frame", FRAME)
        .size(body.frameLength().in(Meters), body.frameWidth().in(Meters), height)
        .at(0, 0, height / 2)
        .outline()
        .style(Style.ACCENT);
    scene
        .box("robot.bumpers", FRAME)
        .size(bumperLength, body.bumperWidth().in(Meters), BUMPER_HEIGHT)
        .at(0, 0, BUMPER_CLEARANCE + BUMPER_HEIGHT / 2)
        .style(Style.ACCENT);
    scene
        .arrow("robot.front", FRAME)
        .from(0, 0, height + 0.02)
        .to(bumperLength / 2 + 0.2, 0, height + 0.02)
        .style(Style.ACCENT);
    for (int i = 0; i < moduleFrames.length; i++) {
      String module = WorldView.MODULES.get(i);
      String frame = moduleFrames[i];
      double radius = radii[i];
      scene
          .frame(frame, FRAME, modules[i].getX(), modules[i].getY(), 0, 0, 0, 0)
          .deadband(frame, WHEEL_DEADBAND_METERS, STEER_DEADBAND_RADIANS)
          .value(moduleSpeeds[i], 0.01);
      scene
          .cylinder("wheel." + module, frame)
          .radius(radius)
          .length(WHEEL_WIDTH)
          .at(0, 0, radius, Math.PI / 2, 0, 0)
          .style(Style.ACCENT);
      scene
          .arrow("drive." + module, frame)
          .from(0, 0, 2 * radius + 0.03)
          .to(1, 0, 2 * radius + 0.03)
          .length(moduleSpeeds[i])
          .style(Style.ACCENT);
    }
    List<Translation3d> labels = labelPlaces(cameras, body);
    for (int i = 0; i < cameras.size(); i++) {
      CameraConfig camera = cameras.get(i);
      String frame = WorldView.cameraFrame(camera.id());
      String fullView = CameraViews.view(camera);
      scene.frame(frame, FRAME, mounts[i]);
      scene
          .arrow(frame + ".axis", frame)
          .to(CAMERA_ARROW, 0, 0)
          .style(Style.ACCENT)
          .inspect(() -> WorldView.camera(camera, cameraAction))
          .reveal(fullView);
      double[] fov = fieldOfView(camera.sensor());
      scene
          .frustum(frame + ".stub", frame)
          .fov(fov[0], fov[1])
          .range(0, VIEW_STUB_METERS)
          .inspect(() -> WorldView.camera(camera, cameraAction))
          .reveal(fullView);
      Translation3d label = labels.get(i);
      scene
          .label(frame + ".name", FRAME)
          .text(camera.name())
          .at(label.getX(), label.getY(), label.getZ());
    }
    for (int i = 0; i < sensors.size(); i++) {
      Sensor.Placed sensor = sensors.get(i);
      String frame = "sensor." + i;
      scene.frame(frame, FRAME, sensorMounts[i]);
      scene
          .box(frame + ".block", frame)
          .size(SENSOR_SIZE, SENSOR_SIZE, SENSOR_SIZE)
          .style(Style.ACCENT)
          .inspect(() -> WorldView.sensor(sensor));
      if (sensor instanceof Sensor.Rangefinder rangefinder) {
        double cone = rangefinder.fieldOfView().in(Radians);
        scene
            .frustum(frame + ".cone", frame)
            .fov(cone, cone)
            .range(0, rangefinder.range().in(Meters))
            .inspect(() -> WorldView.sensor(sensor));
      }
    }
    scene.frame(IMU, FRAME, new Transform3d(gyro.position(), gyro.orientation()));
    scene
        .box(IMU + ".block", IMU)
        .size(IMU_SIZE, IMU_SIZE, IMU_THICKNESS)
        .style(Style.ACCENT)
        .inspect(() -> WorldView.imu("Gyro IMU", gyro));
    scene
        .arrow(IMU + ".x", IMU)
        .to(IMU_ARROW, 0, 0)
        .style(Style.ACCENT)
        .inspect(() -> WorldView.imu("Gyro IMU", gyro));
  }

  /**
   * Where each camera's name is drawn, in the robot's frame: where its view's stub ends, seen from
   * above, then on along the way it faces while that's on the robot or on a name placed before, so
   * the names stay off the robot and off each other.
   */
  static List<Translation3d> labelPlaces(List<CameraConfig> cameras, Body body) {
    double halfLength = body.bumperLength().in(Meters) / 2 + LABEL_CLEARANCE;
    double halfWidth = body.bumperWidth().in(Meters) / 2 + LABEL_CLEARANCE;
    List<Translation3d> places = new ArrayList<>();
    for (CameraConfig camera : cameras) {
      double facing = camera.yaw().in(Radians);
      double x = camera.forward().in(Meters);
      double y = camera.left().in(Meters);
      double out = VIEW_STUB_METERS;
      for (int step = 0; step < 100; step++, out += 0.1) {
        double atX = x + out * Math.cos(facing);
        double atY = y + out * Math.sin(facing);
        boolean onRobot = Math.abs(atX) < halfLength && Math.abs(atY) < halfWidth;
        boolean onAnother =
            places.stream()
                .anyMatch(
                    place -> Math.hypot(place.getX() - atX, place.getY() - atY) < LABEL_SPACING);
        if (!onRobot && !onAnother) {
          break;
        }
      }
      places.add(
          new Translation3d(
              x + out * Math.cos(facing),
              y + out * Math.sin(facing),
              camera.up().in(Meters) + LABEL_RISE));
    }
    return places;
  }

  @Override
  public void update(SceneState state) {
    Pose2d pose = readings.estimate().pose();
    // On a bump the robot stands as high as the ground under it, tilted as its IMU says.
    state.pose(
        FRAME,
        pose.getX(),
        pose.getY(),
        field.terrain().heightAt(pose.getTranslation()),
        readings.rollRadians(),
        readings.pitchRadians(),
        pose.getRotation().getRadians());
    SwerveModuleVelocity[] measured = readings.modules();
    for (int i = 0; i < measured.length && i < moduleFrames.length; i++) {
      state.pose(
          moduleFrames[i], modules[i].getX(), modules[i].getY(), measured[i].angle.getRadians());
      state.value(moduleSpeeds[i], measured[i].velocity * WorldView.ARROW_SECONDS);
    }
  }

  /**
   * A camera's field of view, side to side and top to bottom, in radians, from its diagonal and its
   * image's shape.
   */
  static double[] fieldOfView(CameraConfig.Sensor sensor) {
    double width = sensor.widthPixels();
    double height = sensor.heightPixels();
    double diagonal = Math.hypot(width, height);
    double focal = diagonal / 2 / Math.tan(sensor.diagonalFieldOfView().in(Radians) / 2);
    return new double[] {2 * Math.atan(width / 2 / focal), 2 * Math.atan(height / 2 / focal)};
  }
}
