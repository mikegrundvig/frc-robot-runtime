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
import com.michaelgrundvig.frc.world.robot.Joint;
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
 * The robot where localization believes it is, as its configuration describes it. It draws:
 *
 * <ul>
 *   <li>the frame and the bumpers, with an arrow out the front;
 *   <li>each wheel at its module, turned to its measured angle, with an arrow where it's driving;
 *   <li>each camera at its exact mount, with an arrow along its optical axis, the first meter of
 *       its view, and its name off the robot;
 *   <li>each switch and rangefinder where it's mounted, as a small block, and a rangefinder's cone
 *       out to its range;
 *   <li>the gyro's IMU, as a small block where its mount puts it, turned as it's mounted, with an
 *       arrow along its own x axis.
 * </ul>
 *
 * <p>A click on a camera says how it's mounted, offers its calibration, and shows its full view
 * ({@link CameraViews}) until the details close. A click on the IMU says how it's mounted and
 * whether its accelerometer has been checked. A click on a sensor says what it is and where.
 */
final class RobotLayer implements Layer {
  /** The robot's frame on the page: the frame the page's Follow follows. */
  static final String FRAME = "robot";

  /** About how tall a bumper is, in meters. The rules allow 5 in. */
  private static final double BUMPER_HEIGHT = 0.127;

  /** How far above the floor bumpers ride. */
  private static final double BUMPER_CLEARANCE = 0.02;

  /** How far above the frame's top the arrow out the front is drawn. */
  private static final double FRONT_ARROW_RISE = 0.02;

  /** How far past the front bumper the arrow out the front reaches. */
  private static final double FRONT_ARROW_REACH = 0.2;

  /** How far above its wheel's top a wheel's drive arrow is drawn. */
  private static final double DRIVE_ARROW_RISE = 0.03;

  /** How far a wheel's speed must change, in meters of its arrow, before the page hears of it. */
  private static final double DRIVE_ARROW_DEADBAND = 0.01;

  /** The gyro's IMU on the page. This is its frame, turned as it's mounted. */
  private static final String IMU = "imu.gyro";

  /** The IMU's block, which is about a Pigeon 2's size, and the size AdvantageScope draws. */
  private static final double IMU_SIZE = 0.05;

  private static final double IMU_THICKNESS = 0.02;

  /** The arrow along the IMU's own x axis, which shows how it's turned. */
  private static final double IMU_ARROW = 0.1;

  /** A switch's or a rangefinder's block, which is about a CANrange's or a limit switch's size. */
  private static final double SENSOR_SIZE = 0.03;

  /** About how wide a swerve wheel is. */
  private static final double WHEEL_WIDTH = 0.038;

  /** The arrow along a camera's optical axis. */
  private static final double CAMERA_ARROW = 0.3;

  /**
   * How far a camera's view is always drawn. It reaches past the bumpers, to show where the camera
   * points and how wide it sees, and stops far short of the 7 m that covered the field four times
   * over.
   */
  static final double VIEW_STUB_METERS = 1.0;

  /** How far a camera's name stays clear of the bumpers. */
  private static final double LABEL_CLEARANCE = 0.15;

  /** How far apart two cameras' names stay, center to center. */
  private static final double LABEL_SPACING = 0.35;

  /** How far above its camera a camera's name is drawn. */
  private static final double LABEL_RISE = 0.1;

  /** A camera's name is moved out along the way the camera faces in steps this long. */
  private static final double LABEL_STEP = 0.1;

  /** The most steps a name is moved out, which is 10 m and past any robot. */
  private static final int LABEL_STEPS = 100;

  /**
   * How far the estimate must move before the page hears of it. A still robot's estimate wanders a
   * few millimeters as each image corrects it, which is no news, and a centimeter is less than a
   * pixel of a field on a laptop.
   */
  static final double DEADBAND_METERS = 0.01;

  static final double DEADBAND_RADIANS = Math.toRadians(0.5);

  /**
   * How far a wheel must turn before the page hears of it. A wheel at rest reads a few tenths of a
   * degree either way as its encoder settles, and a degree is less than a pixel of its drawing.
   */
  static final double STEER_DEADBAND_RADIANS = Math.toRadians(1);

  /** A wheel's frame only turns, so the deadband for its place is the least there is: 1 mm. */
  private static final double WHEEL_DEADBAND_METERS = 0.001;

  /** No joint's position given, which the robot's maths takes as every joint at its reference. */
  private static final Map<Joint, Double> JOINTS_AT_REST = Map.of();

  private final FieldModel field;
  private final Body body;

  /** Each wheel's radius, in meters. */
  private final double[] radii;

  private final List<CameraConfig> cameras;

  /**
   * Each camera's mount on the robot, in the cameras' order. A camera on a joint is drawn where its
   * joint rests, because the view doesn't follow the joints yet.
   */
  private final Transform3d[] mounts;

  /** The switches and rangefinders. */
  private final List<Sensor.Placed> sensors;

  /** Each one's mount on the robot, in their order. One on a joint is where its joint rests. */
  private final Transform3d[] sensorMounts;

  private final ImuMount gyro;
  private final WorldView.Readings readings;
  private final Function<String, Optional<String>> cameraAction;
  private final Translation2d[] modules;

  /** Each module's frame and speed on the page. They are named once, since every loop uses them. */
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
    RobotMath math = robot.math();
    mounts = new Transform3d[cameras.size()];
    for (int i = 0; i < cameras.size(); i++) {
      mounts[i] = asMount(math.cameraPose(cameras.get(i), JOINTS_AT_REST));
    }
    sensors =
        math.sensors().stream()
            .filter(Sensor.Placed.class::isInstance)
            .map(Sensor.Placed.class::cast)
            .toList();
    sensorMounts = new Transform3d[sensors.size()];
    for (int i = 0; i < sensors.size(); i++) {
      sensorMounts[i] = asMount(math.sensorPose(sensors.get(i), JOINTS_AT_REST));
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

  /** A pose in the robot's frame, as the mount that puts a part there. */
  private static Transform3d asMount(Pose3d inTheRobotsFrame) {
    return new Transform3d(inTheRobotsFrame.getTranslation(), inTheRobotsFrame.getRotation());
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
    scene.frame(FRAME, SceneBuilder.ROOT).deadband(FRAME, DEADBAND_METERS, DEADBAND_RADIANS);
    describeBody(scene);
    describeWheels(scene);
    describeCameras(scene);
    describeSensors(scene);
    describeImu(scene);
  }

  /** The frame as an outline, the bumpers as a solid, and an arrow out the front. */
  private void describeBody(SceneBuilder scene) {
    double height = body.height().in(Meters);
    double bumperLength = body.bumperLength().in(Meters);
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
        .from(0, 0, height + FRONT_ARROW_RISE)
        .to(bumperLength / 2 + FRONT_ARROW_REACH, 0, height + FRONT_ARROW_RISE)
        .style(Style.ACCENT);
  }

  /**
   * Each wheel in a frame of its own at its module, which {@link #update} turns to the wheel's
   * measured angle. The wheel is a cylinder lying on its side, and its drive arrow points along the
   * frame's x, as long as the module's speed value says.
   */
  private void describeWheels(SceneBuilder scene) {
    for (int i = 0; i < moduleFrames.length; i++) {
      String module = WorldView.MODULES.get(i);
      String frame = moduleFrames[i];
      double radius = radii[i];
      double aboveTheWheel = 2 * radius + DRIVE_ARROW_RISE;
      scene
          .frame(frame, FRAME, modules[i].getX(), modules[i].getY(), 0, 0, 0, 0)
          .deadband(frame, WHEEL_DEADBAND_METERS, STEER_DEADBAND_RADIANS)
          .value(moduleSpeeds[i], DRIVE_ARROW_DEADBAND);
      // A cylinder's axis is its z. Rolled a quarter turn, the axis lies along the wheel's axle.
      scene
          .cylinder("wheel." + module, frame)
          .radius(radius)
          .length(WHEEL_WIDTH)
          .at(0, 0, radius, Math.PI / 2, 0, 0)
          .style(Style.ACCENT);
      scene
          .arrow("drive." + module, frame)
          .from(0, 0, aboveTheWheel)
          .to(1, 0, aboveTheWheel)
          .length(moduleSpeeds[i])
          .style(Style.ACCENT);
    }
  }

  /**
   * Each camera in a frame of its own at its mount: an arrow along its optical axis, the first
   * meter of its view, and its name. A click on the arrow or the view says how the camera is
   * mounted, and shows its full view.
   */
  private void describeCameras(SceneBuilder scene) {
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
      FieldOfView sees = fieldOfView(camera.sensor());
      scene
          .frustum(frame + ".stub", frame)
          .fov(sees.acrossRadians(), sees.upAndDownRadians())
          .range(0, VIEW_STUB_METERS)
          .inspect(() -> WorldView.camera(camera, cameraAction))
          .reveal(fullView);
      Translation3d label = labels.get(i);
      scene
          .label(frame + ".name", FRAME)
          .text(camera.name())
          .at(label.getX(), label.getY(), label.getZ());
    }
  }

  /** Each switch and rangefinder as a small block at its mount, and a rangefinder's cone. */
  private void describeSensors(SceneBuilder scene) {
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
  }

  /** The gyro's IMU as a block in a frame turned as it's mounted, with an arrow along its x. */
  private void describeImu(SceneBuilder scene) {
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
   * Where each camera's name is drawn, in the robot's frame.
   *
   * <p>A name starts where its camera's view stub ends, seen from above. While that spot is over
   * the robot, or too near a name placed before, the name moves on along the way the camera faces,
   * a step at a time. So the names stay off the robot and off each other.
   */
  static List<Translation3d> labelPlaces(List<CameraConfig> cameras, Body body) {
    double halfLength = body.bumperLength().in(Meters) / 2 + LABEL_CLEARANCE;
    double halfWidth = body.bumperWidth().in(Meters) / 2 + LABEL_CLEARANCE;
    List<Translation3d> places = new ArrayList<>();
    for (CameraConfig camera : cameras) {
      double facing = camera.yaw().in(Radians);
      double x = camera.forward().in(Meters);
      double y = camera.left().in(Meters);
      // How far out from the camera the name is, along the way the camera faces.
      double out = VIEW_STUB_METERS;
      for (int step = 0; step < LABEL_STEPS; step++) {
        double atX = x + out * Math.cos(facing);
        double atY = y + out * Math.sin(facing);
        boolean overTheRobot = Math.abs(atX) < halfLength && Math.abs(atY) < halfWidth;
        if (!overTheRobot && !isNearAnother(places, atX, atY)) {
          break;
        }
        out += LABEL_STEP;
      }
      places.add(
          new Translation3d(
              x + out * Math.cos(facing),
              y + out * Math.sin(facing),
              camera.up().in(Meters) + LABEL_RISE));
    }
    return places;
  }

  /** Whether a spot, seen from above, is too near a name already placed. */
  private static boolean isNearAnother(List<Translation3d> places, double x, double y) {
    for (Translation3d place : places) {
      if (Math.hypot(place.getX() - x, place.getY() - y) < LABEL_SPACING) {
        return true;
      }
    }
    return false;
  }

  @Override
  public void update(SceneState state) {
    Pose2d pose = readings.report().pose();
    // On a bump the robot stands as high as the ground under it, and it is tilted as its IMU says.
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
      // The arrow reaches to where the wheel will be in a moment: length = speed × time.
      state.value(moduleSpeeds[i], measured[i].velocity * WorldView.ARROW_SECONDS);
    }
  }

  /**
   * How wide a camera sees.
   *
   * @param acrossRadians its field of view from side to side
   * @param upAndDownRadians its field of view from top to bottom
   */
  record FieldOfView(double acrossRadians, double upAndDownRadians) {}

  /**
   * A camera's field of view from side to side and from top to bottom, worked out from its field of
   * view across the diagonal and its image's shape, for a camera without distortion.
   *
   * <p>The focal length, in pixels, is the distance from the lens at which half the image's
   * diagonal spans half the diagonal's angle:
   *
   * <pre>focal = (diagonal / 2) / tan(diagonal angle / 2)</pre>
   *
   * <p>At that distance, half the image's width spans half the angle across, and likewise its
   * height:
   *
   * <pre>angle across = 2 × atan((width / 2) / focal)</pre>
   */
  static FieldOfView fieldOfView(CameraConfig.Sensor sensor) {
    double widthPixels = sensor.widthPixels();
    double heightPixels = sensor.heightPixels();
    double diagonalPixels = Math.hypot(widthPixels, heightPixels);
    double focalPixels =
        diagonalPixels / 2 / Math.tan(sensor.diagonalFieldOfView().in(Radians) / 2);
    return new FieldOfView(
        2 * Math.atan(widthPixels / 2 / focalPixels),
        2 * Math.atan(heightPixels / 2 / focalPixels));
  }
}
