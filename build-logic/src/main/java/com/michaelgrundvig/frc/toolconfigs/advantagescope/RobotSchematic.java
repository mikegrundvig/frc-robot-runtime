package com.michaelgrundvig.frc.toolconfigs.advantagescope;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import com.michaelgrundvig.frc.world.physical.Vector3;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.tools.RobotDescription;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Draws a robot as a schematic, from its configuration: its frame, bumpers, cameras, IMU and
 * wheels, each a simple solid in a flat color. {@link AdvantageScopeAssets} says what is drawn and
 * what is left out, and writes these meshes to files.
 *
 * <p>Everything is in the robot's frame: x forward, y left, z up, in meters, from the center of the
 * frame on the floor.
 */
final class RobotSchematic {
  // The colors, distinct for each kind of part. AdvantageScope draws each see-through one at its
  // own fixed opacity.
  static final Mesh.Material FRAME = new Mesh.Material("frame", 0.55, 0.62, 0.72, 0.25);
  static final Mesh.Material BUMPERS = new Mesh.Material("bumpers", 0.16, 0.16, 0.19, 1);
  static final Mesh.Material FRONT = new Mesh.Material("front", 1.0, 0.45, 0.0, 1);
  static final Mesh.Material CAMERA = new Mesh.Material("camera", 0.0, 0.7, 0.9, 1);
  static final Mesh.Material CONE = new Mesh.Material("field of view", 0.0, 0.7, 0.9, 0.25);
  static final Mesh.Material IMU = new Mesh.Material("imu", 0.85, 0.2, 0.75, 1);
  static final Mesh.Material WHEEL = new Mesh.Material("wheel", 0.12, 0.12, 0.12, 1);

  /** A wheel is drawn this wide for its radius: a 4 in wheel 1.5 in wide, as typical. */
  private static final double WHEEL_WIDTH_PER_RADIUS = 0.75;

  /** Flat faces around a wheel's tread. */
  private static final int WHEEL_SIDES = 24;

  /** How long a camera's arrow is, from its lens. */
  private static final double CAMERA_ARROW_METERS = 0.2;

  /** How thick the flat arrows are. */
  private static final double ARROW_THICKNESS_METERS = 0.008;

  /** The IMU is drawn as a block this long and wide, and 2 cm thick. */
  private static final double IMU_SIZE_METERS = 0.05;

  private static final double IMU_HALF_THICKNESS_METERS = 0.01;

  /** How far above a wheel's top its arrow floats, so the two do not flicker where they touch. */
  private static final double WHEEL_ARROW_GAP_METERS = 0.004;

  private RobotSchematic() {}

  /**
   * One wheel's model: the wheel, and an arrow above it the way it rolls forward.
   *
   * @param wheel the wheel itself, a disc
   * @param arrow the arrow above it
   */
  record WheelModel(Mesh wheel, Mesh arrow) {
    List<Mesh> meshes() {
      return List.of(wheel, arrow);
    }
  }

  /** The base model: the frame, bumpers, the front arrow, the IMU, and the cameras. */
  static List<Mesh> body(RobotDescription robot) {
    Body body = robot.body();
    List<Mesh> meshes = new ArrayList<>();
    meshes.add(frame(body));
    meshes.add(bumpers(body));
    meshes.add(frontArrow(body));
    Optional<Translation3d> imu = robot.imu();
    if (imu.isPresent()) {
      meshes.add(imu(imu.get()));
    }
    for (CameraConfig camera : camerasOnChassis(robot)) {
      meshes.add(camera(camera));
    }
    return meshes;
  }

  /**
   * The frame: a see-through box as long and wide as the frame, from the bumpers' bottom up to the
   * robot's overall height. A robot no taller than its bumpers' bottom has nothing to draw, and
   * gets an empty mesh.
   */
  private static Mesh frame(Body body) {
    double length = body.frameLength().in(Meters);
    double width = body.frameWidth().in(Meters);
    double height = body.overallHeight().in(Meters);
    double bottom = body.bumperBottomOrDefault().in(Meters);
    Mesh frame = new Mesh("frame", FRAME);
    if (height > bottom) {
      frame.box(
          Frame.ROBOT,
          new Vector3(-length / 2, -width / 2, bottom),
          new Vector3(length / 2, width / 2, height));
    }
    return frame;
  }

  /**
   * The bumpers: a ring around the frame, as four boxes. The ones across the front and the back run
   * the bumpers' whole width, corners included, and the ones along the left and the right fit
   * between them.
   */
  private static Mesh bumpers(Body body) {
    double frontOfFrame = body.frameLength().in(Meters) / 2;
    double leftOfFrame = body.frameWidth().in(Meters) / 2;
    double frontOfBumpers = frontOfFrame + body.bumperThicknessOrDefault().in(Meters);
    double leftOfBumpers = leftOfFrame + body.bumperThicknessLeftRightOrDefault().in(Meters);
    double bottom = body.bumperBottomOrDefault().in(Meters);
    double top = body.bumperTopOrDefault().in(Meters);
    return new Mesh("bumpers", BUMPERS)
        .box(
            Frame.ROBOT,
            new Vector3(frontOfFrame, -leftOfBumpers, bottom),
            new Vector3(frontOfBumpers, leftOfBumpers, top))
        .box(
            Frame.ROBOT,
            new Vector3(-frontOfBumpers, -leftOfBumpers, bottom),
            new Vector3(-frontOfFrame, leftOfBumpers, top))
        .box(
            Frame.ROBOT,
            new Vector3(-frontOfFrame, leftOfFrame, bottom),
            new Vector3(frontOfFrame, leftOfBumpers, top))
        .box(
            Frame.ROBOT,
            new Vector3(-frontOfFrame, -leftOfBumpers, bottom),
            new Vector3(frontOfFrame, -leftOfFrame, top));
  }

  /** An arrow lying on the frame's top, pointing forward, sized to the frame. */
  private static Mesh frontArrow(Body body) {
    double length = body.frameLength().in(Meters);
    double width = body.frameWidth().in(Meters);
    double height = body.overallHeight().in(Meters);
    Mesh.Arrow arrow =
        new Mesh.Arrow(
            -0.3 * length,
            0.35 * length,
            0.2 * length,
            0.1 * width,
            0.35 * width,
            height,
            height + ARROW_THICKNESS_METERS);
    return new Mesh("front", FRONT).arrow(Frame.ROBOT, arrow);
  }

  /** The gyro's IMU: a small flat block centered where its mount puts it. */
  private static Mesh imu(Translation3d mount) {
    Vector3 halfSize =
        new Vector3(IMU_SIZE_METERS / 2, IMU_SIZE_METERS / 2, IMU_HALF_THICKNESS_METERS);
    Frame atMount = Frame.at(new Vector3(mount.getX(), mount.getY(), mount.getZ()));
    return new Mesh("imu", IMU).box(atMount, halfSize.times(-1), halfSize);
  }

  /**
   * A camera: a small box behind its lens, and an arrow along its optical axis, which is a thin
   * square shaft with a pyramid for a head. All of it is given in the lens's own frame, where x is
   * the optical axis.
   */
  private static Mesh camera(CameraConfig camera) {
    Frame lens = lens(camera);
    double shaftLength = CAMERA_ARROW_METERS * 0.7;
    double shaftHalfWidth = 0.006;
    double headHalfWidth = 0.02;
    return new Mesh("camera " + camera.name(), CAMERA)
        .box(lens, new Vector3(-0.04, -0.03, -0.02), new Vector3(0, 0.03, 0.02))
        .box(
            lens,
            new Vector3(0, -shaftHalfWidth, -shaftHalfWidth),
            new Vector3(shaftLength, shaftHalfWidth, shaftHalfWidth))
        .pyramid(
            lens,
            new Vector3(CAMERA_ARROW_METERS, 0, 0),
            List.of(
                new Vector3(shaftLength, headHalfWidth, headHalfWidth),
                new Vector3(shaftLength, -headHalfWidth, headHalfWidth),
                new Vector3(shaftLength, -headHalfWidth, -headHalfWidth),
                new Vector3(shaftLength, headHalfWidth, -headHalfWidth)));
  }

  /**
   * Each wheel: a disc where it sits, a wheel radius up, and an arrow above it the way it rolls
   * forward. A model of its own each, for AdvantageScope to turn.
   */
  static List<WheelModel> wheels(RobotDescription robot) {
    Translation2d[] places = robot.drivetrain().moduleTranslations();
    List<WheelModel> wheels = new ArrayList<>();
    for (int module = 0; module < places.length; module++) {
      double radius = robot.drivetrain().module(module).wheelRadius().in(Meters);
      Frame atCenter = Frame.at(new Vector3(places[module].getX(), places[module].getY(), radius));
      // A drivetrain has exactly four modules, in the order the wheels are named in.
      String name = AdvantageScopeAssets.WHEELS.get(module);
      double halfWidth = radius * WHEEL_WIDTH_PER_RADIUS / 2;
      Mesh wheel = new Mesh("wheel " + name, WHEEL).wheel(atCenter, radius, halfWidth, WHEEL_SIDES);
      // The arrow is given from the wheel's center, so its underside is a radius up: just clear
      // of the wheel's top.
      double arrowUnderside = radius + WHEEL_ARROW_GAP_METERS;
      Mesh.Arrow shape =
          new Mesh.Arrow(
              -0.7 * radius,
              1.3 * radius,
              0.6 * radius,
              0.3 * radius,
              0.9 * radius,
              arrowUnderside,
              arrowUnderside + ARROW_THICKNESS_METERS);
      Mesh arrow = new Mesh("wheel arrow " + name, FRONT).arrow(atCenter, shape);
      wheels.add(new WheelModel(wheel, arrow));
    }
    return wheels;
  }

  /**
   * The cameras fixed to the chassis: a camera that rides a mechanism moves with its joint, which a
   * model drawn once can't follow, so it's left out.
   */
  static List<CameraConfig> camerasOnChassis(RobotDescription robot) {
    return robot.cameras().stream().filter(camera -> camera.joint().isEmpty()).toList();
  }

  /**
   * Each camera's field of view: a pyramid from its lens out to its sight range along its optical
   * axis, as wide and tall as the image sees there.
   */
  static List<Mesh> cones(RobotDescription robot) {
    List<Mesh> cones = new ArrayList<>();
    for (CameraConfig camera : camerasOnChassis(robot)) {
      CameraConfig.ImageSensor sensor = camera.sensor();
      // At its sight range, the image is this far across and up from the optical axis: the range
      // times the tangent of half the angle it sees that way.
      double range = camera.sightRangeOrDefault().in(Meters);
      double halfAcross = range * StrictMath.tan(halfAngle(camera, sensor.widthPixels()));
      double halfUp = range * StrictMath.tan(halfAngle(camera, sensor.heightPixels()));
      cones.add(
          new Mesh("field of view " + camera.name(), CONE)
              .pyramid(
                  lens(camera),
                  Vector3.ZERO,
                  List.of(
                      new Vector3(range, halfAcross, halfUp),
                      new Vector3(range, -halfAcross, halfUp),
                      new Vector3(range, -halfAcross, -halfUp),
                      new Vector3(range, halfAcross, -halfUp))));
    }
    return cones;
  }

  /**
   * Half the angle the camera sees across one side of its image, for a lens that sees its diagonal
   * field of view across the image's diagonal, with square pixels: the way PhotonVision's
   * simulation reads it.
   *
   * <p>The tangent of a half angle is half the image's size that way over the lens's focal length.
   * So the tangent across a side is the tangent across the diagonal, times the side's length over
   * the diagonal's.
   *
   * @param pixels the side's length in pixels: the width, or the height
   */
  static double halfAngle(CameraConfig camera, int pixels) {
    CameraConfig.ImageSensor sensor = camera.sensor();
    double diagonalPixels = StrictMath.hypot(sensor.widthPixels(), sensor.heightPixels());
    double tangentAcrossDiagonal = StrictMath.tan(camera.diagonalFieldOfView().in(Radians) / 2);
    return StrictMath.atan(tangentAcrossDiagonal * pixels / diagonalPixels);
  }

  /** A camera's frame: at its lens, x along its optical axis, as its configuration mounts it. */
  private static Frame lens(CameraConfig camera) {
    Vector3 mount =
        new Vector3(
            camera.forward().orElseThrow().in(Meters),
            camera.left().orElseThrow().in(Meters),
            camera.up().orElseThrow().in(Meters));
    // WPILib's pitch is positive nose down, so a camera tilted up has a negative one.
    double pitchRadians = -camera.tiltUp().orElseThrow().in(Radians);
    return Frame.turned(
        mount,
        camera.rollOrDefault().in(Radians),
        pitchRadians,
        camera.yaw().orElseThrow().in(Radians));
  }
}
