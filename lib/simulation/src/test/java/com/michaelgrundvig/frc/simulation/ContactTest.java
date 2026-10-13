package com.michaelgrundvig.frc.simulation;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.field.Block;
import com.michaelgrundvig.frc.world.field.Bump;
import com.michaelgrundvig.frc.world.field.Element;
import com.michaelgrundvig.frc.world.field.Terrain;
import com.michaelgrundvig.frc.world.physical.Material;
import com.michaelgrundvig.frc.world.physical.Polygon2d;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The robot driven at the walls and at what stands on a field, in a small arena, 8 m by 6 m:
 *
 * <ul>
 *   <li>a post 1 m square in its middle, from (4, 2.5) to (5, 3.5), taller than the bumpers;
 *   <li>a bar across it at x = 2, a meter up, which the robot drives under;
 *   <li>a bump across it with its crest at x = 6.5, which the robot drives over.
 * </ul>
 *
 * <p>Every approach is held to the same things at every physics step: the bumpers are never inside
 * a wall or the post, the robot never moves farther than its wheels rolled, and two runs of it are
 * the same to the bit.
 */
class ContactTest {
  private static final double STEP = 0.004;
  private static final double LENGTH = 8;
  private static final double WIDTH = 6;

  /** From the robot's center to the outside of its bumpers: it is square. */
  private static final double HALF = TestRobot.BODY.bumperLength().in(Meters) / 2;

  /** From its center to a corner of its bumpers. */
  private static final double TO_CORNER = HALF * Math.sqrt(2);

  private static final Material STEEL = Material.builder().name("steel").build();
  private static final Polygon2d WALLS =
      Polygon2d.rectangle(Translation2d.ZERO, new Translation2d(LENGTH, WIDTH));
  private static final Polygon2d POST =
      Polygon2d.rectangle(new Translation2d(4, 2.5), new Translation2d(5, 3.5));
  private static final Polygon2d BAR =
      Polygon2d.rectangle(new Translation2d(2, 0), new Translation2d(2.2, WIDTH));
  private static final List<Element> ELEMENTS =
      List.of(
          Element.standing("Post", POST, 1, STEEL),
          new Element("Bar", List.of(new Block(BAR, 1, 1.2, STEEL)), List.of()));
  private static final Terrain BUMP =
      new Terrain(
          List.of(
              new Bump("Bump", new Translation2d(6.5, 0), new Translation2d(6.5, WIDTH), 0.1, 1)));

  /** How the wheels are driven for an approach. */
  enum Wheels {
    /** Every wheel pointed one way in the robot's frame, and driven. */
    TRANSLATING,
    /** Each wheel pointed around the robot's center, and driven: it spins where it stands. */
    SPINNING
  }

  /** What the robot drives among. */
  enum World {
    /** The arena: its walls, the post and the bar. */
    ARENA,
    /** Nothing to run into anywhere, as the physics is until it is given a field. */
    OPEN_SPACE
  }

  /** How an approach must end. */
  enum Ends {
    /** Standing still against something, its wheels still turning. */
    STOPPED,
    /** Still moving: sliding along something, turning, or driving free. */
    MOVING
  }

  /**
   * One approach: where the robot starts, how it is driven and for how long, and where that must
   * leave it.
   *
   * @param wayDegrees which way the wheels point, in the robot's frame, when translating
   * @param endX where its center must end along x, or NaN where the row doesn't say
   * @param endY where its center must end along y, or NaN where the row doesn't say
   * @param ends whether it must end standing still
   */
  record Approach(
      Pose2d start,
      Wheels wheels,
      double wayDegrees,
      double seconds,
      double endX,
      double endY,
      Ends ends) {}

  /** What a run of an approach did. */
  record Outcome(
      Pose2d end,
      Translation2d endVelocity,
      double turnedRadians,
      double traveledMeters,
      double wheelsRolledMeters,
      double deepestInsideMeters,
      double highestMeters,
      long fingerprint) {}

  private static Pose2d pose(double x, double y, double degrees) {
    return new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
  }

  private static Arguments row(String label, Approach approach) {
    return Arguments.argumentSet(label, approach);
  }

  static Stream<Arguments> approaches() {
    double none = Double.NaN;
    double atFarWall = LENGTH - HALF;
    double atLeftWall = WIDTH - HALF;
    return Stream.of(
        row(
            "head-on into a wall at top speed, under the bar and over the bump",
            new Approach(pose(1, 5, 0), Wheels.TRANSLATING, 0, 3, atFarWall, 5, Ends.STOPPED)),
        row(
            "head-on into the post",
            new Approach(pose(1, 3, 0), Wheels.TRANSLATING, 0, 3, 4 - HALF, 3, Ends.STOPPED)),
        row(
            "glancing off a wall, then sliding along it",
            new Approach(
                pose(1, 1, 0), Wheels.TRANSLATING, 60, 2.4, none, atLeftWall, Ends.MOVING)),
        row(
            "into an inside corner",
            new Approach(
                pose(1, 1, 0), Wheels.TRANSLATING, 60, 4, atFarWall, atLeftWall, Ends.STOPPED)),
        row(
            "catching the post's face with one end of the bumpers, square on",
            new Approach(pose(1, 2.2, 0), Wheels.TRANSLATING, 0, 3, 4 - HALF, 2.2, Ends.STOPPED)),
        row(
            "glancing off the post's face, sliding off its end and on into the far corner",
            new Approach(
                pose(1, 1.6, 0), Wheels.TRANSLATING, 15, 6, atFarWall, atLeftWall, Ends.STOPPED)),
        row(
            "along a wall it is touching",
            new Approach(pose(3, HALF, 0), Wheels.TRANSLATING, 0, 0.8, none, HALF, Ends.MOVING)),
        row(
            "backing off a wall it starts against",
            new Approach(pose(HALF, 1, 0), Wheels.TRANSLATING, 0, 0.8, none, 1, Ends.MOVING)),
        row(
            "a corner swinging into a wall as it spins",
            new Approach(pose(HALF + 0.02, 3, 0), Wheels.SPINNING, 0, 2, none, 3, Ends.MOVING)),
        row(
            "turned a little, square into a wall",
            new Approach(
                pose(5.5, 1, 20), Wheels.TRANSLATING, -20, 2.5, none, none, Ends.STOPPED)));
  }

  @ParameterizedTest
  @MethodSource("approaches")
  void theRobotStopsAtWhatItRunsIntoOrSlidesAlongIt(Approach approach) {
    Outcome outcome = run(approach, World.ARENA);

    assertThat(outcome.deepestInsideMeters())
        .as("how far inside anything, at worst")
        .isLessThan(1e-6);
    if (!Double.isNaN(approach.endX())) {
      assertThat(outcome.end().getX()).as("x at the end").isCloseTo(approach.endX(), within(1e-6));
    }
    if (!Double.isNaN(approach.endY())) {
      assertThat(outcome.end().getY()).as("y at the end").isCloseTo(approach.endY(), within(1e-6));
    }
    if (approach.ends() == Ends.STOPPED) {
      assertThat(outcome.endVelocity().getNorm()).as("speed at the end").isLessThan(1e-6);
      assertThat(outcome.wheelsRolledMeters())
          .as("the wheels kept turning against it")
          .isGreaterThan(outcome.traveledMeters() + 0.5);
    }
    assertThat(outcome.traveledMeters())
        .as("it never goes farther than its wheels rolled it")
        .isLessThanOrEqualTo(outcome.wheelsRolledMeters() + 1e-6);
    assertThat(run(approach, World.ARENA).fingerprint())
        .as("the same run again")
        .isEqualTo(outcome.fingerprint());
  }

  /**
   * What only some approaches show: a slide keeps its speed along the wall, a spin keeps turning.
   */
  @Test
  void aSlideKeepsMovingAndASpinKeepsTurning() {
    Outcome sliding =
        run(
            new Approach(pose(1, 1, 0), Wheels.TRANSLATING, 60, 2.4, 0, 0, Ends.MOVING),
            World.ARENA);
    Outcome spinning =
        run(
            new Approach(pose(HALF + 0.02, 3, 0), Wheels.SPINNING, 0, 2, 0, 0, Ends.MOVING),
            World.ARENA);
    Outcome overTheBump =
        run(new Approach(pose(1, 5, 0), Wheels.TRANSLATING, 0, 3, 0, 0, Ends.STOPPED), World.ARENA);

    assertThat(sliding.endVelocity().getX()).as("along the wall, m/s").isGreaterThan(1);
    assertThat(sliding.endVelocity().getY()).as("into the wall, m/s").isCloseTo(0, within(1e-6));
    assertThat(spinning.turnedRadians()).as("it turned on, pushed off the wall").isGreaterThan(6);
    assertThat(spinning.end().getX())
        .as("clear of the wall")
        .isBetween(HALF - 1e-6, TO_CORNER + 0.02);
    assertThat(overTheBump.highestMeters())
        .as("it rode over the bump on the way")
        .isGreaterThan(0.03);
  }

  /** Where nothing is touched, the walls and the post change nothing: to the bit. */
  @Test
  void aRobotThatTouchesNothingDrivesAsInOpenSpace() {
    Approach alongTheWall =
        new Approach(pose(3, HALF, 0), Wheels.TRANSLATING, 0, 0.8, 0, 0, Ends.MOVING);
    Approach offTheWall =
        new Approach(pose(HALF, 1, 0), Wheels.TRANSLATING, 0, 0.8, 0, 0, Ends.MOVING);

    for (Approach approach : List.of(alongTheWall, offTheWall)) {
      Outcome among = run(approach, World.ARENA);
      Outcome open = run(approach, World.OPEN_SPACE);

      assertThat(among.end()).isEqualTo(open.end());
      assertThat(among.traveledMeters()).isGreaterThan(0.5);
    }
  }

  /**
   * Set down or shoved into something, the robot ends beside it: inside the post it is moved out
   * the nearest way, and shoved through a wall it stops against the wall.
   */
  @Test
  void aRobotPlacedOrShovedIntoSomethingEndsClearOfIt() {
    KinematicPhysics physics = TestRobot.physics();
    physics.apply(new Disturbance.Standing(WALLS, ELEMENTS));

    physics.apply(new Disturbance.Placed(pose(4.2, 3, 0)));
    assertThat(physics.chassis().pose().getX())
        .as("out of the post's near side")
        .isCloseTo(4 - HALF, within(1e-9));
    assertThat(physics.chassis().pose().getY()).isEqualTo(3);

    physics.apply(
        new Disturbance.Placed(pose(1, 1, 0).plus(new Transform2d(-3, 0, Rotation2d.ZERO))));
    assertThat(physics.chassis().pose().getX())
        .as("against the wall it was shoved at")
        .isCloseTo(HALF, within(1e-9));

    physics.apply(new Disturbance.Placed(pose(20, -5, 45)));
    assertThat(deepestInside(physics.chassis().pose()))
        .as("set down off the field")
        .isLessThan(1e-6);
  }

  /** Runs an approach from rest, in the arena or in open space. */
  private static Outcome run(Approach approach, World world) {
    KinematicPhysics physics = TestRobot.physics();
    TestRobot.Controllers controllers = new TestRobot.Controllers();
    physics.apply(new Disturbance.Ground(BUMP));
    if (world == World.ARENA) {
      physics.apply(new Disturbance.Standing(WALLS, ELEMENTS));
    }
    physics.apply(new Disturbance.Placed(approach.start()));
    controllers.volts = 12;
    Translation2d[] wheels = TestRobot.wheels();
    for (int wheel = 0; wheel < wheels.length; wheel++) {
      controllers.steer[wheel] =
          switch (approach.wheels()) {
            case TRANSLATING -> Math.toRadians(approach.wayDegrees());
            // Around the center, counterclockwise: a quarter turn on from the way out to the wheel.
            case SPINNING -> Math.atan2(wheels[wheel].getY(), wheels[wheel].getX()) + Math.PI / 2;
          };
    }

    double traveled = 0;
    double deepest = 0;
    double highest = 0;
    long fingerprint = 17;
    Pose2d before = physics.chassis().pose();
    for (int step = 0; step < Math.round(approach.seconds() / STEP); step++) {
      physics.step(STEP, controllers);
      Pose2d now = physics.chassis().pose();
      traveled += now.getTranslation().getDistance(before.getTranslation());
      if (world == World.ARENA) {
        deepest = Math.max(deepest, deepestInside(now));
      }
      highest = Math.max(highest, physics.chassis().pose3d().getZ());
      fingerprint = 31 * fingerprint + Double.doubleToRawLongBits(now.getX());
      fingerprint = 31 * fingerprint + Double.doubleToRawLongBits(now.getY());
      fingerprint = 31 * fingerprint + Double.doubleToRawLongBits(now.getRotation().getRadians());
      before = now;
    }
    double rolled = Math.abs(physics.wheel(0).rotationRadians()) * TestRobot.RADIUS;
    return new Outcome(
        before,
        physics.chassis().velocity(),
        physics.chassis().yawTurnedRadians(),
        traveled,
        rolled,
        deepest,
        highest,
        fingerprint);
  }

  /**
   * How far the bumpers reach inside the post or past a wall at a pose, at worst, in meters. Worked
   * out from the model's own polygons, apart from the code under test: points every few centimeters
   * round the bumpers against the post and the walls, and the post's corners against the bumpers.
   */
  private static double deepestInside(Pose2d pose) {
    Polygon2d bumpers = Polygon2d.rectangle(pose, 2 * HALF, 2 * HALF);
    double deepest = 0;
    for (Translation2d point : around(bumpers)) {
      deepest = Math.max(deepest, depthInside(POST, point));
      double pastAWall =
          Math.max(
              Math.max(-point.getX(), point.getX() - LENGTH),
              Math.max(-point.getY(), point.getY() - WIDTH));
      deepest = Math.max(deepest, pastAWall);
    }
    for (Translation2d corner : POST.corners()) {
      deepest = Math.max(deepest, depthInside(bumpers, corner));
    }
    return deepest;
  }

  /** How far inside an outline a point is: its distance to the nearest side, or zero outside it. */
  private static double depthInside(Polygon2d outline, Translation2d point) {
    if (!outline.contains(point)) {
      return 0;
    }
    double nearestSide = Double.POSITIVE_INFINITY;
    for (int side = 0; side < outline.corners().size(); side++) {
      Translation2d from = outline.corner(side);
      Translation2d along = outline.corner(side + 1).minus(from);
      double fromSide = Math.abs(along.cross(point.minus(from))) / along.getNorm();
      nearestSide = Math.min(nearestSide, fromSide);
    }
    return nearestSide;
  }

  /** Points round an outline: each corner, and every twentieth of the way along each side. */
  private static List<Translation2d> around(Polygon2d outline) {
    List<Translation2d> points = new ArrayList<>();
    for (int side = 0; side < outline.corners().size(); side++) {
      Translation2d from = outline.corner(side);
      Translation2d to = outline.corner(side + 1);
      for (int part = 0; part < 20; part++) {
        points.add(from.interpolate(to, part / 20.0));
      }
    }
    return points;
  }
}
