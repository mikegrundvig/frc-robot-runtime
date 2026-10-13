package com.michaelgrundvig.frc.simulation;

import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.field.Element;
import com.michaelgrundvig.frc.world.physical.Polygon2d;
import com.michaelgrundvig.frc.world.robot.Body;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.math.geometry.Translation2d;

/**
 * What the robot's bumpers run into: the walls around the field, and each field element that
 * reaches into the height its bumpers are at. Given where the robot's wheels would put it, it says
 * how far the robot has to be pushed back to be clear of all of them ({@link #pushOut}).
 *
 * <p>The push is always straight out of a surface, along the surface's own direction. So the part
 * of a motion that goes into a surface is removed and the part along it is kept: the robot stops
 * against a wall it meets head on, and slides along one it meets at an angle. Nothing bounces.
 *
 * <p>The bumpers are a rectangle, and every outline is one with no corner pointing in ({@link
 * com.michaelgrundvig.frc.world.field.Block}). Two such shapes overlap only if they overlap along
 * every direction that is square to one of their sides, and the shortest way to part them is along
 * the one of those directions where they overlap least (the separating axis theorem).
 *
 * <p>It keeps numbers only and makes no objects as it answers, and the same question always gets
 * the same answer.
 */
final class Contact {
  /**
   * Open space: nothing to run into. Each world gets its own, because a contact keeps what it is
   * working out, and worlds run side by side.
   */
  static Contact openSpace() {
    return new Contact(0, 0, new Outline[0], new Outline[0]);
  }

  /**
   * How deep an overlap has to be before the robot is pushed out of it, in meters. A robot resting
   * against a wall overlaps it by a rounding error or by none, and is left where it is.
   */
  private static final double TOUCHING_METERS = 1e-9;

  /**
   * How many times each surface is tried. Pushing the robot out of one surface can push it into
   * another, in a corner, so they are tried again until none pushes. A corner between two walls
   * settles in two passes; the cap is there for a robot set down deep among several elements.
   */
  private static final int MOST_PASSES = 16;

  private static final int BUMPER_CORNERS = 4;

  /** From the robot's center to the front of its bumpers, and to their left side. */
  private final double halfLengthMeters;

  private final double halfWidthMeters;

  /** Outlines the robot stays inside of: the field's walls. */
  private final Outline[] walls;

  /** Outlines the robot stays outside of: what stands on the field at its bumpers' height. */
  private final Outline[] obstacles;

  /** The bumpers' corners on the field, as x then y for each, worked out afresh for each try. */
  private final double[] corners = new double[2 * BUMPER_CORNERS];

  // The robot as it is being pushed out: its center, and which way it faces as a unit vector.
  private double centerX;
  private double centerY;
  private double forwardX;
  private double forwardY;

  // The shortest push found so far out of the obstacle being tried: how far, and which way.
  private double leastOverlapMeters;
  private double leastOverlapWayX;
  private double leastOverlapWayY;

  // How far the last question pushed the robot, in all.
  private double pushedXMeters;
  private double pushedYMeters;

  /**
   * An outline as numbers: its corners, counterclockwise, and for each side the unit direction
   * square to it that points out of the outline.
   */
  private static final class Outline {
    final double[] cornerX;
    final double[] cornerY;
    final double[] outwardX;
    final double[] outwardY;

    Outline(Polygon2d polygon) {
      int count = polygon.corners().size();
      cornerX = new double[count];
      cornerY = new double[count];
      outwardX = new double[count];
      outwardY = new double[count];
      for (int side = 0; side < count; side++) {
        Translation2d from = polygon.corner(side);
        Translation2d along = polygon.corner(side + 1).minus(from);
        double length = along.getNorm();
        cornerX[side] = from.getX();
        cornerY[side] = from.getY();
        // The inside of a counterclockwise outline is on each side's left, so out is to its right:
        // the side's direction turned a quarter turn clockwise.
        outwardX[side] = along.getY() / length;
        outwardY[side] = -along.getX() / length;
      }
    }
  }

  /**
   * What a robot of this size runs into on a field.
   *
   * @param body the robot, whose bumpers are what touch: their outline, and how high they are
   * @param walls the inside faces of the walls around the field, which the robot stays within
   * @param elements what stands on the field; only what reaches into the bumpers' height counts
   */
  Contact(Body body, Polygon2d walls, List<Element> elements) {
    this(
        body.bumperLength().in(Meters) / 2,
        body.bumperWidth().in(Meters) / 2,
        new Outline[] {new Outline(walls)},
        inTheBumpersWay(body, elements));
  }

  private Contact(
      double halfLengthMeters, double halfWidthMeters, Outline[] walls, Outline[] obstacles) {
    this.halfLengthMeters = halfLengthMeters;
    this.halfWidthMeters = halfWidthMeters;
    this.walls = walls;
    this.obstacles = obstacles;
  }

  /** The outline of every block that stands between the bumpers' bottom edge and their top. */
  private static Outline[] inTheBumpersWay(Body body, List<Element> elements) {
    double bumperBottom = body.bumperBottom().in(Meters);
    double bumperTop = body.bumperTop().in(Meters);
    List<Outline> outlines = new ArrayList<>();
    for (Element element : elements) {
      for (Polygon2d outline : element.inTheWayOf(bumperBottom, bumperTop)) {
        outlines.add(new Outline(outline));
      }
    }
    return outlines.toArray(new Outline[0]);
  }

  /**
   * Finds how far a robot at a pose has to move to be clear of everything, and keeps the answer
   * ({@link #pushedXMeters}, {@link #pushedYMeters}).
   *
   * @param x the robot's center on the field, in meters
   * @param y the robot's center on the field, in meters
   * @param headingRadians which way it faces
   * @return whether it had to move at all
   */
  boolean pushOut(double x, double y, double headingRadians) {
    centerX = x;
    centerY = y;
    forwardX = Math.cos(headingRadians);
    forwardY = Math.sin(headingRadians);
    for (int pass = 0; pass < MOST_PASSES; pass++) {
      boolean pushed = false;
      for (Outline wall : walls) {
        pushed |= pushedBackInside(wall);
      }
      for (Outline obstacle : obstacles) {
        pushed |= pushedOutOf(obstacle);
      }
      if (!pushed) {
        break;
      }
    }
    pushedXMeters = centerX - x;
    pushedYMeters = centerY - y;
    return pushedXMeters != 0 || pushedYMeters != 0;
  }

  /** How far the last {@link #pushOut} moved the robot along the field's x, in meters. */
  double pushedXMeters() {
    return pushedXMeters;
  }

  /** How far the last {@link #pushOut} moved the robot along the field's y, in meters. */
  double pushedYMeters() {
    return pushedYMeters;
  }

  /**
   * A velocity with the part that goes into the surfaces the robot was last pushed out of taken
   * away: what is left runs along them, or away. A robot that was not pushed keeps all of it.
   */
  Translation2d alongTheSurface(Translation2d velocity) {
    double pushedMeters = Math.hypot(pushedXMeters, pushedYMeters);
    if (pushedMeters == 0) {
      return velocity;
    }
    double outX = pushedXMeters / pushedMeters;
    double outY = pushedYMeters / pushedMeters;
    double speedOut = velocity.getX() * outX + velocity.getY() * outY;
    if (speedOut >= 0) {
      return velocity;
    }
    return new Translation2d(velocity.getX() - speedOut * outX, velocity.getY() - speedOut * outY);
  }

  /** Works out where the bumpers' four corners are, for the robot where it now is. */
  private void placeCorners() {
    int corner = 0;
    for (int ahead = -1; ahead <= 1; ahead += 2) {
      for (int left = -1; left <= 1; left += 2) {
        double forward = ahead * halfLengthMeters;
        double sideways = left * halfWidthMeters;
        // The robot's left is its forward direction turned a quarter turn: (-forwardY, forwardX).
        corners[2 * corner] = centerX + forward * forwardX - sideways * forwardY;
        corners[2 * corner + 1] = centerY + forward * forwardY + sideways * forwardX;
        corner++;
      }
    }
  }

  /**
   * Moves the robot back within an outline it has a corner outside of: for each side in turn,
   * straight in by as far as its farthest corner is past that side.
   */
  private boolean pushedBackInside(Outline wall) {
    boolean pushed = false;
    for (int side = 0; side < wall.cornerX.length; side++) {
      placeCorners();
      double farthestPast = 0;
      for (int corner = 0; corner < BUMPER_CORNERS; corner++) {
        double past =
            (corners[2 * corner] - wall.cornerX[side]) * wall.outwardX[side]
                + (corners[2 * corner + 1] - wall.cornerY[side]) * wall.outwardY[side];
        farthestPast = Math.max(farthestPast, past);
      }
      if (farthestPast > TOUCHING_METERS) {
        centerX -= farthestPast * wall.outwardX[side];
        centerY -= farthestPast * wall.outwardY[side];
        pushed = true;
      }
    }
    return pushed;
  }

  /**
   * Moves the robot out of an outline it overlaps, the shortest way. Each side of the outline, and
   * each side of the bumpers, is a direction to part them along; if they don't overlap along any
   * one of those, they don't overlap at all.
   */
  private boolean pushedOutOf(Outline obstacle) {
    placeCorners();
    leastOverlapMeters = Double.POSITIVE_INFINITY;
    for (int side = 0; side < obstacle.cornerX.length; side++) {
      // How far the bumpers' deepest corner is behind this side of the obstacle.
      double deepest = Double.POSITIVE_INFINITY;
      for (int corner = 0; corner < BUMPER_CORNERS; corner++) {
        double past =
            (corners[2 * corner] - obstacle.cornerX[side]) * obstacle.outwardX[side]
                + (corners[2 * corner + 1] - obstacle.cornerY[side]) * obstacle.outwardY[side];
        deepest = Math.min(deepest, past);
      }
      // The robot leaves through this side by moving the way the side faces.
      if (!overlapsBy(-deepest, obstacle.outwardX[side], obstacle.outwardY[side])) {
        return false;
      }
    }
    // The bumpers' own four sides: front, back, left and right. The robot leaves through one of
    // its sides by moving the opposite way.
    boolean overlapsAlongItsOwnSides =
        overlapsPastBumper(obstacle, forwardX, forwardY, halfLengthMeters)
            && overlapsPastBumper(obstacle, -forwardX, -forwardY, halfLengthMeters)
            && overlapsPastBumper(obstacle, -forwardY, forwardX, halfWidthMeters)
            && overlapsPastBumper(obstacle, forwardY, -forwardX, halfWidthMeters);
    if (!overlapsAlongItsOwnSides) {
      return false;
    }
    centerX += leastOverlapMeters * leastOverlapWayX;
    centerY += leastOverlapMeters * leastOverlapWayY;
    return true;
  }

  /**
   * Whether an obstacle reaches in past one side of the bumpers, noting how far.
   *
   * @param outX the way that side of the bumpers faces, as a unit vector
   * @param outY the way that side of the bumpers faces, as a unit vector
   * @param halfExtentMeters how far that side is from the robot's center
   */
  private boolean overlapsPastBumper(
      Outline obstacle, double outX, double outY, double halfExtentMeters) {
    double deepest = Double.POSITIVE_INFINITY;
    for (int corner = 0; corner < obstacle.cornerX.length; corner++) {
      double past =
          (obstacle.cornerX[corner] - centerX) * outX
              + (obstacle.cornerY[corner] - centerY) * outY
              - halfExtentMeters;
      deepest = Math.min(deepest, past);
    }
    return overlapsBy(-deepest, -outX, -outY);
  }

  /**
   * Takes note of how far the two overlap along one direction, and which way the robot would leave
   * along it, if that is the least so far.
   *
   * @return whether they overlap along it at all, past touching
   */
  private boolean overlapsBy(double overlapMeters, double wayOutX, double wayOutY) {
    if (overlapMeters <= TOUCHING_METERS) {
      return false;
    }
    if (overlapMeters < leastOverlapMeters) {
      leastOverlapMeters = overlapMeters;
      leastOverlapWayX = wayOutX;
      leastOverlapWayY = wayOutY;
    }
    return true;
  }
}
