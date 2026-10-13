package com.michaelgrundvig.frc.runtime;

import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.field.Element;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.physical.Polygon2d;
import com.michaelgrundvig.frc.world.robot.Body;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * How much room a robot's bumpers have on a field: the gap between their outline and the nearest
 * wall or field element that stands at their height. A test of an auto on the season's own field
 * holds the robot's true path to it, so a path that only just scrapes past a hub fails before a
 * robot finds out.
 */
public final class Clearance {
  private final FieldModel field;
  private final double bumperLength;
  private final double bumperWidth;

  /** The outline of every block standing at the height of the robot's bumpers. */
  private final List<Polygon2d> standing = new ArrayList<>();

  /**
   * @param field the field, with what stands on it
   * @param body the robot, whose bumpers are what would touch
   */
  public Clearance(FieldModel field, Body body) {
    this.field = field;
    bumperLength = body.bumperLength().in(Meters);
    bumperWidth = body.bumperWidth().in(Meters);
    double bumperBottom = body.bumperBottom().in(Meters);
    double bumperTop = body.bumperTop().in(Meters);
    for (Element element : field.elements()) {
      standing.addAll(element.inTheWayOf(bumperBottom, bumperTop));
    }
  }

  /**
   * The gap around the bumpers of a robot at a pose, in meters: zero when they touch a wall or an
   * element, or are in one.
   */
  public double at(Pose2d robot) {
    Polygon2d bumpers = Polygon2d.rectangle(robot, bumperLength, bumperWidth);
    double gap = Double.POSITIVE_INFINITY;
    for (Translation2d corner : bumpers.corners()) {
      gap = Math.min(gap, fromTheNearestWall(corner));
    }
    for (Polygon2d outline : standing) {
      gap = Math.min(gap, between(bumpers, outline));
    }
    return Math.max(gap, 0);
  }

  /** How far inside the field's walls a point is: negative outside them. */
  private double fromTheNearestWall(Translation2d point) {
    double alongTheField = Math.min(point.getX(), field.length() - point.getX());
    double acrossTheField = Math.min(point.getY(), field.width() - point.getY());
    return Math.min(alongTheField, acrossTheField);
  }

  /**
   * The gap between two outlines with no corner pointing in: zero if they touch, and otherwise the
   * least distance from a corner of either to the other, which is where two such outlines are
   * nearest.
   */
  private static double between(Polygon2d one, Polygon2d other) {
    if (one.touches(other)) {
      return 0;
    }
    double gap = Double.POSITIVE_INFINITY;
    for (Translation2d corner : one.corners()) {
      gap = Math.min(gap, other.distance(corner));
    }
    for (Translation2d corner : other.corners()) {
      gap = Math.min(gap, one.distance(corner));
    }
    return gap;
  }
}
