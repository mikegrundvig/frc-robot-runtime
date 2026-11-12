package com.michaelgrundvig.frc.toolconfigs.advantagescope;

import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.Arrays;
import java.util.List;

/**
 * Flat-shaded triangles in one color, built from simple solids: boxes, pyramids, wheels, arrows.
 * Every triangle carries its own corners and its face's normal, so nothing is smoothed: a
 * schematic, not a rendering. Points are in the robot's frame (x forward, y left, z up, meters).
 *
 * <p>Each solid is convex, and its faces are turned to face away from a point inside it, so every
 * normal points out however a face's corners were listed.
 */
final class Mesh {
  /**
   * How a mesh is colored: red, green and blue from 0 to 1, and how opaque, from 0 to 1. A glTF
   * viewer draws one under 1 see-through; AdvantageScope draws any under 0.75 at its own fixed
   * opacity.
   */
  record Material(String name, double red, double green, double blue, double alpha) {}

  /**
   * A flat arrow along its frame's x axis: a shaft, then a triangular head.
   *
   * @param tail where the shaft starts, along x
   * @param tip where the head ends, at a point
   * @param headLength how long the head is, along x
   * @param shaftWidth how wide the shaft is, along y
   * @param headWidth how wide the head is at its base
   * @param underside the arrow's underside, along z
   * @param top its top
   */
  record Arrow(
      double tail,
      double tip,
      double headLength,
      double shaftWidth,
      double headWidth,
      double underside,
      double top) {}

  /** One end of a box along one of its axes. */
  private enum End {
    LOW,
    HIGH
  }

  /** A corner has three numbers, and a triangle three corners. */
  private static final int NUMBERS_PER_CORNER = 3;

  private static final int CORNERS_PER_TRIANGLE = 3;

  /** Room for a box's twelve triangles to start with. The arrays double as they fill. */
  private static final int STARTING_CORNERS = 36;

  private final String name;
  private final Material material;

  /** Every corner's x, y and z in turn, with room to spare at the end. */
  private float[] positions = new float[NUMBERS_PER_CORNER * STARTING_CORNERS];

  /** Every corner's face normal, as the positions are listed. */
  private float[] normals = new float[NUMBERS_PER_CORNER * STARTING_CORNERS];

  /** How many corners have been added. */
  private int corners;

  Mesh(String name, Material material) {
    this.name = name;
    this.material = material;
  }

  String name() {
    return name;
  }

  Material material() {
    return material;
  }

  /** How many corners: three for each triangle. */
  int vertexCount() {
    return corners;
  }

  /** Each corner's x, y and z in turn, in the robot's frame. */
  float[] positions() {
    return Arrays.copyOf(positions, NUMBERS_PER_CORNER * corners);
  }

  /** Each corner's face normal, as positions lists them. */
  float[] normals() {
    return Arrays.copyOf(normals, NUMBERS_PER_CORNER * corners);
  }

  /** A box, square to its frame, between two opposite corners given in the frame. */
  Mesh box(Frame frame, Vector3 low, Vector3 high) {
    Vector3 inside = frame.place(low.plus(high).times(0.5));
    // Each face is named for the end of the box it is at, and lists its four corners in order
    // around it.
    face( // The bottom.
        inside,
        corner(frame, low, high, End.LOW, End.LOW, End.LOW),
        corner(frame, low, high, End.LOW, End.HIGH, End.LOW),
        corner(frame, low, high, End.HIGH, End.HIGH, End.LOW),
        corner(frame, low, high, End.HIGH, End.LOW, End.LOW));
    face( // The top.
        inside,
        corner(frame, low, high, End.LOW, End.LOW, End.HIGH),
        corner(frame, low, high, End.HIGH, End.LOW, End.HIGH),
        corner(frame, low, high, End.HIGH, End.HIGH, End.HIGH),
        corner(frame, low, high, End.LOW, End.HIGH, End.HIGH));
    face( // The side at low y.
        inside,
        corner(frame, low, high, End.LOW, End.LOW, End.LOW),
        corner(frame, low, high, End.HIGH, End.LOW, End.LOW),
        corner(frame, low, high, End.HIGH, End.LOW, End.HIGH),
        corner(frame, low, high, End.LOW, End.LOW, End.HIGH));
    face( // The side at high y.
        inside,
        corner(frame, low, high, End.LOW, End.HIGH, End.LOW),
        corner(frame, low, high, End.LOW, End.HIGH, End.HIGH),
        corner(frame, low, high, End.HIGH, End.HIGH, End.HIGH),
        corner(frame, low, high, End.HIGH, End.HIGH, End.LOW));
    face( // The side at low x.
        inside,
        corner(frame, low, high, End.LOW, End.LOW, End.LOW),
        corner(frame, low, high, End.LOW, End.LOW, End.HIGH),
        corner(frame, low, high, End.LOW, End.HIGH, End.HIGH),
        corner(frame, low, high, End.LOW, End.HIGH, End.LOW));
    face( // The side at high x.
        inside,
        corner(frame, low, high, End.HIGH, End.LOW, End.LOW),
        corner(frame, low, high, End.HIGH, End.HIGH, End.LOW),
        corner(frame, low, high, End.HIGH, End.HIGH, End.HIGH),
        corner(frame, low, high, End.HIGH, End.LOW, End.HIGH));
    return this;
  }

  /**
   * One of a box's eight corners, in the robot's frame: the corner at the given end of the box
   * along each of its frame's axes.
   */
  private static Vector3 corner(
      Frame frame, Vector3 low, Vector3 high, End alongX, End alongY, End alongZ) {
    double x = alongX == End.LOW ? low.x() : high.x();
    double y = alongY == End.LOW ? low.y() : high.y();
    double z = alongZ == End.LOW ? low.z() : high.z();
    return frame.place(new Vector3(x, y, z));
  }

  /**
   * A pyramid with a four-cornered base: a camera's view out to its sight range, or an arrow's
   * head. Points are given in the frame, the base's corners in order around it.
   */
  Mesh pyramid(Frame frame, Vector3 apex, List<Vector3> base) {
    Vector3 tip = frame.place(apex);
    Vector3[] baseCorners = base.stream().map(frame::place).toArray(Vector3[]::new);
    // The average of all its corners is inside it.
    Vector3 sum = tip;
    for (Vector3 corner : baseCorners) {
      sum = sum.plus(corner);
    }
    Vector3 inside = sum.times(1.0 / (baseCorners.length + 1));
    for (int index = 0; index < baseCorners.length; index++) {
      Vector3 next = baseCorners[(index + 1) % baseCorners.length];
      face(inside, tip, baseCorners[index], next);
    }
    face(inside, baseCorners);
    return this;
  }

  /**
   * A wheel: a cylinder around its frame's y axis, centered on the frame's origin.
   *
   * @param halfWidth half the wheel's width, along y
   * @param sides how many flat faces stand in for the round tread
   */
  Mesh wheel(Frame frame, double radius, double halfWidth, int sides) {
    // The rim's corners, evenly spaced around the axle, on the wheel's left side and its right.
    Vector3[] left = new Vector3[sides];
    Vector3[] right = new Vector3[sides];
    for (int side = 0; side < sides; side++) {
      double angleAround = 2 * StrictMath.PI * side / sides;
      double x = radius * StrictMath.cos(angleAround);
      double z = radius * StrictMath.sin(angleAround);
      left[side] = frame.place(new Vector3(x, halfWidth, z));
      right[side] = frame.place(new Vector3(x, -halfWidth, z));
    }
    Vector3 inside = frame.place(Vector3.ZERO);
    // The tread, one flat face between each pair of neighboring corners, and then the two sides.
    for (int side = 0; side < sides; side++) {
      int next = (side + 1) % sides;
      face(inside, left[side], left[next], right[next], right[side]);
    }
    face(inside, left);
    face(inside, right);
    return this;
  }

  /** A flat arrow, given in a frame. Its shaft is a box, and its head a triangular prism. */
  Mesh arrow(Frame frame, Arrow arrow) {
    // The neck is where the shaft ends and the head begins.
    double neck = arrow.tip() - arrow.headLength();
    double shaftHalfWidth = arrow.shaftWidth() / 2;
    double headHalfWidth = arrow.headWidth() / 2;
    box(
        frame,
        new Vector3(arrow.tail(), -shaftHalfWidth, arrow.underside()),
        new Vector3(neck, shaftHalfWidth, arrow.top()));
    Vector3[] headUnderside =
        headOutline(frame, neck, arrow.tip(), headHalfWidth, arrow.underside());
    Vector3[] headTop = headOutline(frame, neck, arrow.tip(), headHalfWidth, arrow.top());
    // A third of the way from the head's base to its tip, halfway up, is inside it.
    double midHeight = (arrow.underside() + arrow.top()) / 2;
    Vector3 inside = frame.place(new Vector3(neck + arrow.headLength() / 3, 0, midHeight));
    face(inside, headUnderside);
    face(inside, headTop);
    for (int corner = 0; corner < 3; corner++) {
      int next = (corner + 1) % 3;
      face(inside, headUnderside[corner], headUnderside[next], headTop[next], headTop[corner]);
    }
    return this;
  }

  /** An arrow head's triangle at a height: its base's two ends, with its tip between them. */
  private static Vector3[] headOutline(
      Frame frame, double neck, double tip, double halfWidth, double height) {
    return new Vector3[] {
      frame.place(new Vector3(neck, -halfWidth, height)),
      frame.place(new Vector3(tip, 0, height)),
      frame.place(new Vector3(neck, halfWidth, height))
    };
  }

  /**
   * A flat, convex face, cut into a fan of triangles from its first corner, each turned to face
   * away from a point inside the solid.
   *
   * <p>A triangle's normal is the cross product of two of its sides, and points toward whoever sees
   * its corners run counterclockwise. If that is toward the inside, the triangle is listed the
   * other way round and its normal reversed.
   */
  private void face(Vector3 inside, Vector3... faceCorners) {
    for (int index = 1; index + 1 < faceCorners.length; index++) {
      Vector3 a = faceCorners[0];
      Vector3 b = faceCorners[index];
      Vector3 c = faceCorners[index + 1];
      Vector3 normal = b.minus(a).cross(c.minus(a));
      boolean hasNoArea = normal.length() == 0;
      if (hasNoArea) {
        continue;
      }
      boolean facesInward = normal.dot(a.minus(inside)) < 0;
      if (facesInward) {
        triangle(a, c, b, normal.times(-1).unit());
      } else {
        triangle(a, b, c, normal.unit());
      }
    }
  }

  private void triangle(Vector3 a, Vector3 b, Vector3 c, Vector3 normal) {
    boolean full = NUMBERS_PER_CORNER * (corners + CORNERS_PER_TRIANGLE) > positions.length;
    if (full) {
      positions = Arrays.copyOf(positions, 2 * positions.length);
      normals = Arrays.copyOf(normals, 2 * normals.length);
    }
    for (Vector3 corner : new Vector3[] {a, b, c}) {
      int at = NUMBERS_PER_CORNER * corners;
      corners++;
      positions[at] = (float) corner.x();
      positions[at + 1] = (float) corner.y();
      positions[at + 2] = (float) corner.z();
      normals[at] = (float) normal.x();
      normals[at + 1] = (float) normal.y();
      normals[at + 2] = (float) normal.z();
    }
  }
}
