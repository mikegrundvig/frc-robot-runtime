package com.michaelgrundvig.frc.runtime.views.world;

import com.michaelgrundvig.frc.pitwall.scene.Inspection;
import com.michaelgrundvig.frc.pitwall.scene.Layer;
import com.michaelgrundvig.frc.pitwall.scene.SceneBuilder;
import com.michaelgrundvig.frc.pitwall.scene.Shape;
import com.michaelgrundvig.frc.world.field.Crossing;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Surface;
import com.michaelgrundvig.frc.world.field.Tag;
import com.michaelgrundvig.frc.world.physical.TriangleMesh;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.List;
import java.util.Locale;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose3d;

/**
 * The field as the field model has it: its carpet and walls, its raised ground (bumps, ramps,
 * platforms), and its AprilTags where the layout puts them, each with its number. Nothing here
 * moves.
 */
final class FieldLayer implements Layer {
  /** The side of an AprilTag's printed square, in meters. FRC's 36h11 tags are 6.5 in. */
  private static final double TAG_SIZE = 0.1651;

  /** How thick a tag's square is drawn. */
  private static final double TAG_THICKNESS = 0.01;

  /** How far in front of its face a tag's number is drawn. */
  private static final double TAG_NUMBER_OUT = 0.25;

  private final Supplier<FieldModel> active;

  /**
   * @param active the field the robot believes it plays on, which is the selected venue's once
   *     there is one
   */
  FieldLayer(Supplier<FieldModel> active) {
    this.active = active;
  }

  /**
   * The field in use. When it is a new one, because a venue was selected, Pitwall has the layer
   * describe itself again.
   */
  @Override
  public Object configuration() {
    return active.get();
  }

  @Override
  public String id() {
    return "field";
  }

  @Override
  public String title() {
    return "Field";
  }

  @Override
  public void describe(SceneBuilder scene) {
    FieldModel field = active.get();
    describeCarpetAndWalls(scene, field.length(), field.width());
    List<Surface> surfaces = field.terrain().surfaces();
    for (int i = 0; i < surfaces.size(); i++) {
      surface(scene, "surface." + i, surfaces.get(i));
    }
    for (Tag tag : field.tags().values()) {
      describeTag(scene, tag.id(), tag.pose());
    }
  }

  /** The carpet as a filled rectangle from the field's origin, and the walls as its outline. */
  private static void describeCarpetAndWalls(SceneBuilder scene, double length, double width) {
    scene
        .polygon("carpet", SceneBuilder.ROOT)
        .point(0, 0, 0)
        .point(length, 0, 0)
        .point(length, width, 0)
        .point(0, width, 0);
    scene
        .polyline("walls", SceneBuilder.ROOT)
        .point(0, 0, 0)
        .point(length, 0, 0)
        .point(length, width, 0)
        .point(0, width, 0)
        .closed();
  }

  /** A tag in a frame of its own at its pose: its square, and its number in front of it. */
  private static void describeTag(SceneBuilder scene, int id, Pose3d pose) {
    String frame = WorldView.tagFrame(id);
    scene.frame(frame, SceneBuilder.ROOT, pose);
    // The tag's x points out of its face, so its square stands in the y-z plane.
    scene
        .box(frame + ".square", frame)
        .size(TAG_THICKNESS, TAG_SIZE, TAG_SIZE)
        .inspect(() -> tag(id, pose));
    // The number is drawn in front of the face, so that the numbers of tags back to back, or side
    // by side on one panel, keep apart.
    scene.label(frame + ".id", frame).text(Integer.toString(id)).at(TAG_NUMBER_OUT, 0, TAG_SIZE);
  }

  /** Raised ground as its triangles: a bump's ridge, a ramp, or a platform and its ramps. */
  private static void surface(SceneBuilder scene, String id, Surface surface) {
    TriangleMesh mesh = surface.mesh();
    Shape.Mesh shape = scene.mesh(id, SceneBuilder.ROOT);
    for (Vector3 corner : mesh.corners()) {
      shape.vertex(corner.x(), corner.y(), corner.z());
    }
    for (TriangleMesh.Triangle triangle : mesh.triangles()) {
      shape.triangle(triangle.a(), triangle.b(), triangle.c());
    }
    shape.inspect(() -> inspection(surface));
  }

  /**
   * What a click on raised ground says: how high it stands, how steep its slopes are, and how long
   * each line is that it can be crossed at.
   */
  private static Inspection inspection(Surface surface) {
    double height = surface.mesh().corners().stream().mapToDouble(Vector3::z).max().orElse(0);
    Inspection inspection =
        Inspection.of(surface.name()).detail("Height", WorldView.meters(height));
    boolean sloped = surface.steepestSlopeRadians() > 0;
    if (sloped) {
      inspection =
          inspection.detail(
              "Ramps",
              String.format(Locale.ROOT, "%.1f°", Math.toDegrees(surface.steepestSlopeRadians())));
    }
    for (Crossing crossing : surface.crossings()) {
      String kind = "Top";
      if (crossing.kind() == Crossing.Kind.CREST) {
        kind = "Crest";
      }
      inspection = inspection.detail(kind, WorldView.meters(crossing.length()) + " long");
    }
    return inspection;
  }

  /** What a click on a tag says: where the layout has it, and which way it faces. */
  private static Inspection tag(int id, Pose3d pose) {
    return Inspection.of("AprilTag " + id)
        .detail(
            "Where",
            String.format(
                Locale.ROOT,
                "x %.3f m, y %.3f m, %.3f m up",
                pose.getX(),
                pose.getY(),
                pose.getZ()))
        .detail(
            "Facing",
            String.format(Locale.ROOT, "%.1f°", Math.toDegrees(pose.getRotation().getZ())))
        .detail("From", "the field's layout, as the robot program has it");
  }
}
