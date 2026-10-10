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
 * The field as the field model has it: its walls, its raised ground (bumps, ramps, platforms), and
 * its AprilTags where the layout puts them, each with its number. Nothing here moves.
 */
final class FieldLayer implements Layer {
  /** An AprilTag's printed square, as FRC's 36h11 tags are: 6.5 in. */
  private static final double TAG_SIZE = 0.1651;

  private final Supplier<FieldModel> active;

  /**
   * @param active the field the robot believes it plays on: the selected venue's, once there is one
   */
  FieldLayer(Supplier<FieldModel> active) {
    this.active = active;
  }

  /** The field in use: a new one (a venue selected) is described again. */
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
    double length = field.length();
    double width = field.width();
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
    List<Surface> surfaces = field.terrain().surfaces();
    for (int i = 0; i < surfaces.size(); i++) {
      surface(scene, "surface." + i, surfaces.get(i));
    }
    for (Tag tag : field.tags().values()) {
      int id = tag.id();
      Pose3d pose = tag.pose();
      String frame = WorldView.tagFrame(id);
      scene.frame(frame, SceneBuilder.ROOT, pose);
      // The tag's x points out of its face: its square stands in the y-z plane.
      scene
          .box(frame + ".square", frame)
          .size(0.01, TAG_SIZE, TAG_SIZE)
          .inspect(() -> tag(id, pose));
      // In front of its face: tags back to back, or side by side on one panel, keep apart.
      scene.label(frame + ".id", frame).text(Integer.toString(id)).at(0.25, 0, TAG_SIZE);
    }
  }

  /** Raised ground, as its triangles: a bump's ridge, a ramp, a platform and its ramps. */
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

  /** How high it stands, how steep its slopes are, and how long each line it can be crossed at. */
  private static Inspection inspection(Surface surface) {
    double height = surface.mesh().corners().stream().mapToDouble(Vector3::z).max().orElse(0);
    Inspection inspection =
        Inspection.of(surface.name()).detail("Height", WorldView.meters(height));
    if (surface.steepestSlopeRadians() > 0) {
      inspection =
          inspection.detail(
              "Ramps",
              String.format(Locale.ROOT, "%.1f°", Math.toDegrees(surface.steepestSlopeRadians())));
    }
    for (Crossing crossing : surface.crossings()) {
      inspection =
          inspection.detail(
              crossing.kind() == Crossing.Kind.CREST ? "Crest" : "Top",
              WorldView.meters(crossing.length()) + " long");
    }
    return inspection;
  }

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
