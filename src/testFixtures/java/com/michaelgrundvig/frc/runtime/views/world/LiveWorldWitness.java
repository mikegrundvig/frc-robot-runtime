package com.michaelgrundvig.frc.runtime.views.world;

import com.michaelgrundvig.frc.pitwall.scene.Layer;
import com.michaelgrundvig.frc.pitwall.scene.SceneState;
import com.michaelgrundvig.frc.pitwall.scene.Shapes;
import com.michaelgrundvig.frc.pitwall.scene.StillState;
import com.michaelgrundvig.frc.pitwall.scene.Style;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.system.RobotController;

/**
 * The World view's live layers as a witness: every loop it has them say what moves, as they say it
 * to a page, and logs what they said under {@code Witness/World}. The check that the view drawn
 * from the record is the same view reads those back beside the record of the same loop.
 */
public final class LiveWorldWitness {
  /** The live layers' state, kept from loop to loop as a page keeps it, on the robot's clock. */
  private static final class Kept implements SceneState {
    private final StillState said = new StillState(0);

    @Override
    public long nowNanos() {
      return RobotController.getTime();
    }

    @Override
    public void pose(
        String frame, double x, double y, double z, double roll, double pitch, double yaw) {
      said.pose(frame, x, y, z, roll, pitch, yaw);
    }

    @Override
    public void value(String name, double value) {
      said.value(name, value);
    }

    @Override
    public void style(String shape, @Nullable Style style) {
      said.style(shape, style);
    }

    @Override
    public Shapes shapes() {
      return said.shapes();
    }
  }

  private final List<Layer> layers = new ArrayList<>();
  private final Kept kept = new Kept();

  /**
   * @param field the field the robot believes it plays on
   * @param robot the robot's definition
   * @param readings what the live view reads each loop
   * @param truth where the robot really is, in the simulator
   */
  public LiveWorldWitness(
      Supplier<FieldModel> field,
      RobotDefinition robot,
      WorldView.Readings readings,
      Supplier<Pose3d> truth) {
    layers.add(new RobotLayer(field.get(), robot, readings, camera -> Optional.empty()));
    layers.add(new Uncertainty(readings));
    layers.add(new SightLines(field, robot.cameras(), readings, camera -> Optional.empty()));
    layers.add(new TruthLayer(robot.body(), truth));
  }

  /** Has the live layers say what moves now, and logs all they have said. */
  public void log() {
    for (Layer layer : layers) {
      kept.said.layer(layer.id());
      layer.update(kept);
    }
    StillState said = kept.said;
    List<String> frames = new ArrayList<>(said.poses().keySet());
    double[] poses = new double[frames.size() * 6];
    for (int each = 0; each < frames.size(); each++) {
      System.arraycopy(said.poses().get(frames.get(each)), 0, poses, each * 6, 6);
    }
    List<String> names = new ArrayList<>(said.values().keySet());
    double[] values = new double[names.size()];
    for (int each = 0; each < values.length; each++) {
      values[each] = said.values().get(names.get(each));
    }
    List<String> styles = new ArrayList<>();
    said.styles().forEach((shape, style) -> styles.add(shape + "=" + style));
    List<String> lines = new ArrayList<>();
    for (Map<String, Object> line :
        said.shapesOfTheMoment().getOrDefault("sight-lines", List.of())) {
      lines.add(line.get("id") + " " + line.get("style") + " " + line.get("between"));
    }
    Logger.recordOutput("Witness/World/Frames", frames.toArray(String[]::new));
    Logger.recordOutput("Witness/World/Poses", poses);
    Logger.recordOutput("Witness/World/ValueNames", names.toArray(String[]::new));
    Logger.recordOutput("Witness/World/Values", values);
    Logger.recordOutput("Witness/World/Styles", styles.toArray(String[]::new));
    Logger.recordOutput("Witness/World/Lines", lines.toArray(String[]::new));
  }
}
