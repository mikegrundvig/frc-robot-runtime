package com.michaelgrundvig.frc.runtime.views.world;

import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.pitwall.scene.Layer;
import com.michaelgrundvig.frc.pitwall.scene.SceneBuilder;
import com.michaelgrundvig.frc.pitwall.scene.SceneState;
import com.michaelgrundvig.frc.pitwall.scene.Style;
import com.michaelgrundvig.frc.world.robot.Body;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose3d;

/**
 * The simulator only: where the robot really is, as a dashed ghost of its bumpers beside the
 * estimate. The gap between the two is what localization gets wrong.
 */
final class TruthLayer implements Layer {
  private static final String FRAME = "truth";

  private final Body body;
  private final Supplier<Pose3d> truth;

  TruthLayer(Body body, Supplier<Pose3d> truth) {
    this.body = body;
    this.truth = truth;
  }

  @Override
  public String id() {
    return "truth";
  }

  @Override
  public String title() {
    return "Truth (dashed)";
  }

  @Override
  public void describe(SceneBuilder scene) {
    double length = body.bumperLength().in(Meters);
    double height = body.height().in(Meters);
    scene.frame(FRAME, SceneBuilder.ROOT);
    scene
        .box("truth.robot", FRAME)
        .size(length, body.bumperWidth().in(Meters), height)
        .at(0, 0, height / 2)
        .outline()
        .style(Style.TRUTH.dashed());
    scene
        .arrow("truth.front", FRAME)
        .from(0, 0, height + 0.02)
        .to(length / 2 + 0.2, 0, height + 0.02)
        .style(Style.TRUTH.dashed());
  }

  @Override
  public void update(SceneState state) {
    state.pose(FRAME, truth.get());
  }
}
