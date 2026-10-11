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
 * Where the robot really is, as a dashed ghost of its bumpers beside the estimate. Only the
 * simulator knows that, so only the simulator has this layer. The gap between the ghost and the
 * estimate is what localization gets wrong.
 */
final class TruthLayer implements Layer {
  private static final String FRAME = "truth";

  /** How far above the ghost's top its arrow out the front is drawn, and how far past its front. */
  private static final double FRONT_ARROW_RISE = 0.02;

  private static final double FRONT_ARROW_REACH = 0.2;

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
        .from(0, 0, height + FRONT_ARROW_RISE)
        .to(length / 2 + FRONT_ARROW_REACH, 0, height + FRONT_ARROW_RISE)
        .style(Style.TRUTH.dashed());
  }

  @Override
  public void update(SceneState state) {
    state.pose(FRAME, truth.get());
  }
}
