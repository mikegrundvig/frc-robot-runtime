package com.michaelgrundvig.frc.runtime.views.world;

import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.pitwall.scene.Layer;
import com.michaelgrundvig.frc.pitwall.scene.SceneBuilder;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

/**
 * Each camera's full view, out as far as it reads tags (its sensor's sight range). The layer is off
 * until switched on, since four views reach across the field. One camera's view is also shown while
 * that camera is clicked, because {@link RobotLayer}'s camera reveals its own. The views are drawn
 * in the cameras' frames, which the robot's layer adds.
 */
final class CameraViews implements Layer {
  private final List<CameraConfig> cameras;
  private final Function<String, Optional<String>> cameraAction;

  CameraViews(List<CameraConfig> cameras, Function<String, Optional<String>> cameraAction) {
    this.cameras = List.copyOf(cameras);
    this.cameraAction = cameraAction;
  }

  /** The id of the shape that is a camera's full view. */
  static String view(CameraConfig camera) {
    return WorldView.cameraFrame(camera.id()) + ".view";
  }

  @Override
  public String id() {
    return "camera-views";
  }

  @Override
  public String title() {
    return "Camera views";
  }

  @Override
  public void describe(SceneBuilder scene) {
    for (CameraConfig camera : cameras) {
      RobotLayer.FieldOfView sees = RobotLayer.fieldOfView(camera.sensor());
      scene
          .frustum(view(camera), WorldView.cameraFrame(camera.id()))
          .fov(sees.acrossRadians(), sees.upAndDownRadians())
          .range(0, camera.sensor().sightRange().in(Meters))
          .inspect(() -> WorldView.camera(camera, cameraAction));
    }
  }
}
