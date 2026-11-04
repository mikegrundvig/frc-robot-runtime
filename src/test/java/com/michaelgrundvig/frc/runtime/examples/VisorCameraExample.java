package com.michaelgrundvig.frc.runtime.examples;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotConfig;
import com.michaelgrundvig.frc.runtime.RobotConfigBuilder;
import com.michaelgrundvig.frc.spotter.manager.BoardDefinition;
import com.michaelgrundvig.frc.world.robot.CameraConfig.Software;
import com.michaelgrundvig.frc.world.robot.CameraConfig.Software.Lens;
import java.util.List;

/** A camera on Visor as robot code says it (docs/visor.md): its device, settings and lens. */
final class VisorCameraExample {
  private VisorCameraExample() {}

  /** The software of a camera on Visor, with what the robot configures Visor with. */
  static final Software.Visor FRONT_LEFT =
      Software.visor()
          .device("/dev/v4l/by-id/usb-Thrifty_ThriftyCam_0001-video-index0")
          .controls(
              List.of(
                  Software.Control.of("auto_exposure", 1),
                  Software.Control.of("exposure_time_absolute", 10)))
          .lens(
              Lens.builder()
                  .width(1600)
                  .height(1304)
                  .fx(1229.712)
                  .fy(1229.954)
                  .cx(801.183)
                  .cy(650.427)
                  .distortion(List.of(-0.3117, 0.1204, 0.00041, -0.00022, -0.0208))
                  .build())
          .build();

  /**
   * A robot's coprocessors, each by its Spotter agent and the packs the robot pushes it: Visor's.
   * Spotter's debian pack, for the board's own health, comes with the board's image.
   */
  static final RobotConfig WITH_COPROCESSORS =
      RobotConfigBuilder.builder(ExampleRobot.COMPETITION)
          .coprocessors(
              List.of(
                  new BoardDefinition("10.12.34.11", List.of("visor")),
                  new BoardDefinition("10.12.34.12", List.of("visor"))))
          .build();
}
