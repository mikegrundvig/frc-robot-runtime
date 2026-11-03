package com.michaelgrundvig.frc.runtime.examples;

import com.michaelgrundvig.frc.world.robot.CameraConfig.Software;
import com.michaelgrundvig.frc.world.robot.CameraConfig.Software.Lens;
import java.util.List;

/**
 * A camera on Visor as robot code says it (docs/visor.md): its board, device, settings and lens.
 */
final class VisorCameraExample {
  private VisorCameraExample() {}

  /** The software of a camera on Visor, with what the deploy needs. */
  static final Software.Visor FRONT_LEFT =
      Software.visor()
          .coprocessor("vision-a")
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
}
