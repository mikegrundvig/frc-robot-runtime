package com.michaelgrundvig.frc.runtime.examples;

import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Percent;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfig.Software;
import com.michaelgrundvig.frc.world.robot.CameraConfig.Software.Lens;
import java.util.List;

/**
 * A camera on Visor as robot code says it (docs/visor.md): its device, settings and lens. Each is a
 * ThriftyCam, which has no USB serial, so each is named by the USB port it's plugged into.
 */
final class VisorCameraExample {
  private VisorCameraExample() {}

  /**
   * The software of a camera on Visor, with what the robot configures Visor with. It says no
   * exposure or gain, so it runs the defaults: a fixed 1 ms, the camera's automatic exposure off,
   * at the camera's own gain.
   */
  static final Software.Visor FRONT_LEFT =
      Software.visor()
          .device("/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:1:1.0-video-index0")
          .lens(
              Lens.builder()
                  .width(1600)
                  .height(1304)
                  .fx(1229.712)
                  .fy(1229.954)
                  .cx(801.183)
                  .cy(650.427)
                  .distortion(List.of(-0.3117, 0.1204, 0.00041, -0.00022, -0.0208))
                  .build());

  /** A camera with its own exposure and gain, as dialed in at a venue and written back. */
  static final Software.Visor BACK =
      Software.visor()
          .device("/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:2:1.0-video-index0")
          .exposure(Milliseconds.of(1.5))
          .gain(40);

  /** A camera whose live view on its Pitwall card is smaller and slower than Visor's own. */
  static final Software.Visor SIDE =
      Software.visor()
          .device("/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:3:1.0-video-index0")
          .stream(Percent.of(25), Hertz.of(5), 60);

  /**
   * A robot's coprocessors, each by its Spotter agent's address. The robot pushes each Visor's pack
   * itself; Spotter's debian pack, for the board's own health, comes with the board's image.
   */
  static final RobotConfig WITH_COPROCESSORS =
      ExampleRobot.COMPETITION.withCoprocessors("10.12.34.11", "10.12.34.12");
}
