package com.michaelgrundvig.frc.runtime.examples;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Percent;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotConfig;
import com.michaelgrundvig.frc.world.robot.Camera;
import com.michaelgrundvig.frc.world.robot.CameraStation;
import com.michaelgrundvig.frc.world.robot.CameraType;
import com.michaelgrundvig.frc.world.robot.ImageSensor;
import com.michaelgrundvig.frc.world.robot.Lens;
import java.util.List;

/**
 * Cameras on Visor as robot code says them (docs/visor.md): a kind of camera, each camera by the
 * number on its label with its device and its lens, and the station each is in.
 */
final class VisorCameraExample {
  private VisorCameraExample() {}

  /**
   * The kind of camera, at the size and rate it runs at. It says no exposure or gain, so each
   * camera of the kind starts at the defaults: a fixed 1 ms, the camera's automatic exposure off,
   * at the camera's own gain.
   */
  static final CameraType THRIFTYCAM =
      CameraType.of("ThriftyCam", ImageSensor.of(1600, 1304, Hertz.of(60)));

  /** Camera 1, by the number on its label: its device and its lens, said together. */
  static final Camera CAMERA_1 =
      Camera.visor(1, THRIFTYCAM)
          .device("/dev/v4l/by-id/usb-Maker_Camera_A1B2C3-video-index0")
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

  /** The place camera 1 goes. */
  static final CameraStation FRONT_LEFT =
      CameraStation.named("front-left")
          .at(Inches.of(11.75), Inches.of(11.75), Inches.of(21))
          .facing(Degrees.of(10), Degrees.of(45));

  /** The kind with a starting exposure and gain of its own, once a real one is dialed in. */
  static final CameraType DIALED_IN = THRIFTYCAM.exposure(Milliseconds.of(1.5)).gain(40);

  /** The kind with a live view on its Pitwall card smaller and slower than Visor's own. */
  static final CameraType SMALL_VIEW = THRIFTYCAM.stream(Percent.of(25), Hertz.of(5), 60);

  /**
   * A robot with camera 1 in its front-left station, and its coprocessors, each by its Spotter
   * agent's address. The robot pushes each Visor's pack itself; Spotter's debian pack, for the
   * board's own health, comes with the board's image.
   */
  static final RobotConfig WITH_COPROCESSORS =
      ExampleRobot.COMPETITION
          .with(FRONT_LEFT.holds(CAMERA_1))
          .withCoprocessors("10.12.34.11", "10.12.34.12");
}
