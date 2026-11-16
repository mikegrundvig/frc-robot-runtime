package com.michaelgrundvig.frc.runtime;

import com.michaelgrundvig.frc.world.robot.Camera;
import com.michaelgrundvig.frc.world.robot.CameraBuilder;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraType;
import com.michaelgrundvig.frc.world.robot.Lens;

/**
 * The tests' ways of changing a camera in its station: moving it to Visor, giving it a device or a
 * lens, or another type. Each keeps the station and the camera's number, so the robot still has the
 * same cameras where they were.
 */
public final class TestCameras {
  private TestCameras() {}

  /** The station, holding its camera moved to Visor, with nothing said of its device or lens. */
  public static CameraConfig onVisor(CameraConfig placed) {
    return placed.station().holds(Camera.visor(placed.camera().number(), placed.type()));
  }

  /** The station, holding its camera on Visor at this device. */
  public static CameraConfig onVisor(CameraConfig placed, String device) {
    Camera camera = Camera.visor(placed.camera().number(), placed.type()).device(device);
    return placed.station().holds(camera);
  }

  /** The station, holding its camera on Visor at this device, with this lens. */
  public static CameraConfig onVisor(CameraConfig placed, String device, Lens lens) {
    Camera camera = Camera.visor(placed.camera().number(), placed.type()).device(device).lens(lens);
    return placed.station().holds(camera);
  }

  /**
   * A camera type, named apart from the robot's others: a kind set up otherwise needs its own name,
   * since a venue keeps settings by it.
   */
  public static CameraType named(CameraType type, String name) {
    return com.michaelgrundvig.frc.world.robot.CameraTypeBuilder.builder(type).name(name).build();
  }

  /** The station, holding its camera as it is but of this type. */
  public static CameraConfig retyped(CameraConfig placed, CameraType type) {
    return placed.station().holds(CameraBuilder.builder(placed.camera()).type(type).build());
  }

  /** The station, holding its camera as it is but read by this software. */
  public static CameraConfig on(CameraConfig placed, Camera.Software software) {
    return placed
        .station()
        .holds(CameraBuilder.builder(placed.camera()).software(software).build());
  }

  /** The station, holding this camera in place of its own. */
  public static CameraConfig holding(CameraConfig placed, Camera camera) {
    return placed.station().holds(camera);
  }
}
