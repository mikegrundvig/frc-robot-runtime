package com.michaelgrundvig.frc.runtime.coprocessors;

import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Microseconds;

import com.michaelgrundvig.frc.visor.VisorConfigFile;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;

/**
 * Visor's configuration for a coprocessor, from the robot's definition and the cameras the board
 * reports plugged in: each of the robot's cameras on Visor ({@link CameraConfig.Software.Visor})
 * whose device is among them, at its place in the robot's list of cameras, which is the index its
 * frames carry and the robot reads it by. A camera is found by its device's {@code /dev/v4l/by-id/}
 * name, which is the camera's own (its USB serial), so it's configured on whichever board it's
 * plugged into.
 */
public final class VisorConfigs {
  private VisorConfigs() {}

  /**
   * One board's configuration: the robot's Visor cameras plugged into it. Empty when none is, so
   * the board needs no Visor.
   *
   * @param cameras the robot's cameras, in its definition's order
   * @param team the team number, which finds the robot controller, 10.TE.AM.2
   * @param devices the {@code /dev/v4l/by-id/} names the board reports
   */
  public static Optional<String> forBoard(
      List<CameraConfig> cameras, int team, Collection<String> devices) {
    VisorConfigFile file = VisorConfigFile.forTeam(team);
    boolean any = false;
    for (int index = 0; index < cameras.size(); index++) {
      CameraConfig camera = cameras.get(index);
      if (!(camera.software() instanceof CameraConfig.Software.Visor visor)) {
        continue;
      }
      Optional<String> device = visor.device();
      if (device.isEmpty() || !devices.contains(deviceName(device.get()))) {
        continue;
      }
      any = true;
      CameraConfig.Sensor sensor = camera.sensor();
      VisorConfigFile.Camera entry =
          file.camera(
                  camera.name(), index, device.get(), sensor.widthPixels(), sensor.heightPixels())
              .fps((int) Math.round(sensor.frameRate().in(Hertz)));
      visor
          .timeOffset()
          .ifPresent(offset -> entry.timeOffsetMicros(Math.round(offset.in(Microseconds))));
      for (CameraConfig.Software.Control control : visor.controls()) {
        entry.control(control.name(), control.value());
      }
      visor
          .lens()
          .ifPresent(
              lens ->
                  entry.lens(
                      lens.width(),
                      lens.height(),
                      lens.fx(),
                      lens.fy(),
                      lens.cx(),
                      lens.cy(),
                      lens.distortion().stream().mapToDouble(Double::doubleValue).toArray()));
    }
    return any ? Optional.of(file.toToml()) : Optional.empty();
  }

  /**
   * The robot's cameras on Visor that no board has: none of the devices given is theirs, or they
   * name no device. Each as its name and why.
   *
   * @param devices every board's devices together
   */
  public static List<String> unplaced(List<CameraConfig> cameras, Collection<String> devices) {
    List<String> unplaced = new ArrayList<>();
    for (CameraConfig camera : cameras) {
      if (!(camera.software() instanceof CameraConfig.Software.Visor visor)) {
        continue;
      }
      if (visor.device().isEmpty()) {
        unplaced.add(camera.name() + " (it names no device)");
      } else if (!devices.contains(deviceName(visor.device().get()))) {
        unplaced.add(camera.name() + " (" + deviceName(visor.device().get()) + ")");
      }
    }
    return unplaced;
  }

  /** A device's name under {@code /dev/v4l/by-id/}: its path's last part. */
  static String deviceName(String device) {
    Path name = Path.of(device).getFileName();
    return name == null ? device : name.toString();
  }
}
