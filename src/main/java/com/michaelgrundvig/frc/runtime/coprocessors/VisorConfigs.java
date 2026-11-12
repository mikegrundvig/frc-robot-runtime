package com.michaelgrundvig.frc.runtime.coprocessors;

import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Microseconds;
import static org.wpilib.units.Units.Value;

import com.michaelgrundvig.frc.visor.VisorConfigFile;
import com.michaelgrundvig.frc.visor.VisorPack;
import com.michaelgrundvig.frc.visor.VisorReceiver;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.TagFamily;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfig.Software.Exposure;
import com.michaelgrundvig.frc.world.robot.CameraConfig.Software.Gain;
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
 * plugged into; or, for a camera with no serial of its own, by its port's {@code by-path/} name.
 *
 * <p>Each camera's exposure and gain are written as its definition gives them, or as they're tuned
 * live ({@link VisorTuning}): Visor has no defaults of its own, so a camera's own exposure or gain
 * is written as nothing, and Visor puts back the camera's. Its live view is written as its
 * definition gives it, at no more frames a second than the camera's own, or left out for Visor's
 * own.
 *
 * <p>Each configuration names the port the robot receives frames on, {@link
 * VisorReceiver#DEFAULT_PORT}, so Visor's own default never decides where frames go, and the
 * field's tag size ({@link FieldModel#tagSizeMeters}), so every camera's poses are solved at the
 * size of the tags it sees. {@link VisorConfigFile} refuses any setting Visor would, and the robot
 * writes every camera's entry as it starts ({@link #requireRunnable}), so a robot definition Visor
 * can't run fails then, in Visor's words, before any board is configured.
 */
public final class VisorConfigs {
  /** The one tag family Visor detects: its detector, NAPA, is built for 36h11 alone. */
  static final TagFamily VISOR_DETECTS = TagFamily.TAG_36H11;

  /**
   * The team a configuration is written for as the robot starts, only to judge its cameras: the
   * team addresses Visor's frames, and every team's file is judged alike.
   */
  private static final int ANY_TEAM = 1;

  /**
   * The device a camera that names none yet is judged at as the robot starts: Visor doesn't judge a
   * device's name, and a camera with none is never written for a board.
   */
  private static final String NO_DEVICE_YET = "/dev/v4l/by-id/none";

  private VisorConfigs() {}

  /**
   * One board's configuration: the robot's Visor cameras plugged into it, with its hash as the
   * board's report gives it ({@link VisorPack#configHash}).
   *
   * @param toml the configuration's text
   * @param hash its SHA-256
   * @param cameras the index of each of the robot's cameras in it, in the robot's order: none for a
   *     board that has none of them, whose configuration then opens no camera
   */
  public record ForBoard(String toml, String hash, List<Integer> cameras) {
    public ForBoard {
      cameras = List.copyOf(cameras);
    }

    /** Whether it has none of the robot's cameras. */
    public boolean empty() {
      return cameras.isEmpty();
    }
  }

  /**
   * One board's configuration: the robot's Visor cameras plugged into it. A board with none of them
   * gets a configuration with no camera.
   *
   * @param cameras the robot's cameras, in its definition's order
   * @param team the team number, which finds the robot controller, 10.TE.AM.2
   * @param tagSizeMeters the side of the field's tags' black square ({@link
   *     FieldModel#tagSizeMeters})
   * @param devices the {@code /dev/v4l/by-id/} and {@code by-path/} names the board reports
   */
  public static ForBoard forBoard(
      List<CameraConfig> cameras, int team, double tagSizeMeters, Collection<String> devices) {
    VisorConfigFile file = file(team, tagSizeMeters);
    List<Integer> on = new ArrayList<>();
    for (int index = 0; index < cameras.size(); index++) {
      CameraConfig camera = cameras.get(index);
      Optional<CameraConfig.Software.Visor> visor = visorOn(camera, devices);
      if (visor.isEmpty()) {
        continue;
      }
      on.add(index);
      cameraEntry(file, index, camera, visor.get(), visor.get().device().orElseThrow());
    }
    String toml = file.toToml();
    return new ForBoard(toml, VisorPack.configHash(toml), on);
  }

  /** A configuration's file for this team, with its port and the field's tag size. */
  private static VisorConfigFile file(int team, double tagSizeMeters) {
    return VisorConfigFile.forTeam(team)
        .port(VisorReceiver.DEFAULT_PORT)
        .tagSizeMeters(tagSizeMeters);
  }

  /**
   * Refuses, as the robot starts, a robot whose cameras on Visor Visor couldn't run on this field:
   * a field whose tags are of a family Visor doesn't detect, so a camera on Visor would see none of
   * them, or a camera whose settings Visor refuses, in Visor's words. Each camera on Visor is
   * written into a configuration, as a board's would be, whether or not it names its device yet.
   *
   * @throws IllegalArgumentException naming each camera and why
   */
  public static void requireRunnable(List<CameraConfig> cameras, FieldModel field) {
    List<String> problems = new ArrayList<>();
    VisorConfigFile file = file(ANY_TEAM, field.tagSizeMeters());
    for (int index = 0; index < cameras.size(); index++) {
      CameraConfig camera = cameras.get(index);
      if (!(camera.software() instanceof CameraConfig.Software.Visor visor)) {
        continue;
      }
      if (field.tagFamily() != VISOR_DETECTS) {
        problems.add(
            camera.name()
                + " is on Visor, which detects "
                + VISOR_DETECTS
                + " tags only, and the field's tags are "
                + field.tagFamily()
                + ": it would see none of them. Put it on PhotonVision, which detects both");
        continue;
      }
      try {
        cameraEntry(file, index, camera, visor, visor.device().orElse(NO_DEVICE_YET));
      } catch (IllegalArgumentException refused) {
        problems.add(camera.name() + ": Visor refuses it: " + refused.getMessage());
      }
    }
    if (!problems.isEmpty()) {
      throw new IllegalArgumentException(
          "The robot's cameras on Visor can't run on "
              + field.name()
              + ": "
              + String.join("; ", problems));
    }
  }

  /** A camera's Visor software, when it's on Visor and its device is among those given. */
  private static Optional<CameraConfig.Software.Visor> visorOn(
      CameraConfig camera, Collection<String> devices) {
    if (!(camera.software() instanceof CameraConfig.Software.Visor visor)) {
      return Optional.empty();
    }
    Optional<String> device = visor.device();
    if (device.isEmpty() || !devices.contains(deviceName(device.get()))) {
      return Optional.empty();
    }
    return Optional.of(visor);
  }

  /**
   * Writes one camera's entry, at this device: its sensor, exposure, gain, live view, controls and
   * lens.
   */
  private static void cameraEntry(
      VisorConfigFile file,
      int index,
      CameraConfig camera,
      CameraConfig.Software.Visor visor,
      String device) {
    CameraConfig.ImageSensor sensor = camera.sensor();
    int fps = (int) Math.round(sensor.frameRate().in(Hertz));
    VisorConfigFile.Camera entry =
        file.camera(camera.name(), index, device, sensor.widthPixels(), sensor.heightPixels())
            .fps(fps);
    visor
        .timeOffset()
        .ifPresent(offset -> entry.timeOffsetMicros(Math.round(offset.in(Microseconds))));
    // A fixed exposure turns the camera's automatic exposure off; the camera's own is left out.
    if (visor.exposure() instanceof Exposure.Fixed fixed) {
      entry.exposureMicros(Math.round(fixed.time().in(Microseconds)));
    }
    if (visor.gain() instanceof Gain.Level level) {
      entry.gain(level.value());
    }
    // A live view the definition doesn't give is left out, so Visor's own applies.
    if (visor.stream().isPresent()) {
      CameraConfig.Software.Stream stream = visor.stream().get();
      int streamFps = (int) Math.max(1, Math.min(fps, Math.round(stream.frameRate().in(Hertz))));
      entry.stream(stream.scale().in(Value), streamFps, stream.quality());
    }
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
        continue;
      }
      if (visorOn(camera, devices).isEmpty()) {
        unplaced.add(camera.name() + " (" + deviceName(visor.device().get()) + ")");
      }
    }
    return unplaced;
  }

  /** A device's name under {@code /dev/v4l/by-id/} or {@code by-path/}: its path's last part. */
  static String deviceName(String device) {
    Path name = Path.of(device).getFileName();
    return name == null ? device : name.toString();
  }
}
