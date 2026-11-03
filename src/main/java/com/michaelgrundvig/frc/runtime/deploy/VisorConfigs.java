package com.michaelgrundvig.frc.runtime.deploy;

import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Microseconds;

import com.michaelgrundvig.frc.visor.VisorConfigFile;
import com.michaelgrundvig.frc.world.bridges.RobotCatalog;
import com.michaelgrundvig.frc.world.bridges.RobotDescription;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;

/**
 * Visor's configuration for each coprocessor of one robot, from its definition: every camera on
 * Visor ({@link CameraConfig.Software.Visor}) in its board's file, at its place in the robot's list
 * of cameras, which is the index its frames carry and the robot reads it by.
 *
 * <p>The build's {@code frc.visor-deploy} plugin runs {@link #main} on the compiled program, as the
 * AdvantageScope plugin runs its writer, and deploys each file to its board.
 */
public final class VisorConfigs {
  private VisorConfigs() {}

  /**
   * Each board's file, by the board's name.
   *
   * @param cameras the robot's cameras, in its definition's order
   * @param team the team number, which finds the robot controller, 10.TE.AM.2
   * @param boards the boards the build can deploy to
   * @throws IllegalArgumentException for a camera on Visor that doesn't say where it is, or is on a
   *     board the build doesn't name
   */
  public static Map<String, String> of(List<CameraConfig> cameras, int team, Set<String> boards) {
    Map<String, VisorConfigFile> files = new TreeMap<>();
    List<String> problems = new ArrayList<>();
    for (int index = 0; index < cameras.size(); index++) {
      CameraConfig camera = cameras.get(index);
      if (!(camera.software() instanceof CameraConfig.Software.Visor visor)) {
        continue;
      }
      Optional<String> board = visor.coprocessor();
      Optional<String> device = visor.device();
      if (board.isEmpty() || device.isEmpty()) {
        problems.add(
            camera.name()
                + " runs Visor but doesn't say "
                + (board.isEmpty() ? "which coprocessor it's on" : "its device")
                + ": Software.visor().coprocessor(...).device(...)");
        continue;
      }
      if (!boards.contains(board.get())) {
        problems.add(
            camera.name()
                + " is on the coprocessor "
                + board.get()
                + ", which the build's visor block doesn't name"
                + (boards.isEmpty()
                    ? ""
                    : " (it names " + String.join(", ", sorted(boards)) + ")"));
        continue;
      }
      CameraConfig.Sensor sensor = camera.sensor();
      VisorConfigFile.Camera entry =
          files
              .computeIfAbsent(board.get(), name -> VisorConfigFile.forTeam(team))
              .camera(
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
    if (!problems.isEmpty()) {
      throw new IllegalArgumentException(String.join("\n", problems));
    }
    Map<String, String> texts = new TreeMap<>();
    files.forEach((board, file) -> texts.put(board, file.toToml()));
    return texts;
  }

  /**
   * The robot to deploy Visor for: the one named, or else the only robot with a camera on Visor on
   * a coprocessor; none when no robot has one.
   *
   * @throws IllegalArgumentException when several robots do and none is named
   */
  public static Optional<RobotDescription> robot(RobotCatalog catalog, Optional<String> named) {
    if (named.isPresent()) {
      return Optional.of(catalog.named(named.get()));
    }
    List<RobotDescription> onVisor =
        catalog.all().stream().filter(VisorConfigs::hasVisorOnACoprocessor).toList();
    if (onVisor.size() > 1) {
      throw new IllegalArgumentException(
          "Several robots run Visor ("
              + String.join(", ", onVisor.stream().map(RobotDescription::name).toList())
              + "): name the one to deploy to, -Probot=NAME");
    }
    return onVisor.stream().findFirst();
  }

  private static boolean hasVisorOnACoprocessor(RobotDescription robot) {
    return robot.cameras().stream()
        .anyMatch(
            camera ->
                camera.software() instanceof CameraConfig.Software.Visor visor
                    && visor.coprocessor().isPresent());
  }

  /**
   * Writes each board's file as {@code <out>/<board>/visor.toml}, after emptying {@code <out>}, so
   * a board with no camera any more has no file to deploy.
   *
   * <pre>{@code
   * VisorConfigs --catalog=<a RobotCatalog's class> --team=<number> --out=<folder>
   *     [--board=<name>]... [--robot=<name>]
   * }</pre>
   */
  public static void main(String... args) throws IOException, ReflectiveOperationException {
    Optional<String> catalogClass = Optional.empty();
    Optional<String> named = Optional.empty();
    Optional<Path> out = Optional.empty();
    int team = 0;
    Set<String> boards = new TreeSet<>();
    for (String arg : args) {
      String value = arg.substring(arg.indexOf('=') + 1);
      if (arg.startsWith("--catalog=")) {
        catalogClass = Optional.of(value);
      } else if (arg.startsWith("--robot=")) {
        named = value.isBlank() ? Optional.empty() : Optional.of(value);
      } else if (arg.startsWith("--out=")) {
        out = Optional.of(Path.of(value));
      } else if (arg.startsWith("--team=")) {
        team = Integer.parseInt(value);
      } else if (arg.startsWith("--board=")) {
        boards.add(value);
      } else {
        throw new IllegalArgumentException("unknown argument " + arg + "; " + USAGE);
      }
    }
    if (catalogClass.isEmpty() || out.isEmpty() || team <= 0) {
      throw new IllegalArgumentException(USAGE);
    }
    RobotCatalog catalog =
        Class.forName(catalogClass.get())
            .asSubclass(RobotCatalog.class)
            .getDeclaredConstructor()
            .newInstance();
    write(robot(catalog, named), team, boards, out.get());
  }

  static final String USAGE =
      "VisorConfigs --catalog=<a RobotCatalog's class> --team=<number> --out=<folder>"
          + " [--board=<name>]... [--robot=<name>]";

  static void write(Optional<RobotDescription> robot, int team, Set<String> boards, Path out)
      throws IOException {
    Map<String, String> files =
        robot.isPresent() ? of(robot.get().cameras(), team, boards) : Map.of();
    if (Files.isDirectory(out)) {
      try (var old = Files.walk(out)) {
        for (Path path : old.sorted((a, b) -> b.getNameCount() - a.getNameCount()).toList()) {
          if (!path.equals(out)) {
            Files.delete(path);
          }
        }
      }
    }
    Files.createDirectories(out);
    for (Map.Entry<String, String> file : files.entrySet()) {
      Path folder = out.resolve(file.getKey());
      Files.createDirectories(folder);
      Files.writeString(folder.resolve("visor.toml"), file.getValue());
    }
    String robotName = robot.map(RobotDescription::name).orElse("");
    for (String board : boards) {
      System.out.println(
          files.containsKey(board)
              ? "Visor on " + board + ": " + robotName + "'s cameras"
              : "Visor on "
                  + board
                  + ": no camera of "
                  + (robotName.isEmpty() ? "any robot" : robotName)
                  + " is on it, so nothing to deploy");
    }
  }

  private static List<String> sorted(Set<String> names) {
    return new TreeSet<>(names).stream().toList();
  }
}
