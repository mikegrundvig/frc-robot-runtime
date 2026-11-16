package com.michaelgrundvig.frc.toolconfigs;

import com.michaelgrundvig.frc.world.tools.RobotCatalog;
import com.michaelgrundvig.frc.world.tools.RobotDescription;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * What a tool's writer is asked to do on its command line, as each plugin runs it: {@code
 * --catalog=<class> --out=<folder> [--robot=<name>]...}.
 *
 * @param catalogClass the name of the {@link RobotCatalog} class that has the team's robots
 * @param out the folder to write the robots' files into
 * @param robots the robots to write, by name. Empty means every robot the catalog has.
 */
public record WriterArguments(String catalogClass, Path out, List<String> robots) {
  /** What a writer's command line takes, after the writer's name. */
  static final String USAGE =
      " --catalog=<a RobotCatalog's class> --out=<folder> [--robot=<name>]...";

  private static final String CATALOG = "--catalog=";
  private static final String OUT = "--out=";
  private static final String ROBOT = "--robot=";

  /**
   * The arguments of a command line. Given twice, a catalog or a folder is the last one given.
   *
   * @param writer the writer's name, for a refusal to say how it's run
   * @throws IllegalArgumentException for an argument that is none of the three, or if the catalog
   *     or the folder is not given
   */
  public static WriterArguments parse(String writer, String... args) {
    Optional<String> catalogClass = Optional.empty();
    Optional<Path> out = Optional.empty();
    List<String> robots = new ArrayList<>();
    for (String arg : args) {
      if (arg.startsWith(CATALOG)) {
        catalogClass = Optional.of(arg.substring(CATALOG.length()));
      } else if (arg.startsWith(OUT)) {
        out = Optional.of(Path.of(arg.substring(OUT.length())));
      } else if (arg.startsWith(ROBOT)) {
        robots.add(arg.substring(ROBOT.length()));
      } else {
        throw new IllegalArgumentException("unknown argument " + arg + "; " + writer + USAGE);
      }
    }
    if (catalogClass.isEmpty() || out.isEmpty()) {
      throw new IllegalArgumentException(writer + USAGE);
    }
    return new WriterArguments(catalogClass.get(), out.get(), robots);
  }

  /**
   * The robots to write: those named, or every robot the catalog has when none is named. The
   * catalog is made by its name, on the program's classpath, which the plugin runs the writer on.
   *
   * @throws ReflectiveOperationException if the catalog can't be made
   * @throws IllegalArgumentException for a robot the catalog doesn't have
   */
  public List<RobotDescription> robotsToWrite() throws ReflectiveOperationException {
    RobotCatalog catalog =
        Class.forName(catalogClass)
            .asSubclass(RobotCatalog.class)
            .getDeclaredConstructor()
            .newInstance();
    if (robots.isEmpty()) {
      return catalog.all();
    }
    return robots.stream().map(catalog::named).toList();
  }
}
