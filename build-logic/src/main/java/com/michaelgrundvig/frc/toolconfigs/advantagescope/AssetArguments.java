package com.michaelgrundvig.frc.toolconfigs.advantagescope;

import com.michaelgrundvig.frc.world.tools.RobotCatalog;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * What the AdvantageScope writer is asked to do on its command line: {@code --catalog=<class>
 * --out=<folder> [--robot=<name>]...}.
 *
 * @param catalogClass the name of the {@link RobotCatalog} class that has the team's robots
 * @param out the folder to write the robots' folders into
 * @param robots the robots to write, by name. Empty means every robot the catalog has.
 */
record AssetArguments(String catalogClass, Path out, List<String> robots) {
  static final String USAGE =
      "AdvantageScopeAssets --catalog=<a RobotCatalog's class> --out=<folder> [--robot=<name>]...";

  private static final String CATALOG = "--catalog=";
  private static final String OUT = "--out=";
  private static final String ROBOT = "--robot=";

  /**
   * The arguments of a command line. Given twice, a catalog or a folder is the last one given.
   *
   * @throws IllegalArgumentException for an argument that is none of the three, or if the catalog
   *     or the folder is not given
   */
  static AssetArguments parse(String... args) {
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
        throw new IllegalArgumentException("unknown argument " + arg + "; " + USAGE);
      }
    }
    if (catalogClass.isEmpty() || out.isEmpty()) {
      throw new IllegalArgumentException(USAGE);
    }
    return new AssetArguments(catalogClass.get(), out.get(), robots);
  }
}
