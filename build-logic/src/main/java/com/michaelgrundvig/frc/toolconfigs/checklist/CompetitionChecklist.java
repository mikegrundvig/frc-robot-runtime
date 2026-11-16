package com.michaelgrundvig.frc.toolconfigs.checklist;

import com.michaelgrundvig.frc.toolconfigs.WriterArguments;
import com.michaelgrundvig.frc.world.robot.Camera;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.tools.RobotDescription;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * A robot's pre-competition checklist, generated from its definition, never written by hand: which
 * camera belongs in each station, by the number on its label and the end of its serial, then what
 * to check in the pit and on the field. The build writes one Markdown file per robot into {@code
 * checklists/}, committed, to print or copy into an event's notes; nothing on the robot reads it.
 *
 * <p>Every file comes out the same, byte for byte, for the same definition on any computer: the
 * build compares it with what's committed.
 */
public final class CompetitionChecklist {
  /**
   * The fewest of a serial's last characters the checklist shows; more where two of the robot's
   * cameras' serials end alike, until each reads apart.
   */
  static final int SERIAL_TAIL = 4;

  /** What every file says made it, and that it's generated. */
  static final String GENERATED =
      "Generated from `Robots.java` by the build (`./gradlew competitionChecklist`, part of every"
          + " build): don't edit it here, change the robot instead.";

  private CompetitionChecklist() {}

  /**
   * Writes the robots' checklists: {@code --catalog=<class> --out=<folder> [--robot=<name>]...}.
   * With no robot named, every robot the catalog has. The build's checklist plugin runs this on the
   * compiled program; it only builds the robots' configuration records and writes files.
   *
   * @throws IllegalArgumentException for a missing or unknown argument, or a robot the catalog
   *     doesn't have
   */
  public static void main(String... args) throws IOException, ReflectiveOperationException {
    WriterArguments arguments = WriterArguments.parse("CompetitionChecklist", args);
    write(arguments.robotsToWrite(), arguments.out());
  }

  /**
   * Writes each robot's checklist into a folder.
   *
   * @throws IllegalArgumentException if two robots' checklists would have the same name
   */
  public static void write(List<RobotDescription> robots, Path folder) throws IOException {
    Map<String, String> robotByFile = new HashMap<>();
    for (RobotDescription robot : robots) {
      String earlier = robotByFile.put(fileName(robot.name()), robot.name());
      if (earlier != null) {
        throw new IllegalArgumentException(
            "the robots " + earlier + " and " + robot.name() + " would share a checklist");
      }
    }
    Files.createDirectories(folder);
    for (Map.Entry<String, String> file : files(robots).entrySet()) {
      Files.writeString(folder.resolve(file.getKey()), file.getValue(), StandardCharsets.UTF_8);
    }
  }

  /** Each robot's checklist, by its file's name. */
  static Map<String, String> files(List<RobotDescription> robots) {
    Map<String, String> files = new LinkedHashMap<>();
    for (RobotDescription robot : robots) {
      files.put(fileName(robot.name()), checklist(robot));
    }
    return files;
  }

  /** A robot's checklist's file name: its name, kept to letters, digits, dashes and underscores. */
  static String fileName(String robot) {
    return robot.replaceAll("[^A-Za-z0-9_-]", "_") + ".md";
  }

  /** A robot's checklist, as its file has it. */
  static String checklist(RobotDescription robot) {
    StringBuilder out = new StringBuilder();
    out.append("# Pre-competition checklist: ").append(robot.name()).append("\n\n");
    out.append(GENERATED).append("\n\n");
    List<CameraConfig> cameras = robot.cameras();
    if (cameras.isEmpty()) {
      out.append("The robot has no cameras, so there's nothing about them to check.\n");
      return out.toString();
    }
    int tail = tailLength(cameras);
    whichCameraGoesWhere(out, cameras, tail);
    inThePit(out, cameras, tail);
    onTheField(out, cameras);
    return out.toString();
  }

  /** The table of stations, each with the camera that belongs there. */
  private static void whichCameraGoesWhere(
      StringBuilder out, List<CameraConfig> cameras, int tail) {
    out.append("## Which camera goes where\n\n");
    out.append("| Station | Camera (its label) | Its serial ends | Type | Read by |\n");
    out.append("|---|---|---|---|---|\n");
    for (CameraConfig camera : cameras) {
      out.append("| ")
          .append(cell(camera.name()))
          .append(" | ")
          .append(camera.camera().number())
          .append(" | ")
          .append(serialCell(camera.camera(), tail))
          .append(" | ")
          .append(cell(camera.type().name()))
          .append(" | ")
          .append(program(camera.camera()))
          .append(" |\n");
    }
    out.append('\n');
    boolean anyOnVisor =
        cameras.stream().anyMatch(camera -> camera.software() instanceof Camera.Software.Visor);
    if (anyOnVisor) {
      out.append(
          "No camera is tied to a coprocessor in the code: the robot finds the board each camera on"
              + " Visor is plugged into by its device, and the camera's card on Pitwall's Pit view"
              + " names the board its frames come from.\n\n");
    }
  }

  /** Text for a table's cell: a {@code |} in it would end the cell, so it's escaped. */
  static String cell(String text) {
    return text.replace("|", "\\|");
  }

  /**
   * How many of the serials' last characters tell the robot's cameras apart: at least {@link
   * #SERIAL_TAIL}, and more while two cameras' serials end alike, up to the longest serial.
   */
  static int tailLength(List<CameraConfig> cameras) {
    List<String> serials =
        cameras.stream().flatMap(camera -> camera.camera().serial().stream()).toList();
    int longest = serials.stream().mapToInt(String::length).max().orElse(SERIAL_TAIL);
    for (int length = SERIAL_TAIL; length < longest; length++) {
      int count = length;
      long distinct = serials.stream().map(serial -> tail(serial, count)).distinct().count();
      if (distinct == serials.size()) {
        return length;
      }
    }
    return Math.max(longest, SERIAL_TAIL);
  }

  /** What the pit reads to know a camera: the end of its serial, or why there's none. */
  private static String serialCell(Camera camera, int tail) {
    if (camera.serial().isPresent()) {
      return "`" + cell(tail(camera.serial().get(), tail)) + "`";
    }
    if (camera.device().isPresent()) {
      return "none: named by its USB port, so it goes back in the same port";
    }
    if (camera.software() instanceof Camera.Software.Visor) {
      return "none yet: its device isn't in the code";
    }
    return "kept by PhotonVision, not in the code";
  }

  /** A serial's last few characters, or all of a short one. */
  static String tail(String serial, int length) {
    if (serial.length() <= length) {
      return serial;
    }
    return serial.substring(serial.length() - length);
  }

  /** The cameras' types' names, each once, in the order the robot lists its cameras. */
  private static List<String> typesOf(List<CameraConfig> cameras) {
    return cameras.stream().map(camera -> camera.type().name()).distinct().toList();
  }

  /** The vision program that reads a camera, by name. */
  private static String program(Camera camera) {
    if (camera.software() instanceof Camera.Software.Visor) {
      return "Visor";
    }
    return "PhotonVision";
  }

  /** What to check in the pit: each station's camera, each camera reading a tag, the venue. */
  private static void inThePit(StringBuilder out, List<CameraConfig> cameras, int tail) {
    out.append("## In the pit\n\n");
    for (CameraConfig camera : cameras) {
      out.append("- [ ] ")
          .append(camera.name())
          .append(" holds camera ")
          .append(camera.camera().number())
          .append(serialSentence(camera.camera(), tail))
          .append(".\n");
    }
    for (CameraConfig camera : cameras) {
      out.append("- [ ] ").append(camera.name()).append(' ').append(readsATag(camera)).append('\n');
    }
    out.append(
        "- [ ] The event's venue is chosen on the venue card. With no survey of the field, make it"
            + " there: New venue for this event, with the event's name.\n\n");
  }

  /** The end of a camera's serial, as the pit checks it, or nothing for a camera without one. */
  private static String serialSentence(Camera camera, int tail) {
    if (camera.serial().isPresent()) {
      return ", its serial ending `" + tail(camera.serial().get(), tail) + "`";
    }
    if (camera.device().isPresent()) {
      return ", plugged into the USB port its code names";
    }
    return "";
  }

  /** A camera reading a tag, as its card shows it. */
  private static String readsATag(CameraConfig camera) {
    if (camera.software() instanceof Camera.Software.Visor) {
      return "reads a tag on its Pitwall card, with no dark, clipping or incomplete-frame warning.";
    }
    return "reads a tag on PhotonVision's own page, which keeps its settings.";
  }

  /**
   * What to check on the event's field: each camera's warnings in its light, and its gain set once
   * and kept in the venue. Pitwall changes nothing while the field system is connected, so it's
   * done in the field's calibration time, not before a match.
   */
  private static void onTheField(StringBuilder out, List<CameraConfig> cameras) {
    List<CameraConfig> onVisor =
        cameras.stream()
            .filter(camera -> camera.software() instanceof Camera.Software.Visor)
            .toList();
    out.append("## On the field\n\n");
    if (onVisor.isEmpty()) {
      out.append(
          "No camera is on Visor: PhotonVision's own page sets each camera for the field's"
              + " light.\n");
      return;
    }
    out.append(
        "On the event's field, with the robot disabled and no field system connected (the field's"
            + " calibration time, not a match), from each camera's card on Pitwall's Pit view:\n\n");
    for (CameraConfig camera : onVisor) {
      out.append("- [ ] Read ").append(camera.name()).append("'s dark and clipping warnings.\n");
    }
    for (String type : typesOf(onVisor)) {
      out.append("- [ ] Adjust the ")
          .append(type)
          .append("s' gain once, from one of their cards, until none warns dark or clipping, and")
          .append(" keep it: Keep in this venue: every ")
          .append(type)
          .append(".\n");
    }
    out.append(
        "- [ ] Download the venue from the venue card, and commit it to"
            + " `src/main/deploy/venues/`.\n");
  }
}
