package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.stream.Stream;

/**
 * The robot program recording a scripted session in the simulator ({@link ReplaySessionRobot}), and
 * replaying its log, each run its own process. The log is what the robot logs, by name and type:
 * the runtime's replay test reads it, and so does a team's check of its dashboards' layouts.
 */
public final class SessionRecording {
  private SessionRecording() {}

  /**
   * Records the session: runs the robot program in the simulator, in a process of its own, in a
   * folder.
   *
   * @param dir the folder it runs in, which gets its {@code logs/}
   * @return the session's log, as the robot recorded it
   */
  public static Path record(Path dir) throws IOException, InterruptedException {
    run(dir, "record", Map.of("frc.simMode", "SIM", "pitwall.port", "0"), Map.of());
    return onlyLog(dir);
  }

  /** Runs the session robot in its own process, in the test's temporary directory. */
  static void run(
      Path dir, String name, Map<String, String> properties, Map<String, String> environment)
      throws IOException, InterruptedException {
    List<String> command = ChildJvm.command(List.of(), properties, ReplaySessionRobot.class);

    Path output = dir.resolve(name + ".out");
    ProcessBuilder builder =
        new ProcessBuilder(command).directory(dir.toFile()).redirectErrorStream(true);
    builder.redirectOutput(output.toFile());
    builder.environment().remove("HALSIM_EXTENSIONS"); // replay refuses simulation extensions
    builder.environment().putAll(environment);
    Process process = builder.start();
    boolean finished = process.waitFor(90, TimeUnit.SECONDS);
    if (!finished) {
      process.destroyForcibly();
    }
    assertThat(finished && process.exitValue() == 0)
        .as(name + " run finished cleanly; its output:\n" + tail(output))
        .isTrue();
  }

  private static Path onlyLog(Path dir) throws IOException {
    try (Stream<Path> logs = Files.list(dir.resolve("logs"))) {
      List<Path> found = logs.filter(p -> p.toString().endsWith(".wpilog")).toList();
      assertThat(found).as("the recorded log").hasSize(1);
      return found.get(0);
    }
  }

  private static String tail(Path output) {
    try {
      List<String> lines = Files.readAllLines(output);
      return String.join("\n", lines.subList(Math.max(0, lines.size() - 40), lines.size()));
    } catch (IOException e) {
      return "(no output: " + e + ")";
    }
  }
}
