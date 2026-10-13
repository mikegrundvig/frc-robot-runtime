package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.stream.Stream;
import org.wpilib.framework.RobotBase;
import org.wpilib.networktables.NetworkTableInstance;

/**
 * The robot program recording a scripted session in the simulator ({@link ReplaySessionRobot}, or
 * {@link ControlsSessionRobot} with the guide's controls), and replaying its log, each run its own
 * process. The log is what the robot logs, by name and type: the runtime's replay test reads it,
 * and so does a team's check of its dashboards' layouts.
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
    return record(dir, program -> {});
  }

  /**
   * Records the session as {@link #record(Path)} does, and hands the program to a watcher as soon
   * as it has started, so that a test can look at the process while the session runs.
   *
   * @param watcher told of the program once, as it starts; it must return at once, and watch from a
   *     thread of its own
   */
  public static Path record(Path dir, Consumer<ProcessHandle> watcher)
      throws IOException, InterruptedException {
    run(
        ReplaySessionRobot.class,
        dir,
        "record",
        Map.of("frc.simMode", "SIM", "pitwall.port", "0"),
        Map.of(),
        watcher);
    return onlyLog(dir);
  }

  /**
   * Records a session of the guide's robot with its guide's controls ({@link
   * ControlsSessionRobot}): moves that end, a gate that goes false and then true, and bindings that
   * fire. It is a second session beside {@link #record}'s, which has no controls.
   *
   * @param dir the folder it runs in, which gets its {@code logs/}
   * @return the session's log, as the robot recorded it
   */
  public static Path recordWithControls(Path dir) throws IOException, InterruptedException {
    return recordWithControls(dir, program -> {});
  }

  /**
   * Records the guide's session as {@link #recordWithControls(Path)} does, and hands the program to
   * a watcher as it starts, as {@link #record(Path, Consumer)} does.
   */
  public static Path recordWithControls(Path dir, Consumer<ProcessHandle> watcher)
      throws IOException, InterruptedException {
    run(
        ControlsSessionRobot.class,
        dir,
        "record",
        Map.of("frc.simMode", "SIM", "pitwall.port", "0"),
        Map.of(),
        watcher);
    return onlyLog(dir);
  }

  /**
   * Records a session of the guide's robot driven about the field and then shoved ({@link
   * WorldSessionRobot}): its cameras' measurements accepted and then refused, and the estimate
   * recovering. Beside every loop's record it logs what the robot's own objects said in that loop.
   *
   * @param dir the folder it runs in, which gets its {@code logs/}
   * @return the session's log, as the robot recorded it
   */
  public static Path recordTheWorld(Path dir) throws IOException, InterruptedException {
    run(
        WorldSessionRobot.class,
        dir,
        "record",
        Map.of("frc.simMode", "SIM", "pitwall.port", "0"),
        Map.of(),
        program -> {});
    return onlyLog(dir);
  }

  /**
   * Starts a session robot as WPILib starts a robot program, with NetworkTables kept to this
   * program. WPILib's robot constructor starts a NetworkTables server on port 5810, announced on
   * the network as "robot", unless NetworkTables has been told to stay local; this tells it before
   * WPILib has made the robot. A session needs no server: the Driver Station and the joysticks come
   * from the simulator's HAL, and the robot's tables are read in its own process. With one, any
   * dashboard on the computer could connect to a test's robot and change its tunables, a simulator
   * a person has open would hold the port against it, and so would a second session.
   *
   * @param robot makes the session robot, given the clock it runs on ({@link #clock})
   */
  static <T extends RobotBase> void start(Function<Optional<SessionClock>, T> robot) {
    NetworkTableInstance.getDefault().startLocal();
    RobotBase.startRobot(() -> robot.apply(clock()));
  }

  /**
   * The clock a session robot runs on, for the program this is called in, which must be a session
   * robot's as WPILib starts it. A recording runs on a clock of its own, stopped here before the
   * robot program is made, so that the session is the same on any computer ({@link SessionClock}).
   * A replay runs on no clock but its log's.
   */
  static Optional<SessionClock> clock() {
    if (RunMode.current() != RunMode.SIM) {
      return Optional.empty();
    }
    return Optional.of(SessionClock.stopped());
  }

  /** Runs the session robot in its own process, in the test's temporary directory. */
  static void run(
      Path dir, String name, Map<String, String> properties, Map<String, String> environment)
      throws IOException, InterruptedException {
    run(ReplaySessionRobot.class, dir, name, properties, environment, program -> {});
  }

  private static void run(
      Class<?> robot,
      Path dir,
      String name,
      Map<String, String> properties,
      Map<String, String> environment,
      Consumer<ProcessHandle> watcher)
      throws IOException, InterruptedException {
    List<String> command = ChildJvm.command(List.of(), properties, robot);

    Path output = dir.resolve(name + ".out");
    ProcessBuilder builder =
        new ProcessBuilder(command).directory(dir.toFile()).redirectErrorStream(true);
    builder.redirectOutput(output.toFile());
    builder.environment().remove("HALSIM_EXTENSIONS"); // replay refuses simulation extensions
    builder.environment().putAll(environment);
    Process process = builder.start();
    watcher.accept(process.toHandle());
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
