package com.michaelgrundvig.frc.runtime.coprocessors;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotConfig;
import com.michaelgrundvig.frc.runtime.RobotConfigBuilder;
import com.michaelgrundvig.frc.spotter.manager.BoardDefinition;
import com.michaelgrundvig.frc.visor.VisorVersion;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfig.Software;
import com.michaelgrundvig.frc.world.robot.CameraConfigBuilder;
import java.io.IOException;
import java.net.ServerSocket;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.attribute.PosixFilePermissions;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.util.AlertDataJNI;

/**
 * A robot's coprocessors as the robot program starts: its configuration names two boards. Spotter's
 * real agent runs on the first with nothing of Visor's, and the robot pushes it Visor's pack (from
 * the Visor release this runtime's client belongs to), installs its Visor release, and configures
 * it for the robot's camera plugged into it. The board is this computer: the agent runs from its
 * published jar with its files under a folder of the test's, and Visor's program is a script that
 * answers {@code version} and {@code check} as Visor does. The second board never answers, and the
 * robot's alerts say so.
 */
class CoprocessorsEndToEndTest {
  private static final String CAMERA = "usb-Thrifty_Cam_0001-video-index0";

  @TempDir Path work;

  private @Nullable Process agent;
  private volatile boolean stopping;
  private @Nullable Thread supervisor;
  private @Nullable Coprocessors coprocessors;

  @AfterEach
  void stop() throws InterruptedException {
    if (coprocessors != null) {
      coprocessors.close();
    }
    stopping = true;
    if (supervisor != null) {
      supervisor.interrupt();
      supervisor.join(5_000);
    }
    if (agent != null) {
      agent.destroy();
      agent.waitFor();
    }
  }

  @Test
  void aBoardWithNothingGetsTheRobotsVisorAndItsCamerasConfiguration() throws Exception {
    // The board: its cameras, and where Visor goes.
    Path visor = Files.createDirectories(work.resolve("opt-visor"));
    Path devices = Files.createDirectories(work.resolve("by-id"));
    Files.createFile(devices.resolve(CAMERA));
    Files.createFile(devices.resolve("usb-Thrifty_Cam_0001-video-index1"));
    Path restarts = work.resolve("restarts");
    Path restart =
        executable(work.resolve("restart"), "#!/bin/sh\necho restarted >> " + restarts + "\n");
    int port = freePort();
    String dead = "127.0.0.1:" + freePort(); // a second board, where nothing answers
    startAgent(port, visor, devices, restart);

    // The robot's deploy folder: Visor's pack among the team's packs, and its release.
    Path deploy = Files.createDirectories(work.resolve("deploy"));
    unpack(Path.of(System.getProperty("frc.visorRelease")), work.resolve("released"));
    Path packs = Files.createDirectories(deploy.resolve("spotter-packs"));
    copyFolder(work.resolve("released/spotter/visor"), packs.resolve("visor"));
    Path releases = Files.createDirectories(deploy.resolve("visor"));
    release(releases.resolve(Coprocessors.releaseName(VisorVersion.VERSION)));

    // The guide's robot with its first camera on Visor, plugged into the board.
    CameraConfig first = ExampleRobot.COMPETITION.definition().cameras().get(0);
    CameraConfig onVisor =
        CameraConfigBuilder.builder(first)
            .software(Software.visor().device("/dev/v4l/by-id/" + CAMERA).build())
            .build();
    RobotConfig robot =
        RobotConfigBuilder.builder(ExampleRobot.COMPETITION.with(onVisor))
            .coprocessors(
                List.of(
                    new BoardDefinition("127.0.0.1:" + port, List.of("visor")),
                    new BoardDefinition(dead, List.of("visor"))))
            .build();
    String wanted =
        VisorConfigs.forBoard(robot.definition().cameras(), 1234, List.of(CAMERA)).orElseThrow();

    Coprocessors robotsBoards =
        Coprocessors.on(
            robot.coprocessors(),
            robot.definition().cameras(),
            deploy,
            1234,
            () -> false,
            () -> false,
            System::nanoTime);
    coprocessors = robotsBoards;
    Path program = visor.resolve("visor");
    Path config = visor.resolve("visor.toml");
    loopUntil(
        robotsBoards,
        () -> Files.exists(config) && read(config).equals(wanted),
        "the board configured");
    assertThat(read(program)).isEqualTo(program(VisorVersion.VERSION));
    // Readable and runnable by Visor's own user, though the agent writes them under umask 0077.
    assertThat(PosixFilePermissions.toString(Files.getPosixFilePermissions(config)))
        .isEqualTo("rw-r--r--");
    assertThat(PosixFilePermissions.toString(Files.getPosixFilePermissions(program)))
        .isEqualTo("rwxr-xr-x");
    assertThat(restarts).content().isEqualTo("restarted\nrestarted\n");
    // The board that never answers is the robot's alert, though the other was seen to.
    assertThat(coprocessorsAlerts()).contains("HIGH " + dead + "/missing");

    // Brought up to date, it's left alone.
    long until = System.nanoTime() + 5_000_000_000L;
    while (System.nanoTime() < until) {
      robotsBoards.periodic();
      Thread.sleep(20);
    }
    assertThat(restarts).content().as("nothing more").isEqualTo("restarted\nrestarted\n");
  }

  /**
   * A Systemcore with no team number (0): Visor's configuration would send frames to 10.0.0.2, so
   * nothing of Visor's is done, and an alert says how to set it. The board isn't there: nothing
   * waits on it.
   */
  @Test
  void withNoTeamNumberVisorIsLeftAloneAndTheRobotSaysWhy() throws Exception {
    CameraConfig first = ExampleRobot.COMPETITION.definition().cameras().get(0);
    CameraConfig onVisor =
        CameraConfigBuilder.builder(first)
            .software(Software.visor().device("/dev/v4l/by-id/" + CAMERA).build())
            .build();
    RobotConfig robot = ExampleRobot.COMPETITION.with(onVisor);
    Path deploy = Files.createDirectories(work.resolve("deploy"));
    List<BoardDefinition> boards =
        List.of(new BoardDefinition("127.0.0.1:" + freePort(), List.of("visor")));
    for (int team : new int[] {0, 1234}) {
      try (Coprocessors robotsBoards =
          Coprocessors.on(
              boards,
              robot.definition().cameras(),
              deploy,
              team,
              () -> false,
              () -> false,
              System::nanoTime)) {
        robotsBoards.periodic();
        assertThat(alerts("Visor"))
            .as("team " + team)
            .isEqualTo(team == 0 ? List.of("HIGH noTeam") : List.of());
      }
    }
  }

  /** The robot's Coprocessors alerts a dashboard shows now: level and key. */
  private static List<String> coprocessorsAlerts() {
    return alerts("Coprocessors");
  }

  /** The alerts of one group a dashboard shows now: level and key. */
  private static List<String> alerts(String group) {
    return Arrays.stream(AlertDataJNI.getAlerts())
        .filter(alert -> alert.group.equals(group) && alert.activeStartTime != 0)
        .map(alert -> (alert.level == AlertDataJNI.LEVEL_HIGH ? "HIGH " : "MEDIUM ") + alert.id)
        .toList();
  }

  /** Visor's program, as the pack sees it: its version, and a check that passes. */
  private static String program(String version) {
    return "#!/bin/sh\ncase \"$1\" in\n  version) echo \"visor "
        + version
        + " (version ID 1)\" ;;\n  check) echo \"$3: checked\" ;;\nesac\n";
  }

  /** This robot's release as the release workflow makes one: the program, as {@code ./visor}. */
  private void release(Path archive) throws IOException, InterruptedException {
    Path folder = Files.createDirectories(work.resolve("release"));
    executable(folder.resolve("visor"), program(VisorVersion.VERSION));
    run("tar", "-czf", archive.toString(), "-C", folder.toString(), ".");
  }

  private void unpack(Path archive, Path into) throws IOException, InterruptedException {
    Files.createDirectories(into);
    run("tar", "-xzf", archive.toString(), "-C", into.toString());
  }

  private static void copyFolder(Path from, Path to) throws IOException, InterruptedException {
    run("cp", "-r", from.toString(), to.toString());
  }

  private static void run(String... command) throws IOException, InterruptedException {
    Process process = new ProcessBuilder(command).inheritIO().start();
    assertThat(process.waitFor()).as(String.join(" ", command)).isZero();
  }

  /**
   * The agent, as systemd runs it on a board: started again whenever it exits, as it does after a
   * push to read the packs pushed.
   */
  private void startAgent(int port, Path visor, Path devices, Path restart) throws IOException {
    Path root = Files.createDirectories(work.resolve("root"));
    Files.createDirectories(root.resolve("var/lib/frc-spotter"));
    ProcessBuilder builder =
        new ProcessBuilder(
                "/bin/sh",
                "-c",
                // With the umask its systemd unit gives it (UMask=0077), as on a board.
                "umask 077; exec \"$0\" -Xmx128m -jar \"$1\" --root=\"$2\" --controller=127.0.0.1"
                    + " --port=\"$3\"",
                ProcessHandle.current().info().command().orElse("java"),
                System.getProperty("frc.spotterAgent"),
                root.toString(),
                Integer.toString(port))
            .redirectErrorStream(true)
            .redirectOutput(ProcessBuilder.Redirect.appendTo(work.resolve("agent.log").toFile()));
    builder.environment().put("VISOR_DIR", visor.toString());
    builder.environment().put("VISOR_DEVICES", devices.toString());
    builder.environment().put("VISOR_RESTART", restart.toString());
    agent = builder.start();
    Thread thread =
        new Thread(
            () -> {
              while (!stopping) {
                try {
                  Objects.requireNonNull(agent).waitFor();
                  if (!stopping) {
                    Thread.sleep(200);
                    agent = builder.start();
                  }
                } catch (InterruptedException | IOException e) {
                  return;
                }
              }
            },
            "agent-supervisor");
    thread.setDaemon(true);
    thread.start();
    supervisor = thread;
  }

  /** The robot loop's part: the coprocessors first each loop, every 20 ms. */
  private void loopUntil(Coprocessors boards, BooleanSupplier done, String what) throws Exception {
    long until = System.nanoTime() + 60_000_000_000L;
    while (System.nanoTime() < until) {
      boards.periodic();
      if (done.getAsBoolean()) {
        return;
      }
      Thread.sleep(20);
    }
    Path log = work.resolve("agent.log");
    throw new AssertionError(
        "waited a minute for "
            + what
            + "; the agent said:\n"
            + (Files.exists(log) ? Files.readString(log) : "(nothing)"));
  }

  private static String read(Path file) {
    try {
      return Files.readString(file);
    } catch (IOException e) {
      return "";
    }
  }

  private static Path executable(Path file, String text) throws IOException {
    Files.writeString(file, text);
    Files.setPosixFilePermissions(file, PosixFilePermissions.fromString("rwxr-xr-x"));
    return file;
  }

  private static int freePort() throws IOException {
    try (ServerSocket socket = new ServerSocket(0)) {
      return socket.getLocalPort();
    }
  }
}
