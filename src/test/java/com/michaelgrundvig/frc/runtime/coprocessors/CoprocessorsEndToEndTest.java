package com.michaelgrundvig.frc.runtime.coprocessors;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.pitwall.Devices;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotConfig;
import com.michaelgrundvig.frc.runtime.RobotConfigBuilder;
import com.michaelgrundvig.frc.spotter.manager.BoardDefinition;
import com.michaelgrundvig.frc.spotter.manager.Connection;
import com.michaelgrundvig.frc.spotter.manager.Level;
import com.michaelgrundvig.frc.visor.VisorPack;
import com.michaelgrundvig.frc.visor.VisorVersion;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfig.Software;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.nio.file.attribute.PosixFilePermissions;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.simulation.AlertSim;
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

  /** The robot loop's period, which the test's loops sleep between them. */
  private static final long LOOP_MILLIS = 20;

  /** The longest a wait for the robot, the agent or the board takes before the test fails. */
  private static final long WAIT_NANOS = 60_000_000_000L;

  /**
   * How many loops a board brought up to date is watched for, to see it's left alone: past the
   * installer's settle time after its last step, and two of the board's 2 s reports after that, so
   * the installer has read the board up to date and had its chance to start another step. Counted
   * in loops, each at least {@link #LOOP_MILLIS}, so a slow machine watches for longer, never less.
   */
  private static final int LEFT_ALONE_LOOPS =
      (int) ((VisorInstaller.SETTLE_NANOS / 1_000_000 + 4_000) / LOOP_MILLIS);

  /** What the agent says once it serves, with the port it bound. */
  private static final Pattern SERVING = Pattern.compile("serving protocol 2 on port (\\d+)");

  @TempDir Path work;

  private @Nullable Process agent;
  private volatile boolean stopping;
  private @Nullable Thread supervisor;
  private @Nullable Coprocessors coprocessors;

  /** How a start of the agent failed, once one has: the test ends with it. */
  private volatile @Nullable String agentFailed;

  /** A port bound and never listened on: a board where nothing answers, refused at once. */
  private @Nullable Socket nothingAnswers;

  @BeforeEach
  void noAlertsLeftOver() {
    AlertSim.resetData();
  }

  @AfterEach
  void stop() throws InterruptedException, IOException {
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
    if (nothingAnswers != null) {
      nothingAnswers.close();
    }
    AlertSim.resetData();
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
    int port = startAgent(visor, devices, restart);
    String dead = nothingAnswers(); // a second board

    // The robot's deploy folder: Visor's pack among the team's packs, and its release.
    Path deploy = Files.createDirectories(work.resolve("deploy"));
    unpack(Path.of(System.getProperty("frc.visorRelease")), work.resolve("released"));
    Path packs = Files.createDirectories(deploy.resolve("spotter-packs"));
    copyFolder(work.resolve("released/spotter/visor"), packs.resolve("visor"));
    Path releases = Files.createDirectories(deploy.resolve("visor"));
    Path archive = releases.resolve(VisorPack.releaseName(VisorVersion.VERSION));
    release(archive);

    // The guide's robot with its first camera on Visor, plugged into the board.
    CameraConfig first = ExampleRobot.COMPETITION.definition().cameras().get(0);
    CameraConfig onVisor = first.withSoftware(Software.visor().device("/dev/v4l/by-id/" + CAMERA));
    RobotConfig robot =
        RobotConfigBuilder.builder(ExampleRobot.COMPETITION.with(onVisor))
            .coprocessors(List.of("127.0.0.1:" + port, dead))
            .build();
    String wanted =
        VisorConfigs.forBoard(
                robot.definition().cameras(), 1234, VisorConfigsTest.TAG_SIZE, List.of(CAMERA))
            .toml();

    // The release is the test's own, so the robot names its SHA-256, as a client published with it
    // would.
    Coprocessors robotsBoards =
        Coprocessors.withBoards(
            Coprocessors.definitions(robot.coprocessors(), robot.definition().cameras()),
            VisorTuning.live(robot.definition().cameras(), Map.of()),
            deploy,
            1234,
            VisorConfigsTest.TAG_SIZE,
            VisorPack.sha256(Files.readAllBytes(archive)),
            VisorMemory.NONE,
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
    // Visor is restarted by the install and by the configuration, each as the agent runs it.
    loopUntil(robotsBoards, () -> lines(restarts) >= 2, "Visor restarted twice");
    assertThat(restarts).content().isEqualTo("restarted\nrestarted\n");
    // The board that never answers is the robot's alert, though the other was seen to.
    assertThat(coprocessorsAlerts()).contains("HIGH " + dead + "/missing");

    // Brought up to date, it's left alone.
    for (int loop = 0; loop < LEFT_ALONE_LOOPS; loop++) {
      robotsBoards.periodic();
      restOfTheLoop();
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
    CameraConfig onVisor = first.withSoftware(Software.visor().device("/dev/v4l/by-id/" + CAMERA));
    RobotConfig robot = ExampleRobot.COMPETITION.with(onVisor);
    Path deploy = Files.createDirectories(work.resolve("deploy"));
    List<String> boards = List.of(nothingAnswers());
    for (int team : new int[] {0, 1234}) {
      try (Coprocessors robotsBoards =
          Coprocessors.on(
              boards,
              VisorTuning.live(robot.definition().cameras(), Map.of()),
              deploy,
              team,
              VisorConfigsTest.TAG_SIZE,
              work.resolve("kept"),
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

  /**
   * A board's health reaches the robot's log through Spotter's real agent: the board runs a
   * stand-in for Spotter's {@code debian} pack (its image's, on a real board; pushed by the robot
   * here), whose scripts report what the test writes. It's fine, then throttles, runs near slowing
   * for heat, loses a USB device and has rebooted, and its card says each. The other board never
   * answers, and its card says it's missing.
   */
  @Test
  void aBoardsHealthReachesTheRobotThroughItsAgent() throws Exception {
    Path visor = Files.createDirectories(work.resolve("opt-visor"));
    Path devices = Files.createDirectories(work.resolve("by-id"));
    Path restart = executable(work.resolve("restart"), "#!/bin/sh\n");
    int port = startAgent(visor, devices, restart);
    String dead = nothingAnswers();
    Path deploy = Files.createDirectories(work.resolve("deploy"));
    Path reports = Files.createDirectories(work.resolve("reports"));
    standInDebianPack(Files.createDirectories(deploy.resolve("spotter-packs/debian")), reports);
    report(reports, 42, false, 61, 24, 0, 600);
    List<BoardDefinition> boards =
        List.of(
            new BoardDefinition("127.0.0.1:" + port, List.of("debian")), new BoardDefinition(dead));
    Coprocessors robotsBoards =
        // The debian stand-in is pushed, as a test's board can't have it installed as root.
        Coprocessors.withBoards(
            boards,
            VisorTuning.live(ExampleRobot.COMPETITION.definition().cameras(), Map.of()),
            deploy,
            1234,
            VisorConfigsTest.TAG_SIZE,
            VisorVersion.RELEASE_SHA256,
            VisorMemory.NONE,
            () -> false,
            () -> false,
            System::nanoTime);
    coprocessors = robotsBoards;
    CoprocessorHealth health = robotsBoards.health();
    CoprocessorHealth.Watched board = health.boards().get(0);
    CoprocessorHealth.Watched away = health.boards().get(1);
    Runnable loop =
        () -> {
          robotsBoards.periodic();
          health.periodic(System.nanoTime());
        };

    loopUntil(
        loop,
        () -> board.inputs.uptimeSeconds == 600 && away.inputs.connection == Connection.MISSING,
        "the board's health read");
    BoardIO.BoardIOInputs inputs = board.inputs;
    assertThat(inputs.connection).isEqualTo(Connection.CONNECTED);
    assertThat(inputs.cpuBusiestPercent).isEqualTo(42);
    assertThat(inputs.cpuCapped).isFalse();
    assertThat(inputs.hottestCelsius).isEqualTo(61);
    assertThat(inputs.thermalMarginCelsius).isEqualTo(24);
    assertThat(inputs.thermalMarginLevel).isEqualTo(Level.OK);
    assertThat(inputs.usbDevices).isEqualTo("7-1 ThriftyCam");
    assertThat(inputs.usbDisconnects).isZero();
    assertThat(inputs.unavailable).isEmpty();
    Devices.Card fine = new BoardPit(board).card();
    assertThat(fine.level()).as(fine.lines().toString()).isEqualTo(Devices.Level.OK);
    Devices.Card missing = new BoardPit(away).card();
    assertThat(missing.state()).isEqualTo("Missing");
    assertThat(missing.level()).isEqualTo(Devices.Level.PROBLEM);

    // Hot, slowed, a camera dropped off, and up for less than before: it rebooted.
    report(reports, 97, true, 88, 8, 1, 30);
    loopUntil(
        loop,
        () ->
            board.health.reboots() == 1
                && board.health.throttled().holds()
                && board.health.lowThermalMargin().holds()
                && board.health.usbDisconnects() == 1,
        "the board's trouble read");
    assertThat(board.inputs.thermalMarginLevel).isEqualTo(Level.WARNING);
    Devices.Card card = new BoardPit(board).card();
    assertThat(card.level()).isEqualTo(Devices.Level.WARNING);
    assertThat(card.lines())
        .extracting(line -> line.label() + ": " + line.value())
        .anySatisfy(line -> assertThat(line).startsWith("Up for: 30 s; rebooted once"))
        .anySatisfy(line -> assertThat(line).contains("slowed for heat or power"))
        .anySatisfy(line -> assertThat(line).contains("8 °C from slowing; near slowing"))
        .anySatisfy(line -> assertThat(line).contains("a device dropped off once"));
  }

  /**
   * A stand-in for Spotter's debian pack: its values' ids, types and limits, each collector a
   * script that prints what the test last wrote ({@link #report}).
   */
  private static void standInDebianPack(Path pack, Path reports) throws IOException {
    Files.writeString(
        pack.resolve("pack.yaml"),
        """
        pack: debian
        version: 1.0.0
        collectors:
          - id: cpu
            run: [./cpu]
            every: 1s
            fields:
              busiest: {label: Busiest core, type: number, unit: "%"}
              capped: {label: CPU slowed for heat or power, type: boolean, warn: {equals: true}}
          - id: thermal
            run: [./thermal]
            every: 1s
            fields:
              hottest: {label: Hottest zone, type: number, unit: "°C"}
              margin: {label: Thermal margin, type: number, unit: "°C", warn: {below: 10}, \
        fail: {below: 5}}
          - id: usb
            run: [./usb]
            every: 1s
            fields:
              devices: {label: USB devices, type: text}
              disconnects: {label: USB disconnects since boot, type: number, warn: {above: 0}}
          - id: uptime
            run: [./uptime]
            every: 1s
            fields:
              seconds: {label: Up for, type: number, unit: s}
        """);
    for (String collector : new String[] {"cpu", "thermal", "usb", "uptime"}) {
      executable(
          pack.resolve(collector),
          "#!/bin/sh\ncat '" + reports.resolve(collector + ".json") + "'\n");
    }
  }

  /** What the stand-in pack reports from now on, each file written whole. */
  private static void report(
      Path reports,
      double busiest,
      boolean capped,
      double hottest,
      double margin,
      int disconnects,
      double uptime)
      throws IOException {
    write(
        reports,
        "cpu",
        String.format(Locale.ROOT, "{\"busiest\": %s, \"capped\": %s}", busiest, capped));
    write(
        reports,
        "thermal",
        String.format(Locale.ROOT, "{\"hottest\": %s, \"margin\": %s}", hottest, margin));
    write(
        reports,
        "usb",
        String.format(
            Locale.ROOT, "{\"devices\": \"7-1 ThriftyCam\", \"disconnects\": %d}", disconnects));
    write(reports, "uptime", String.format(Locale.ROOT, "{\"seconds\": %s}", uptime));
  }

  private static void write(Path reports, String collector, String json) throws IOException {
    Path written = reports.resolve(collector + ".json.new");
    Files.writeString(written, json + "\n");
    Files.move(written, reports.resolve(collector + ".json"), StandardCopyOption.ATOMIC_MOVE);
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

  /**
   * Visor's program, as the pack sees it: its version, a check that passes, and the camera plugged
   * in, by its name, as {@code visor cameras --names} names it.
   */
  private static String program(String version) {
    return "#!/bin/sh\ncase \"$1\" in\n  version) echo \"visor "
        + version
        + " (version ID 1)\" ;;\n  check) echo \"$3: checked\" ;;\n  cameras) echo "
        + CAMERA
        + " ;;\nesac\n";
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
   * The agent, as systemd runs it on a board: started again whenever it ends itself, as it does
   * after a push to read the packs pushed. It first serves on a port the system picks as it binds,
   * so no other program can take the port between the test's choosing it and the agent's binding
   * it, and each start after is on that port. A start that fails (its port taken in the moment
   * between two starts, say) ends the test with what the agent said, rather than being started
   * again until a wait gives up.
   *
   * @return the port the agent serves on
   */
  private int startAgent(Path visor, Path devices, Path restart) throws Exception {
    Path root = Files.createDirectories(work.resolve("root"));
    Files.createDirectories(root.resolve("var/lib/frc-spotter"));
    // udev's names, by-id's alone, and the kernel's description of each device, with its serial:
    // a camera named by its serial has to report one.
    Path byPath = Files.createDirectories(work.resolve("by-path"));
    Path sysfs = Files.createDirectories(work.resolve("sysfs"));
    try (var names = Files.list(devices)) {
      for (Path name : names.toList()) {
        Path video = Files.createDirectories(sysfs.resolve(name.getFileName()).resolve("device"));
        Files.writeString(video.resolveSibling("serial"), "0001\n");
      }
    }
    Map<String, String> environment = new LinkedHashMap<>();
    environment.put("VISOR_DIR", visor.toString());
    environment.put("VISOR_BY_ID", devices.toString());
    environment.put("VISOR_BY_PATH", byPath.toString());
    environment.put("VISOR_SYSFS", sysfs.toString());
    environment.put("VISOR_RESTART", restart.toString());
    // Where the service would say what it runs: nothing there, as on an image whose unit can't.
    environment.put("VISOR_RUN", Files.createDirectories(work.resolve("run")).toString());

    agent = agentOn(0, root, environment).start();
    int port = servedPort();
    ProcessBuilder again = agentOn(port, root, environment);
    Thread thread =
        new Thread(
            () -> {
              while (!stopping) {
                try {
                  int exit = Objects.requireNonNull(agent).waitFor();
                  if (stopping) {
                    return;
                  }
                  if (exit != 0) {
                    agentFailed = "the agent on port " + port + " ended with exit code " + exit;
                    return;
                  }
                  Thread.sleep(200);
                  agent = again.start();
                } catch (InterruptedException | IOException e) {
                  return;
                }
              }
            },
            "agent-supervisor");
    thread.setDaemon(true);
    thread.start();
    supervisor = thread;
    return port;
  }

  /** The agent's command on a port, 0 for one the system picks, its output in the test's log. */
  private ProcessBuilder agentOn(int port, Path root, Map<String, String> environment) {
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
    builder.environment().putAll(environment);
    return builder;
  }

  /** The port the agent's first start bound, once it says it serves. */
  private int servedPort() throws Exception {
    Path log = work.resolve("agent.log");
    long until = System.nanoTime() + WAIT_NANOS;
    while (System.nanoTime() < until) {
      Matcher serving = SERVING.matcher(read(log));
      if (serving.find()) {
        return Integer.parseInt(serving.group(1));
      }
      if (!Objects.requireNonNull(agent).isAlive()) {
        throw new AssertionError("the agent ended before it served; it said:\n" + read(log));
      }
      Thread.sleep(LOOP_MILLIS);
    }
    throw new AssertionError("waited a minute for the agent to serve; it said:\n" + read(log));
  }

  /**
   * A board where nothing answers, at a port the test holds bound and never listens on: a
   * connection to it is refused, and no other program can take the port while the test runs.
   */
  private String nothingAnswers() throws IOException {
    Socket held = new Socket();
    held.bind(new InetSocketAddress("127.0.0.1", 0));
    nothingAnswers = held;
    return "127.0.0.1:" + held.getLocalPort();
  }

  /** The robot loop's part: the coprocessors first each loop, every 20 ms. */
  private void loopUntil(Coprocessors boards, BooleanSupplier done, String what) throws Exception {
    loopUntil(boards::periodic, done, what);
  }

  /** The robot loop's part, every 20 ms, until it's done. */
  private void loopUntil(Runnable loop, BooleanSupplier done, String what) throws Exception {
    long until = System.nanoTime() + WAIT_NANOS;
    while (System.nanoTime() < until) {
      loop.run();
      if (done.getAsBoolean()) {
        return;
      }
      restOfTheLoop();
    }
    throw new AssertionError("waited a minute for " + what + "; the agent said:\n" + agentLog());
  }

  /**
   * The rest of the robot loop's period, after its turn. A failed start of the agent ends the test
   * here, saying so, whatever it was waiting for.
   */
  private void restOfTheLoop() throws InterruptedException {
    String failed = agentFailed;
    if (failed != null) {
      throw new AssertionError(failed + "; it said:\n" + agentLog());
    }
    Thread.sleep(LOOP_MILLIS);
  }

  private String agentLog() {
    Path log = work.resolve("agent.log");
    if (!Files.exists(log)) {
      return "(nothing)";
    }
    return read(log);
  }

  /** How many lines a file has, none if it isn't there yet. */
  private static long lines(Path file) {
    return read(file).lines().count();
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
}
