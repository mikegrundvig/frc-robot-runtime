package com.michaelgrundvig.frc.runtime.coprocessors;

import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.HexFormat;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.LongSupplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.util.Alert;

/**
 * Brings each coprocessor's Visor up to date with the robot, while the robot is disabled: each
 * board reports what it has (Visor's Spotter pack: its version, its configuration's hash, the
 * cameras plugged in), and a board with any of the robot's Visor cameras gets this robot's Visor
 * release, then the configuration for those cameras, wherever it has another. So a board that
 * missed a deploy, a spare swapped in, or a camera moved to another board is put right as the robot
 * starts, or as the board comes back.
 *
 * <p>Each step is tried once for what's wanted: one that fails is said by an alert and not tried
 * again until what's wanted changes, or the program restarts. A board that's never reported, or has
 * no camera of the robot's, is left alone.
 */
public final class VisorInstaller implements AutoCloseable {
  /** What a board says it has, through Visor's pack. */
  public record Report(String version, String config, List<String> cameras) {}

  /** A coprocessor, as the installer sees it: Spotter's board ({@link SpotterBoards}). */
  public interface Board {
    /** Its name, for the alerts. */
    String name();

    /** What it has, once it's connected and reporting through Visor's pack; else empty. */
    Optional<Report> report();

    /** Installs a release: its archive, {@code visor-<version>-linux-arm64.tar.gz}. */
    Step install(byte[] release);

    /** Puts a configuration in place, once Visor's check of it passes. */
    Step configure(String config);
  }

  /** An install or a configuration under way on a board. */
  public interface Step {
    boolean done();

    /** Once done: whether it worked. */
    boolean succeeded();

    /** Once done and failed: why, in the board's words. */
    String why();
  }

  /** Where this robot's Visor release comes from: its archive's bytes. */
  public interface Release {
    byte[] bytes() throws java.io.IOException;
  }

  /**
   * How long after a step succeeds the board's report may still show what it had: its status is
   * read every 2 s.
   */
  static final long SETTLE_NANOS = 10_000_000_000L;

  private final List<CameraConfig> cameras;
  private final int team;
  private final String version;
  private final Release release;
  private final BooleanSupplier disabled;
  private final LongSupplier nanos;
  private final List<OnBoard> boards = new ArrayList<>();
  private final Alert unplaced = new Alert("Visor", "unplaced", "", Alert.Level.MEDIUM);
  private byte @Nullable [] releaseBytes;

  /** One board, and what the installer last did there. */
  private static final class OnBoard {
    final Board board;
    final Alert failed;
    final Alert working;
    @Nullable Step step;
    String doing = "";
    String failedAt = "";
    String settled = "";
    long settledNanos;

    OnBoard(Board board) {
      this.board = board;
      this.failed = new Alert("Visor", "failed/" + board.name(), "", Alert.Level.HIGH);
      this.working = new Alert("Visor", "working/" + board.name(), "", Alert.Level.LOW);
    }
  }

  /**
   * @param cameras the robot's cameras, in its definition's order
   * @param team the team number, for each configuration
   * @param version the Visor this robot carries: its client's ({@code VisorVersion.VERSION})
   * @param release that version's archive, read the first time a board needs it
   * @param disabled whether the robot is disabled and off the field: the only time a board is
   *     changed
   * @param nanos the robot's clock
   */
  public VisorInstaller(
      List<CameraConfig> cameras,
      int team,
      String version,
      Release release,
      BooleanSupplier disabled,
      LongSupplier nanos,
      List<? extends Board> boards) {
    this.cameras = List.copyOf(cameras);
    this.team = team;
    this.version = version;
    this.release = release;
    this.disabled = disabled;
    this.nanos = nanos;
    for (Board board : boards) {
      this.boards.add(new OnBoard(board));
    }
  }

  /** Once each loop: looks at each board, and starts what it needs. */
  public void periodic() {
    List<String> devices = new ArrayList<>();
    boolean everyBoardReported = true;
    for (OnBoard on : boards) {
      Optional<Report> report = on.board.report();
      if (report.isEmpty()) {
        everyBoardReported = false;
      } else {
        devices.addAll(report.get().cameras());
      }
      look(on, report);
    }
    // Only once every board has said what it has can a camera be on none of them.
    List<String> missing = everyBoardReported ? VisorConfigs.unplaced(cameras, devices) : List.of();
    unplaced.setText(
        "Visor cameras plugged into no coprocessor: "
            + String.join(", ", missing)
            + ". Plug them in, or check each one's device in Robots.java.");
    unplaced.set(!missing.isEmpty());
  }

  private void look(OnBoard on, Optional<Report> report) {
    Step step = on.step;
    if (step != null) {
      if (!step.done()) {
        return;
      }
      on.step = null;
      on.working.set(false);
      if (step.succeeded()) {
        on.settled = on.doing;
        on.settledNanos = nanos.getAsLong();
        on.failed.set(false);
      } else {
        fail(on, on.doing, step.why());
      }
      return;
    }
    if (report.isEmpty() || !disabled.getAsBoolean()) {
      return;
    }
    Optional<String> wanted = VisorConfigs.forBoard(cameras, team, report.get().cameras());
    if (wanted.isEmpty()) {
      return;
    }
    if (!report.get().version().equals(version)) {
      start(on, "install " + version, () -> on.board.install(releaseBytes()));
    } else if (!report.get().config().equals(sha256(wanted.get()))) {
      start(on, "configure " + sha256(wanted.get()), () -> on.board.configure(wanted.get()));
    }
  }

  /** A step that makes a board's step. */
  private interface Starting {
    Step start() throws java.io.IOException;
  }

  private void start(OnBoard on, String doing, Starting starting) {
    if (doing.equals(on.failedAt)) {
      return; // said already, and not tried again until what's wanted changes
    }
    if (doing.equals(on.settled) && nanos.getAsLong() - on.settledNanos < SETTLE_NANOS) {
      return; // done: the board's report hasn't caught up yet
    }
    on.doing = doing;
    try {
      on.step = starting.start();
    } catch (java.io.IOException e) {
      fail(
          on,
          doing,
          "this robot's Visor release can't be read (" + e.getMessage() + "): deploy again");
      return;
    }
    on.working.setText(
        (doing.startsWith("install") ? "Installing Visor " + version : "Configuring Visor")
            + " on "
            + on.board.name());
    on.working.set(true);
  }

  private void fail(OnBoard on, String doing, String why) {
    on.failedAt = doing;
    on.failed.setText(
        "Visor on "
            + on.board.name()
            + (doing.startsWith("install") ? " didn't install: " : " refused its configuration: ")
            + why);
    on.failed.set(true);
  }

  private byte[] releaseBytes() throws java.io.IOException {
    byte[] bytes = releaseBytes;
    if (bytes == null) {
      bytes = release.bytes();
      releaseBytes = bytes;
    }
    return bytes;
  }

  /** A configuration's SHA-256, as the board's report gives it: lowercase hex. */
  static String sha256(String text) {
    try {
      return HexFormat.of()
          .formatHex(
              MessageDigest.getInstance("SHA-256").digest(text.getBytes(StandardCharsets.UTF_8)));
    } catch (NoSuchAlgorithmException e) {
      throw new IllegalStateException("every Java has SHA-256", e);
    }
  }

  @Override
  public void close() {
    unplaced.close();
    for (OnBoard on : boards) {
      on.failed.close();
      on.working.close();
    }
  }
}
