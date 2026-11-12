package com.michaelgrundvig.frc.runtime.coprocessors;

import com.michaelgrundvig.frc.visor.VisorPack;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.function.BooleanSupplier;
import java.util.function.LongSupplier;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.util.Alert;

/**
 * Brings each coprocessor's Visor up to date with the robot, while the robot is disabled and off
 * the field: each board with Visor's Spotter pack reports what it has ({@link VisorPack}: the Visor
 * on its disk and the one running, their configurations' hashes, the cameras plugged in), and gets
 * this robot's Visor release wherever its disk has another, then the configuration for the robot's
 * cameras plugged into it. So a board that missed a deploy, a spare swapped in, or a camera moved
 * to another board is put right as the robot starts, or as the board comes back. The cameras are
 * read each loop, so a camera's exposure or gain tuned live ({@link VisorTuning}) reaches its board
 * the same way, once the robot is disabled off the field.
 *
 * <p>What a board runs is believed over what's on its disk: a Visor put in place but never started
 * is restarted (a configure restarts it), then installed again. A board that can't say what runs
 * (its image is older than the pack's report of it, or Visor isn't running) is judged by its disk,
 * and left alone once that's right.
 *
 * <p>A step that gets no answer (the board rebooted or dropped off mid-step, the run timed out, a
 * push of the robot's packs restarted its agent) is tried again {@value #RETRIES} times, {@link
 * #RETRY_NANOS} apart, and from the start when the board's report comes back or it reboots. One the
 * board refuses (Visor refused the configuration, the release won't run there) is said by an alert
 * and not tried again until what's wanted changes. No step starts while a push of the robot's packs
 * to the board is under way, and none while the robot is enabled or on the field: one already
 * running then is shown, and finishes.
 *
 * <p>Each board's Visor frames are taken only from a configuration the installer gave it ({@link
 * #otherConfig}): the one wanted now, the committed one, the last one it saw the board run while it
 * was the one wanted, and what was kept of these before the program restarted, while the committed
 * configuration is still the one it was kept with ({@link VisorMemory}). So a tuned exposure that
 * hasn't reached its board yet, a configuration on its way, or a tuning a board runs on after a
 * restart on the field costs no frames, and a board running one the robot didn't give it (cameras
 * reordered, a lens calibrated again) or one that never reached it puts none into localization.
 * Until a board has reported with this robot's Visor on its disk, which configurations it may run
 * isn't known, and its frames aren't refused for it.
 *
 * <p>The release is checked against the SHA-256 the robot's Visor client was published with as it's
 * read, and sent to each board naming that hash, which the board checks before anything in the
 * archive runs ({@link VisorPack#installInput}).
 *
 * <p>A board without Visor's pack is ignored, and so is every board while none of the robot's
 * cameras is on Visor. A board that runs a configuration but has none of the robot's cameras is
 * given one with no camera. The wanted configurations are rebuilt only when the cameras or a
 * board's report change, and the release is read once, off the loop.
 */
public final class VisorInstaller implements AutoCloseable {
  /** What a board says, as the installer last read it. */
  public sealed interface Seen {
    /** Not reached, or not reporting through Visor's pack yet. */
    Seen SILENT = new Silent();

    /** Reached, and without Visor's pack: none of the installer's. */
    Seen WITHOUT_VISOR = new WithoutVisor();

    /** Not reached, or not reporting yet. */
    record Silent() implements Seen {}

    /** Reached, and without Visor's pack. */
    record WithoutVisor() implements Seen {}

    /** Reporting through Visor's pack. */
    record Reported(VisorPack.Report report) implements Seen {}
  }

  /** A coprocessor, as the installer sees it: Spotter's board ({@link SpotterBoards}). */
  public interface Board {
    /** Its name, for the alerts. */
    String name();

    /**
     * Its address as the robot's configuration lists it, without its agent's port: what a Visor
     * frame's sender is matched to ({@link VisorPlacements#withoutPort}).
     */
    String address();

    /** What it says now: the same value while nothing on it changed. */
    Seen seen();

    /** Whether a push of the robot's packs to it is under way, which restarts its agent. */
    boolean pushPending();

    /** Seconds since it booted; NaN while it can't say. */
    double uptimeSeconds();

    /**
     * Installs a release: its archive after the line naming its SHA-256, as {@link
     * VisorPack#installInput} makes it.
     */
    Step install(byte[] input);

    /** Puts a configuration in place, once Visor's check of it passes, and restarts Visor. */
    Step configure(String config);
  }

  /** How a step ended. */
  public enum Outcome {
    /** It did what it was asked. */
    WORKED,
    /** It ran to the end and said no: tried again only for something else. */
    REFUSED,
    /** It never said: not reached, timed out, or lost as the board's agent restarted. */
    NO_ANSWER
  }

  /** An install or a configuration under way on a board. */
  public interface Step {
    boolean done();

    /** Once done: how it ended. */
    Outcome outcome();

    /** Once done and not worked: why, in the board's words. */
    String why();
  }

  /** Where this robot's Visor release comes from: its archive's bytes. */
  public interface Release {
    byte[] bytes() throws IOException;
  }

  /** How many times a step that didn't take is tried again before the installer stops. */
  static final int RETRIES = 3;

  /** How long after a step that didn't take the same step is tried again. */
  static final long RETRY_NANOS = 30_000_000_000L;

  /**
   * How long after a step ends before its board is asked for another: its status is read every 2 s,
   * so its report may still show what it had.
   */
  static final long SETTLE_NANOS = 10_000_000_000L;

  private final Supplier<List<CameraConfig>> cameras;
  private final List<CameraConfig> committed;

  /**
   * Whether any of the robot's cameras is on Visor. Without one, the robot pushes no board Visor's
   * pack, and takes away one pushed before, so a board still reporting through it is left alone.
   */
  private final boolean onVisor;

  private final int team;
  private final double tagSizeMeters;
  private final String version;
  private final String releaseSha256;
  private final Release release;
  private final Executor reading;
  private final BooleanSupplier enabled;
  private final BooleanSupplier fieldAttached;
  private final LongSupplier nanos;
  private final List<OnBoard> boards = new ArrayList<>();
  private final Alert unplaced = new Alert("Visor", "unplaced", "", Alert.Level.MEDIUM);
  private final Alert waiting = new Alert("Visor", "waiting", "", Alert.Level.LOW);
  private final Alert uncommitted = new Alert("Visor", "uncommitted", "", Alert.Level.MEDIUM);
  private final Alert forgetting = new Alert("Visor", "forgetting", "", Alert.Level.MEDIUM);
  private final VisorMemory memory;
  private String memoryProblem = "";
  private @Nullable CompletableFuture<byte[]> releaseRead;
  private List<CameraConfig> seenCameras = List.of();

  /** Whether each camera's configuration hasn't reached its board yet, by its index. */
  private final boolean[] pending;

  private boolean anyPending;
  private boolean anyUncommitted;

  /** What a board's step does. */
  private enum Kind {
    INSTALL,
    /** A configure of what's wanted, which starts the Visor on its disk. */
    RESTART,
    CONFIGURE
  }

  /** Where a step's tries are. */
  private enum Tries {
    /** Tried no more often than it may be. */
    GOING,
    /** The board refused it: not tried again for the same thing. */
    REFUSED,
    /** Tried as often as it may be: again only once the board comes back or reboots. */
    GAVE_UP
  }

  /** One board, and what the installer last did there. */
  private static final class OnBoard {
    final Board board;
    final Alert failed;
    final Alert working;
    final Alert landing;
    Seen seen = Seen.SILENT;
    boolean pushing;
    @Nullable Step step;

    /** The configuration wanted for its cameras now; null until its cameras are known. */
    VisorConfigs.@Nullable ForBoard wanted;

    /** The committed configuration's hash for its cameras; null until they're known. */
    @Nullable String committedHash;

    /**
     * Which configurations its frames may come from, by ID: known once what it wants first is, and
     * kept while it can't say, so a board that's away for a moment keeps them.
     */
    final TakenConfigs taken;

    /** Its last step: what it was for (a version or a hash), and how often it was started. */
    @Nullable Kind kind;

    String target = "";
    int tries;
    Tries state = Tries.GOING;

    /** When its last step ended, once one has. */
    boolean ended;

    long endedNanos;

    /** Whether the Visor on its disk was restarted since it last ran another. */
    boolean restarted;

    /** Whether it went silent, or rebooted, since it last reported: its tries start over. */
    boolean cameBack;

    double uptime = Double.NaN;

    OnBoard(Board board, VisorMemory.@Nullable Kept recalled) {
      this.board = board;
      this.taken = new TakenConfigs(recalled);
      this.failed = new Alert("Visor", "failed/" + board.name(), "", Alert.Level.HIGH);
      this.working = new Alert("Visor", "working/" + board.name(), "", Alert.Level.LOW);
      this.landing = new Alert("Visor", "landing/" + board.name(), "", Alert.Level.MEDIUM);
    }
  }

  /**
   * @param cameras the robot's cameras, in its definition's order, as tuned now: read each loop,
   *     and the same list while nothing changed ({@link VisorTuning#live})
   * @param committed the robot's cameras as its definition commits them
   * @param team the team number, for each configuration
   * @param tagSizeMeters the side of the field's tags' black square, for each configuration
   * @param version the Visor this robot carries: its client's ({@code VisorVersion.VERSION})
   * @param releaseSha256 the SHA-256 that version's archive has, as its client was published with
   *     it ({@code VisorVersion.RELEASE_SHA256}): an archive without it isn't sent
   * @param release that version's archive, read the first time a board needs it
   * @param reading where the release is read, off the loop
   * @param enabled whether the robot is enabled
   * @param fieldAttached whether the field is attached
   * @param nanos the robot's clock
   * @param memory which configurations each board's frames were taken from before the program
   *     started, and where what's taken now is kept
   */
  public VisorInstaller(
      Supplier<List<CameraConfig>> cameras,
      List<CameraConfig> committed,
      int team,
      double tagSizeMeters,
      String version,
      String releaseSha256,
      Release release,
      Executor reading,
      BooleanSupplier enabled,
      BooleanSupplier fieldAttached,
      LongSupplier nanos,
      List<? extends Board> boards,
      VisorMemory memory) {
    this.cameras = cameras;
    this.committed = List.copyOf(committed);
    this.onVisor =
        committed.stream()
            .anyMatch(camera -> camera.software() instanceof CameraConfig.Software.Visor);
    this.team = team;
    this.tagSizeMeters = tagSizeMeters;
    this.version = version;
    this.releaseSha256 = releaseSha256;
    this.release = release;
    this.reading = reading;
    this.enabled = enabled;
    this.fieldAttached = fieldAttached;
    this.nanos = nanos;
    this.pending = new boolean[committed.size()];
    this.memory = memory;
    Map<String, VisorMemory.Kept> recalled = memory.recalled();
    for (Board board : boards) {
      this.boards.add(new OnBoard(board, recalled.get(board.address())));
    }
  }

  /** Once each loop: looks at each board, and starts what it needs. */
  public void periodic() {
    if (!onVisor) {
      return;
    }
    List<CameraConfig> now = cameras.get();
    boolean camerasChanged = !now.equals(seenCameras);
    seenCameras = now;
    boolean offField = !enabled.getAsBoolean() && !fieldAttached.getAsBoolean();
    boolean anyChanged = camerasChanged;
    boolean keep = false;
    // By index, as an iterator would be made each loop.
    for (int i = 0; i < boards.size(); i++) {
      OnBoard on = boards.get(i);
      if (heard(on) || camerasChanged) {
        keep |= want(on, now);
        anyChanged = true;
      }
      follow(on);
      look(on, offField);
    }
    if (anyChanged) {
      sayWhereTheCamerasAre(now);
    }
    if (keep) {
      keepWhatsTaken();
    }
    sayWhetherItsKept();
    waiting.set(anyPending && !offField);
    uncommitted.set(anyUncommitted && fieldAttached.getAsBoolean());
  }

  /**
   * Whether a camera's configuration as wanted now (tuned, or as committed) hasn't reached the
   * board it's plugged into yet: it waits for the robot to be disabled off the field, or is on its
   * way. False while its board isn't known, so a camera no board has reported isn't said to wait.
   *
   * @param camera its index in the robot's list of cameras
   */
  boolean pending(int camera) {
    if (camera < 0 || camera >= pending.length) {
      return false;
    }
    return pending[camera];
  }

  /**
   * Whether a Visor frame from this board, naming the configuration with this ID, comes from
   * another configuration than one the installer gave it: the one wanted now, the committed one, or
   * the last one it was seen running while it was the one wanted. False while the board's
   * configurations aren't known (it hasn't reported this robot's Visor on its disk yet), for a
   * board that isn't one of the installer's, and while no camera is on Visor. Asked for each frame,
   * so it allocates nothing.
   *
   * @param board the board's address as the robot's configuration lists it, without its port
   */
  public boolean otherConfig(String board, int configId) {
    for (int i = 0; i < boards.size(); i++) {
      OnBoard on = boards.get(i);
      if (!on.board.address().equals(board)) {
        continue;
      }
      if (!on.taken.known()) {
        return false;
      }
      return !on.taken.takes(configId);
    }
    return false;
  }

  /** Keeps which configurations each board's frames are taken from, off the loop. */
  private void keepWhatsTaken() {
    Map<String, VisorMemory.Kept> kept = new LinkedHashMap<>();
    for (OnBoard on : boards) {
      VisorMemory.Kept board = on.taken.kept();
      if (board != null) {
        kept.put(on.board.address(), board);
      }
    }
    memory.keep(kept);
  }

  /** Raises an alert while what's taken can't be kept: said once each time the reason changes. */
  private void sayWhetherItsKept() {
    String problem = memory.problem();
    if (problem.equals(memoryProblem)) {
      return;
    }
    memoryProblem = problem;
    forgetting.setText(
        "Visor's configurations for the coprocessors can't be kept on the robot's disk ("
            + problem
            + "): after a restart, a board running a tuning is left out until the robot is"
            + " disabled off the field");
    forgetting.set(!problem.isEmpty());
  }

  /**
   * Reads what a board says now and whether a push to it is under way: whether either changed. A
   * report is compared by value, as a board may read one again that says the same (Spotter's does
   * whenever any of its values changes, its uptime included).
   */
  private static boolean heard(OnBoard on) {
    Seen seen = on.board.seen();
    boolean pushing = on.board.pushPending();
    boolean changed = !seen.equals(on.seen) || pushing != on.pushing;
    // The newest kept even when it says the same, so the next loop compares it with itself.
    on.seen = seen;
    on.pushing = pushing;
    return changed;
  }

  /**
   * Works out what a board wants, from its report and the cameras: only when either changed.
   * Whether which configurations it's taken from, to be kept, changed.
   */
  private boolean want(OnBoard on, List<CameraConfig> now) {
    on.wanted = null;
    on.committedHash = null;
    if (!(on.seen instanceof Seen.Reported reported)) {
      return false;
    }
    // Its cameras are known once the Visor on its disk is the robot's, which names them.
    if (!reported.report().version().equals(version)) {
      return false;
    }
    List<String> devices = reported.report().cameras();
    VisorConfigs.ForBoard wanted = VisorConfigs.forBoard(now, team, tagSizeMeters, devices);
    on.wanted = wanted;
    String committedHash = wanted.hash();
    if (!now.equals(committed)) {
      committedHash = VisorConfigs.forBoard(committed, team, tagSizeMeters, devices).hash();
    }
    on.committedHash = committedHash;
    // The board runs what it says now: when that's the one wanted, its frames stay taken after
    // what's wanted changes, until the board is seen running the next one wanted.
    return on.taken.update(
        VisorPack.configId(wanted.hash()),
        VisorPack.configId(committedHash),
        runningConfig(reported.report()).equals(wanted.hash()));
  }

  /** Notes a board going silent or rebooting, so its tries start over once it reports again. */
  private static void follow(OnBoard on) {
    if (on.seen instanceof Seen.Silent) {
      on.cameBack = true;
    }
    double uptime = on.board.uptimeSeconds();
    if (Double.isNaN(uptime)) {
      return;
    }
    if (!Double.isNaN(on.uptime) && uptime < on.uptime) {
      on.cameBack = true;
    }
    on.uptime = uptime;
  }

  private void look(OnBoard on, boolean offField) {
    Step step = on.step;
    if (step != null) {
      if (!step.done()) {
        // Begun while disabled off the field, it runs on: stopping it halfway helps no one.
        on.landing.set(!offField);
        return;
      }
      finish(on, step);
      return;
    }
    if (!(on.seen instanceof Seen.Reported reported)) {
      return;
    }
    if (on.cameBack) {
      on.cameBack = false;
      startOver(on);
    }
    if (!offField || on.pushing) {
      return;
    }
    if (on.ended && nanos.getAsLong() - on.endedNanos < SETTLE_NANOS) {
      return; // the board's report may not show the last step yet
    }
    VisorPack.Report report = reported.report();
    if (!report.version().equals(version)) {
      start(on, Kind.INSTALL, version);
      return;
    }
    VisorConfigs.ForBoard wanted = on.wanted;
    if (wanted == null) {
      return;
    }
    String running = report.runningVersion();
    if (!running.isEmpty() && !running.equals(version)) {
      // On its disk and not running: restart it once, then install it again.
      start(on, on.restarted ? Kind.INSTALL : Kind.RESTART, version);
      return;
    }
    String config = runningConfig(report);
    if (wanted.empty() && config.isEmpty()) {
      // None of the robot's cameras, and no configuration: it runs nothing to stop.
      settled(on);
      return;
    }
    if (!config.equals(wanted.hash())) {
      start(on, Kind.CONFIGURE, wanted.hash());
      return;
    }
    settled(on);
  }

  /** The configuration a board runs, or when it can't say, the one on its disk. */
  private static String runningConfig(VisorPack.Report report) {
    if (report.runningConfig().isEmpty()) {
      return report.config();
    }
    return report.runningConfig();
  }

  /** A board that has what's wanted: nothing more to say, and its tries start over. */
  private static void settled(OnBoard on) {
    on.failed.set(false);
    on.kind = null;
    on.target = "";
    on.tries = 0;
    on.state = Tries.GOING;
    on.restarted = false;
  }

  /** A board that came back: what didn't take is tried from the start; a refusal stands. */
  private static void startOver(OnBoard on) {
    on.restarted = false;
    on.ended = false;
    if (on.state == Tries.REFUSED) {
      return;
    }
    on.tries = 0;
    on.state = Tries.GOING;
  }

  private void start(OnBoard on, Kind kind, String target) {
    if (kind != on.kind || !target.equals(on.target)) {
      on.kind = kind;
      on.target = target;
      on.tries = 0;
      on.state = Tries.GOING;
    }
    if (on.state != Tries.GOING) {
      return; // said already
    }
    if (on.tries > 0 && on.ended && nanos.getAsLong() - on.endedNanos < RETRY_NANOS) {
      return;
    }
    if (on.tries > RETRIES) {
      giveUp(on, "it didn't take");
      return;
    }
    Step step;
    if (kind == Kind.INSTALL) {
      CompletableFuture<byte[]> read = readRelease();
      if (!read.isDone()) {
        return;
      }
      if (read.isCompletedExceptionally()) {
        on.state = Tries.REFUSED;
        say(on, "this robot's Visor release can't be used (" + why(read) + "): deploy again");
        return;
      }
      step = on.board.install(read.join());
    } else {
      VisorConfigs.ForBoard wanted = on.wanted;
      if (wanted == null) {
        return;
      }
      step = on.board.configure(wanted.toml());
    }
    on.step = step;
    on.tries++;
    if (kind == Kind.RESTART) {
      on.restarted = true;
    }
    String doing = doing(on, kind);
    on.working.setText(doing);
    on.working.set(true);
    on.landing.setText(
        doing + " began before the robot was enabled: it finishes now, and its cameras restart");
  }

  private String doing(OnBoard on, Kind kind) {
    String doing =
        switch (kind) {
          case INSTALL -> "Installing Visor " + version;
          case RESTART -> "Restarting Visor " + version;
          case CONFIGURE -> "Configuring Visor";
        };
    return doing + " on " + on.board.name();
  }

  private void finish(OnBoard on, Step step) {
    on.step = null;
    on.working.set(false);
    on.landing.set(false);
    on.ended = true;
    on.endedNanos = nanos.getAsLong();
    switch (step.outcome()) {
      case WORKED -> on.failed.set(false);
      case REFUSED -> {
        on.state = Tries.REFUSED;
        say(on, step.why());
      }
      case NO_ANSWER -> {
        if (on.tries > RETRIES) {
          giveUp(on, step.why());
          return;
        }
        say(
            on,
            step.why()
                + "; trying again in "
                + RETRY_NANOS / 1_000_000_000L
                + " s ("
                + on.tries
                + " of "
                + (RETRIES + 1)
                + " tries)");
      }
    }
  }

  private static void giveUp(OnBoard on, String why) {
    on.state = Tries.GAVE_UP;
    say(
        on,
        why
            + "; stopped after "
            + on.tries
            + " tries, until the board reconnects or reboots, or the robot program restarts");
  }

  /** Raises a board's failure alert: what didn't happen, and why. */
  private static void say(OnBoard on, String why) {
    on.failed.setText("Visor on " + on.board.name() + didnt(on) + why);
    on.failed.set(true);
  }

  private static String didnt(OnBoard on) {
    Kind kind = on.kind;
    if (kind == Kind.INSTALL) {
      return " didn't install: ";
    }
    if (kind == Kind.RESTART) {
      return " didn't restart: ";
    }
    VisorConfigs.ForBoard wanted = on.wanted;
    boolean none = wanted != null && wanted.empty();
    // Only a configuration Visor checked and said no to is refused; one that got no answer wasn't.
    String took = on.state == Tries.REFUSED ? "refused" : "didn't take";
    if (none) {
      return " runs cameras that aren't this robot's, and "
          + took
          + " the configuration with none: ";
    }
    return " " + took + " its configuration: ";
  }

  /**
   * The install's input, read off the loop the first time a board needs it: the release, checked
   * against the SHA-256 the robot's Visor client names, after the line naming that hash.
   */
  private CompletableFuture<byte[]> readRelease() {
    CompletableFuture<byte[]> read = releaseRead;
    if (read == null) {
      read =
          CompletableFuture.supplyAsync(
              () -> {
                try {
                  return installInput(release.bytes());
                } catch (IOException e) {
                  throw new UncheckedIOException(e);
                }
              },
              reading);
      releaseRead = read;
    }
    return read;
  }

  /** The install's input for the release read, once it's the one the robot's client names. */
  private byte[] installInput(byte[] archive) throws IOException {
    if (releaseSha256.isEmpty()) {
      throw new IOException("the robot's Visor client names no SHA-256 for its release");
    }
    String sha256 = VisorPack.sha256(archive);
    if (!sha256.equals(releaseSha256)) {
      throw new IOException(
          "its SHA-256 is " + sha256 + ", not the " + releaseSha256 + " its Visor client names");
    }
    return VisorPack.installInput(archive, sha256);
  }

  /** Why a read failed: its first cause's words. */
  private static String why(CompletableFuture<byte[]> read) {
    try {
      read.join();
      return "";
    } catch (RuntimeException e) {
      Throwable cause = e;
      for (Throwable next = cause.getCause(); next != null; next = next.getCause()) {
        cause = next;
      }
      return String.valueOf(cause.getMessage());
    }
  }

  /**
   * Once a board's report or the cameras changed: which cameras no board has, which configurations
   * haven't reached their boards, and which boards run another than the committed one.
   */
  private void sayWhereTheCamerasAre(List<CameraConfig> now) {
    boolean everyBoardKnown = true;
    List<String> devices = new ArrayList<>();
    List<String> notCommitted = new ArrayList<>();
    Arrays.fill(pending, false);
    for (OnBoard on : boards) {
      if (on.seen instanceof Seen.WithoutVisor && !on.pushing) {
        continue; // a board without Visor carries none of the cameras
      }
      VisorConfigs.ForBoard wanted = on.wanted;
      if (!(on.seen instanceof Seen.Reported reported) || wanted == null) {
        everyBoardKnown = false;
        continue;
      }
      VisorPack.Report report = reported.report();
      devices.addAll(report.cameras());
      String config = runningConfig(report);
      if (!config.equals(wanted.hash())) {
        for (int camera : wanted.cameras()) {
          pending[camera] = true;
        }
      }
      if (!config.isEmpty() && !config.equals(on.committedHash)) {
        notCommitted.add(on.board.name());
      }
    }
    anyPending = false;
    List<String> waitingCameras = new ArrayList<>();
    for (int camera = 0; camera < pending.length; camera++) {
      if (pending[camera]) {
        anyPending = true;
        waitingCameras.add(now.get(camera).name());
      }
    }
    waiting.setText(
        "Visor's configuration for "
            + String.join(", ", waitingCameras)
            + " reaches its coprocessor once the robot is disabled off the field");
    anyUncommitted = !notCommitted.isEmpty();
    uncommitted.setText(
        "Visor on "
            + String.join(", ", notCommitted)
            + " runs another configuration than the committed one, so the match runs it: a tuned"
            + " exposure or gain, or one that never reached it. Its frames are taken only if the"
            + " robot gave it that one. Put tuned values in Robots.java.");
    // Only once every board with Visor has said what it has can a camera be on none of them.
    List<String> missing = everyBoardKnown ? VisorConfigs.unplaced(now, devices) : List.of();
    unplaced.setText(
        "Visor cameras plugged into no coprocessor: "
            + String.join(", ", missing)
            + ". Plug them in, or check each one's device in Robots.java.");
    unplaced.set(!missing.isEmpty());
  }

  @Override
  public void close() {
    unplaced.close();
    forgetting.close();
    waiting.close();
    uncommitted.close();
    for (OnBoard on : boards) {
      on.failed.close();
      on.working.close();
      on.landing.close();
    }
  }
}
