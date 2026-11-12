package com.michaelgrundvig.frc.runtime.coprocessors;

import com.michaelgrundvig.frc.spotter.manager.Alert;
import com.michaelgrundvig.frc.spotter.manager.Board;
import com.michaelgrundvig.frc.spotter.manager.Connection;
import com.michaelgrundvig.frc.spotter.manager.Level;
import com.michaelgrundvig.frc.spotter.manager.Run;
import com.michaelgrundvig.frc.spotter.manager.Value;
import com.michaelgrundvig.frc.spotter.protocol.Spotter;
import com.michaelgrundvig.frc.visor.VisorPack;
import java.util.List;
import org.jspecify.annotations.Nullable;

/**
 * A Spotter board as the Visor installer sees it, through Visor's Spotter pack ({@link VisorPack}):
 * its status values, and its install and configure actions; whether a push of the robot's packs to
 * it is under way; and its uptime, from the {@code debian} pack its image has.
 *
 * <p>Its report is read again only when Spotter says something on the board changed, so a steady
 * loop reads the same one and builds nothing.
 */
final class SpotterBoards implements VisorInstaller.Board {
  /**
   * The end of the key of Spotter's alert for a board whose packs differ from the robot's and won't
   * be pushed now (Spotter's {@code Alert}): the key is the board's address, then this.
   */
  private static final String PACKS_ALERT = "/packs";

  private final Board board;
  private final SpotterRecorder.Counts counts;
  private final @Nullable String packs;
  private final Value version;
  private final Value config;
  private final Value runningVersion;
  private final Value runningConfig;
  private final Value cameras;
  private final Value uptime;

  private long seenChanges = -1;
  private Connection seenConnection = Connection.CONNECTING;
  private VisorInstaller.Seen seen = VisorInstaller.Seen.SILENT;

  /**
   * @param counts its pushes and connects, from Spotter's recorder
   * @param packs the hash of the robot's packs for it, as its description gives the hash of those
   *     it has; null when the robot has none it can read, so none are pushed
   */
  SpotterBoards(Board board, SpotterRecorder.Counts counts, @Nullable String packs) {
    this.board = board;
    this.counts = counts;
    this.packs = packs;
    this.version = board.value(VisorPack.VERSION);
    this.config = board.value(VisorPack.CONFIG);
    this.runningVersion = board.value(VisorPack.RUNNING_VERSION);
    this.runningConfig = board.value(VisorPack.RUNNING_CONFIG);
    this.cameras = board.value(VisorPack.CAMERAS);
    this.uptime = board.value(BoardIOSpotter.UPTIME);
  }

  @Override
  public String name() {
    return board.name();
  }

  @Override
  public String address() {
    return VisorPlacements.withoutPort(board.address());
  }

  @Override
  public VisorInstaller.Seen seen() {
    if (board.changes() == seenChanges && board.connection() == seenConnection) {
      return seen;
    }
    seenChanges = board.changes();
    seenConnection = board.connection();
    seen = read();
    return seen;
  }

  private VisorInstaller.Seen read() {
    if (board.connection() != Connection.CONNECTED) {
      return VisorInstaller.Seen.SILENT;
    }
    if (!hasVisorPack(board.description())) {
      return VisorInstaller.Seen.WITHOUT_VISOR;
    }
    if (!version.available() || !config.available() || !cameras.available()) {
      return VisorInstaller.Seen.SILENT;
    }
    // A pack older than the one that reports what runs has no running values: not known.
    return new VisorInstaller.Seen.Reported(
        VisorPack.report(
            version.text(),
            config.text(),
            runningVersion.available() ? runningVersion.text() : "",
            runningConfig.available() ? runningConfig.text() : "",
            cameras.text()));
  }

  private static boolean hasVisorPack(Spotter.Description description) {
    for (Spotter.Pack pack : description.getPacks()) {
      if (pack.getName().equals(VisorPack.NAME)) {
        return true;
      }
    }
    return false;
  }

  /**
   * Whether the robot's packs are on their way to it: pushed, and its agent not yet back with them;
   * or differing from its own, with Spotter's manager about to push them (no alert says it won't).
   */
  @Override
  public boolean pushPending() {
    if (counts.pushesSinceDescribed() > 0) {
      return true;
    }
    String robots = packs;
    if (robots == null || board.connection() != Connection.CONNECTED) {
      return false;
    }
    Spotter.Description description = board.description();
    if (description.getPushedPacks().equals(robots) || description.getRefusesPushes()) {
      return false;
    }
    // Spotter says, by an alert, when the packs differ and won't be pushed now: a push that failed,
    // or automatic pushes off. Without one, its manager is pushing them.
    List<Alert> alerts = board.alerts();
    String key = board.address() + PACKS_ALERT;
    for (int i = 0; i < alerts.size(); i++) {
      if (alerts.get(i).key().equals(key)) {
        return false;
      }
    }
    return true;
  }

  @Override
  public double uptimeSeconds() {
    return uptime.available() ? uptime.number() : Double.NaN;
  }

  @Override
  public VisorInstaller.Step install(byte[] input) {
    return new Running(board.run(VisorPack.INSTALL, input));
  }

  @Override
  public VisorInstaller.Step configure(String configuration) {
    return new Running(board.run(VisorPack.CONFIGURE, configuration));
  }

  /** A Spotter run, as one of the installer's steps. */
  private record Running(Run run) implements VisorInstaller.Step {
    @Override
    public boolean done() {
      return run.done();
    }

    /**
     * Worked when its verdict is ok; refused when it ran to the end and said no (Visor refused the
     * configuration, or the release won't run there); otherwise it got no answer: the board wasn't
     * reached, the run timed out, or was lost as the agent restarted.
     */
    @Override
    public VisorInstaller.Outcome outcome() {
      if (run.level() == Level.OK) {
        return VisorInstaller.Outcome.WORKED;
      }
      if (run.state() == Run.State.FINISHED && run.outcome() == Spotter.Outcome.OUTCOME_COMPLETED) {
        return VisorInstaller.Outcome.REFUSED;
      }
      return VisorInstaller.Outcome.NO_ANSWER;
    }

    /** The run's verdict, and the last thing its action said, which is Visor's reason. */
    @Override
    public String why() {
      List<Spotter.LogEntry> log = run.log();
      String said = log.isEmpty() ? "" : ": " + log.get(log.size() - 1).getMessage();
      return run.reason() + said;
    }
  }
}
