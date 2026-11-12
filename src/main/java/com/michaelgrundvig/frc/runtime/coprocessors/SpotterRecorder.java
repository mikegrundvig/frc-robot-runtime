package com.michaelgrundvig.frc.runtime.coprocessors;

import com.michaelgrundvig.frc.spotter.manager.Board;
import com.michaelgrundvig.frc.spotter.manager.Recorder;
import com.michaelgrundvig.frc.spotter.protocol.Spotter;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * What Spotter's manager tells as it happens, on its own threads, of what its boards' state doesn't
 * hold: how many times each board's agent described itself (once on each connect), and how many
 * times it took the robot's packs (after which its agent restarts). The robot loop reads the counts
 * into each board's inputs ({@link BoardIOSpotter}), where AdvantageKit logs them: a replay reads
 * them from the log, never from here. The values themselves are read from the board's state in the
 * loop, so nothing else is kept.
 */
final class SpotterRecorder implements Recorder {
  /**
   * One board's counts. Taken in the order Spotter tells them, so whether a push came after the
   * latest connect is never in doubt, however the loop's reads fall.
   */
  static final class Counts {
    private long described;
    private long pushed;
    private long pushedWhenDescribed;

    synchronized void described() {
      described++;
      pushedWhenDescribed = pushed;
    }

    synchronized void pushed() {
      pushed++;
    }

    /** How many times its agent described itself: once on each connect. */
    synchronized long describes() {
      return described;
    }

    /** How many times it took the robot's packs. */
    synchronized long pushes() {
      return pushed;
    }

    /** How many of those came after its agent last described itself: it restarts with them. */
    synchronized long pushesSinceDescribed() {
      return pushed - pushedWhenDescribed;
    }
  }

  /** Each board's, by its address: made before the manager starts, and only read after. */
  private final Map<String, Counts> boards = new HashMap<>();

  /**
   * @param addresses each board's address, as robot code gave it
   */
  SpotterRecorder(List<String> addresses) {
    for (String address : addresses) {
      this.boards.put(address, new Counts());
    }
  }

  /** A board's counts, by its address as robot code gave it. */
  Counts counts(String address) {
    Counts counts = boards.get(address);
    if (counts == null) {
      throw new IllegalArgumentException("no board " + address);
    }
    return counts;
  }

  @Override
  public void described(Board board, Spotter.Description description) {
    Counts counts = boards.get(board.address());
    if (counts != null) {
      counts.described();
    }
  }

  @Override
  public void pushed(Board board, String packs, boolean forced) {
    Counts counts = boards.get(board.address());
    if (counts != null) {
      counts.pushed();
    }
  }
}
