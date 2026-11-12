package com.michaelgrundvig.frc.runtime.coprocessors;

import java.util.List;
import java.util.Map;

/**
 * Which configurations the Visor installer takes each board's frames from, kept across the robot
 * program's restarts ({@link VisorInstaller#otherConfig}). A restart mid-match, or the power-on at
 * the field, must not forget a configuration the robot gave a board: nothing is configured while
 * the field is attached, so a board still running a tuning pushed before the restart would
 * otherwise be left out for the whole match.
 */
public interface VisorMemory {
  /** The most configuration IDs kept for one board: what fills a board's set, and one spare. */
  int MOST_PER_BOARD = 4;

  /**
   * What's kept for one board.
   *
   * @param committedId the ID of the committed configuration for the board's cameras when they were
   *     kept: they hold only while it's still the committed one, so a robot whose cameras or lenses
   *     changed in its definition forgets them
   * @param ids the configurations its frames are taken from, newest first, at most {@link
   *     #MOST_PER_BOARD}
   */
  record Kept(int committedId, List<Integer> ids) {
    public Kept {
      ids = List.copyOf(ids);
    }
  }

  /** Nothing kept: a robot that can't keep it, and the tests that don't look. */
  VisorMemory NONE =
      new VisorMemory() {
        @Override
        public Map<String, Kept> recalled() {
          return Map.of();
        }

        @Override
        public void keep(Map<String, Kept> boards) {}

        @Override
        public String problem() {
          return "";
        }
      };

  /** What was kept before this program started, by each board's address; empty for nothing. */
  Map<String, Kept> recalled();

  /**
   * Keeps this, by each board's address, in place of what was kept: off the loop, never throwing.
   */
  void keep(Map<String, Kept> boards);

  /** Why the last keep failed; empty while none has. */
  String problem();
}
