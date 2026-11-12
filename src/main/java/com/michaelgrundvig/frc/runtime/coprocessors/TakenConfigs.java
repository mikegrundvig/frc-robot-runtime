package com.michaelgrundvig.frc.runtime.coprocessors;

import java.util.ArrayList;
import java.util.List;
import org.jspecify.annotations.Nullable;

/**
 * The configurations one board's Visor frames are taken from, by their IDs ({@link
 * VisorInstaller#otherConfig}): the one wanted now, the committed one, the last one the board was
 * seen running while it was the one wanted, and what was kept before the program restarted ({@link
 * VisorMemory}). What was kept holds only while the committed configuration for the board is the
 * one it was kept with, and only until the board is seen running one wanted since the restart.
 */
final class TakenConfigs {
  private static final int[] NONE = {};

  /** What was kept before the program started; null once it no longer holds, or for none. */
  private VisorMemory.@Nullable Kept recalled;

  /**
   * Its IDs, for the check each frame makes, which allocates nothing; empty once it no longer
   * holds.
   */
  private int[] recalledIds;

  private boolean known;
  private int wantedId;
  private int committedId;
  private boolean ran;
  private int ranId;

  /** What's kept now; null until the board's configurations are known. */
  private VisorMemory.@Nullable Kept kept;

  TakenConfigs(VisorMemory.@Nullable Kept recalled) {
    this.recalled = recalled;
    this.recalledIds = idsOf(recalled);
  }

  /** Whether the board's configurations are known: once what it wants first is. */
  boolean known() {
    return known;
  }

  /**
   * Once what the board wants is worked out again.
   *
   * @param runsWanted whether the board runs the one wanted now, as it reports
   * @return whether what's to be kept changed
   */
  boolean update(int wantedId, int committedId, boolean runsWanted) {
    known = true;
    this.wantedId = wantedId;
    this.committedId = committedId;
    if (runsWanted) {
      ran = true;
      ranId = wantedId;
      // Seen running one wanted since the restart: what was kept before has done its work.
      forget();
    }
    VisorMemory.Kept before = recalled;
    if (before != null && before.committedId() != committedId) {
      // The robot's cameras or lenses changed for this board: what was kept is another robot's.
      forget();
    }
    VisorMemory.Kept now = new VisorMemory.Kept(committedId, ids());
    boolean changed = !now.equals(kept);
    kept = now;
    return changed;
  }

  /** Whether the board's frames from the configuration with this ID are taken. */
  boolean takes(int configId) {
    if (configId == wantedId || configId == committedId) {
      return true;
    }
    if (ran && configId == ranId) {
      return true;
    }
    for (int id : recalledIds) {
      if (id == configId) {
        return true;
      }
    }
    return false;
  }

  private void forget() {
    recalled = null;
    recalledIds = NONE;
  }

  private static int[] idsOf(VisorMemory.@Nullable Kept kept) {
    if (kept == null) {
      return NONE;
    }
    return kept.ids().stream().mapToInt(Integer::intValue).toArray();
  }

  /**
   * What's to be kept for the board: what it's taken from now, once that's known; until then, what
   * was kept before, so a board that hasn't reported since the restart doesn't lose it.
   */
  VisorMemory.@Nullable Kept kept() {
    if (kept != null) {
      return kept;
    }
    return recalled;
  }

  /** The IDs taken now, newest first, each once, at most {@link VisorMemory#MOST_PER_BOARD}. */
  private List<Integer> ids() {
    List<Integer> ids = new ArrayList<>();
    if (ran) {
      add(ids, ranId);
    }
    add(ids, wantedId);
    add(ids, committedId);
    VisorMemory.Kept before = recalled;
    if (before != null) {
      before.ids().forEach(id -> add(ids, id));
    }
    return ids;
  }

  private static void add(List<Integer> ids, int id) {
    if (ids.size() < VisorMemory.MOST_PER_BOARD && !ids.contains(id)) {
      ids.add(id);
    }
  }
}
