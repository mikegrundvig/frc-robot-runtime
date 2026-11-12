package com.michaelgrundvig.frc.runtime.lib.health;

import java.util.Arrays;

/**
 * One camera's frame numbers as its frames arrive, for telling a new frame from one seen before,
 * one that arrived out of order from one sent after a restart, and for the highest number heard.
 *
 * <p>Visor numbers each camera's frames from its start. On a network a datagram can arrive after
 * one sent later, or twice. Such a late frame has a number below the highest and was also taken
 * before the highest frame was. A Visor that restarted counts from 0 again: its numbers go back,
 * but its frames are taken after the ones before. So a number at or below the highest is a restart
 * when its frame was taken later than the highest one, or when it's far below it ({@link
 * #REORDER_FRAMES}, about 17 s at 60 frames a second: no network holds a datagram that long), as
 * when the 32-bit number wraps.
 *
 * <p>It keeps which of the last {@link #REORDER_FRAMES} numbers it has seen, in a fixed set of
 * bits, so nothing is allocated as frames arrive.
 */
public final class FrameNumbers {
  /** How far below the highest number a frame may arrive and still be one sent before it. */
  public static final int REORDER_FRAMES = 1024;

  private static final int WORDS = REORDER_FRAMES / Long.SIZE;
  private static final int SLOT_MASK = REORDER_FRAMES - 1;

  /** How a frame's number stands against the numbers before it. */
  public enum Step {
    /** Higher than any before: the newest frame. The first frame heard is one. */
    NEWEST,
    /** Below the highest, taken before it, and not seen before: it arrived after a later frame. */
    LATE,
    /** Seen before: the same datagram again. */
    SEEN_BEFORE,
    /** The numbering started again, and this is its newest frame. */
    RESTARTED
  }

  /** Which of the window's numbers have been seen, each at its number's slot (number mod size). */
  private final long[] seen = new long[WORDS];

  /** The highest number heard since the last restart; -1 before the first frame. */
  private long highest = -1;

  /** When the highest-numbered frame was taken, by its own stamp. */
  private long highestTakenNanos;

  /**
   * Takes a frame's number, and says how it stands. A frame seen before changes nothing.
   *
   * @param takenNanos when the frame was taken, by its stamp: on one clock from one frame to the
   *     next while its coprocessor runs, whether or not that clock is synced to the robot's
   */
  public Step take(long number, long takenNanos) {
    if (highest < 0) {
      restartAt(number, takenNanos);
      return Step.NEWEST;
    }
    if (number > highest) {
      advanceTo(number, takenNanos);
      return Step.NEWEST;
    }
    if (startedAgain(number, takenNanos)) {
      restartAt(number, takenNanos);
      return Step.RESTARTED;
    }
    if (isSeen(number)) {
      return Step.SEEN_BEFORE;
    }
    mark(number);
    return Step.LATE;
  }

  /** Whether a number at or below the highest is a numbering started again, not a late frame. */
  private boolean startedAgain(long number, long takenNanos) {
    boolean farBelow = highest - number >= REORDER_FRAMES;
    boolean takenAfterTheHighest = takenNanos > highestTakenNanos;
    return farBelow || takenAfterTheHighest;
  }

  /** The highest number heard since the numbering last started again; -1 before any. */
  public long highest() {
    return highest;
  }

  /** Moves the window up to a higher number, forgetting the slots of the numbers it passes. */
  private void advanceTo(long number, long takenNanos) {
    long passed = Math.min(number - highest, REORDER_FRAMES);
    for (long i = 1; i <= passed; i++) {
      clear(highest + i);
    }
    highest = number;
    highestTakenNanos = takenNanos;
    mark(number);
  }

  private void restartAt(long number, long takenNanos) {
    Arrays.fill(seen, 0);
    highest = number;
    highestTakenNanos = takenNanos;
    mark(number);
  }

  private boolean isSeen(long number) {
    int slot = (int) (number & SLOT_MASK);
    return (seen[slot / Long.SIZE] & (1L << (slot % Long.SIZE))) != 0;
  }

  private void mark(long number) {
    int slot = (int) (number & SLOT_MASK);
    seen[slot / Long.SIZE] |= 1L << (slot % Long.SIZE);
  }

  private void clear(long number) {
    int slot = (int) (number & SLOT_MASK);
    seen[slot / Long.SIZE] &= ~(1L << (slot % Long.SIZE));
  }
}
